"""Extends :class:`.BridgeNode` to publish mock GPS (GNSS) messages that can substitute real GPS"""
import time
import sys
import numpy as np
import rclpy
import socket
import json

from rclpy.qos import QoSPresetProfiles
from rclpy.node import Node

from typing import Optional, Union
from datetime import datetime

from px4_msgs.msg import SensorGps
from mavros_msgs.msg import GPSINPUT, Altitude as ROSAltitude
from geographic_msgs.msg import GeoPoint as ROSGeoPoint, GeoPoseStamped as ROSGeoPoseStamped

from gps_time import GPSTime

from . import messaging

from gisnav.data import Attitude
from gisnav.assertions import assert_type


class MockGPSNode(Node):
    """A node that publishes a mock GPS message over the microRTPS bridge"""

    GPS_INPUT_TOPIC_NAME = '/mavros/gps_input/gps_input'
    """Name of ROS topic for outgoing :class:`mavros_msgs.msg.GPSINPUT` messages over MAVROS"""

    SENSOR_GPS_TOPIC_NAME = '/fmu/sensor_gps/in'
    """Name of ROS topic for outgoing :class:`px4_msgs.msg.SensorGps` messages over PX4 microRTPS bridge"""

    UDP_IP = "127.0.0.1"
    """MAVProxy GPSInput plugin host"""

    UDP_PORT = 25100
    """MAVProxy GPSInput plugin port"""

    def __init__(self, name: str, px4_micrortps: bool = True):
        """Class initializer

        :param name: Node name
        :param px4_micrortps: Set True to use PX4 microRTPS bridge, MAVROS otherwise
        """
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self._px4_micrortps = px4_micrortps
        if self._px4_micrortps:
            self._gps_publisher = self.create_publisher(SensorGps,
                                                        self.SENSOR_GPS_TOPIC_NAME,
                                                        rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
            self._socket = None
        else:
            self._gps_publisher = self.create_publisher(GPSINPUT,
                                                        self.GPS_INPUT_TOPIC_NAME,
                                                        rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        #self._geopoint_sub = self.create_subscription(ROSGeoPoint, "geopoint_estimate",
        #                                              self._geopoint_callback,
        #                                              QoSPresetProfiles.SENSOR_DATA.value)
        self._geopose_sub = self.create_subscription(ROSGeoPoseStamped, "geopose_estimate",
                                                     self._geopose_callback,
                                                     QoSPresetProfiles.SENSOR_DATA.value)
        self._altitude_sub = self.create_subscription(ROSAltitude, "altitude_estimate",
                                                      self._altitude_callback,
                                                      QoSPresetProfiles.SENSOR_DATA.value)
        self._geopose_estimate = None
        self._altitude_estimate = None

    #def _geopoint_callback(self, msg: ROSGeoPoint) -> None:
    #    """Handles latest geopoint estimate
    #
    #    :param msg: Latest :class:`geographic_msgs.msg.GeoPoint` message
    #    """
    #    self._geopoint_estimate = msg

    def _geopose_callback(self, msg: ROSGeoPoseStamped) -> None:
        """Handles latest geopose estimate

        :param msg: Latest :class:`geographic_msgs.msg.GeoPose` message
        """
        self._geopose_estimate = msg
        if self._altitude_estimate is not None:
            lat, lon = msg.pose.position.latitude, msg.pose.position.longitude
            alt_amsl = self._altitude_estimate.amsl
            alt_ellipsoid = msg.pose.position.altitude
            attitude = msg.pose.orientation
            q_xyzw = np.array([attitude.x, attitude.y, attitude.z, attitude.w])
            yaw = Attitude(q=q_xyzw).yaw
            timestamp = messaging.usec_from_header(msg.header)
            self._publish(lat, lon, alt_amsl, alt_ellipsoid, yaw, timestamp)
        else:
            self.get_logger().warn('Altitude estimate not yet received, skipping publishing mock GPS message.')

    def _altitude_callback(self, msg: ROSAltitude) -> None:
        """Handles latest altitude message

        :param msg: Latest :class:`mavros_msgs.msg.Altitude` message
        """
        self._altitude_estimate = msg

    #def publish(self, fixed_camera: FixedCamera) -> None:
    def _publish(self, lat, lon, altitude_amsl, altitude_ellipsoid, heading, timestamp) -> None:
        """Publishes drone position as a :class:`px4_msgs.msg.SensorGps` message

        :param fixed_camera: Estimated fixed camera
        """
        #assert_type(fixed_camera, FixedCamera)
        msg: Optional[Union[dict, SensorGps]] = self._generate_sensor_gps(lat, lon, altitude_amsl, altitude_ellipsoid, heading, timestamp) if self._px4_micrortps \
            else self._generate_gps_input(lat, lon, altitude_amsl, heading, timestamp)

        if msg is not None:
            if self._px4_micrortps:
                assert_type(msg, SensorGps)
                self._gps_publisher.publish(msg)
            else:
                assert_type(msg, dict)
                self._socket.sendto(f'{json.dumps(msg)}'.encode('utf-8'), (self.UDP_IP, self.UDP_PORT))
        else:
            self.get_logger().info('Could not create GPS message, skipping publishing.')

    #def _generate_gps_input(self, fixed_camera: FixedCamera) -> Optional[dict]:
    def _generate_gps_input(self, lat, lon, altitude_amsl, heading, timestamp) -> Optional[dict]:
        """Generates a :class:`.GPSINPUT` message to send over MAVROS

        .. seealso:
            `GPS_INPUT_IGNORE_FLAGS <https://mavlink.io/en/messages/common.html#GPS_INPUT_IGNORE_FLAGS>`_
        """
        #position = fixed_camera.position

        #if position.altitude.amsl is None:
        #    self.get_logger().warn(f'AMSL altitude not estimated ({position.altitude}).')
        #    return

        msg = {}

        # Adjust UTC epoch timestamp for estimation delay
        msg['usec'] = timestamp # int(time.time_ns() / 1e3) - (self._bridge.synchronized_time - timestamp)
        msg['gps_id'] = 0
        msg['ignore_flags'] = 56  # vel_horiz + vel_vert + speed_accuracy

        gps_time = GPSTime.from_datetime(datetime.utcfromtimestamp(msg['usec'] / 1e6))
        msg['time_week'] = gps_time.week_number
        msg['time_week_ms'] = int(gps_time.time_of_week * 1e3)  # TODO this implementation accurate only up to 1 second
        msg['fix_type'] = 3  # 3D position
        msg['lat'] = int(lat * 1e7)
        msg['lon'] = int(lon * 1e7)
        msg['alt'] = altitude_amsl  # ArduPilot Gazebo SITL expects AMSL
        msg['horiz_accuracy'] = 10.0  # position.eph
        msg['vert_accuracy'] = 3.0  # position.epv
        msg['speed_accuracy'] = np.nan # should be in ignore_flags
        msg['hdop'] = 0.0
        msg['vdop'] = 0.0
        msg['vn'] = np.nan  # should be in ignore_flags
        msg['ve'] = np.nan  # should be in ignore_flags
        msg['vd'] = np.nan  # should be in ignore_flags
        msg['satellites_visible'] = np.iinfo(np.uint8).max

        # TODO check yaw sign (NED or ENU?)
        yaw = int(np.degrees(heading % (2 * np.pi)) * 100)
        yaw = 36000 if yaw == 0 else yaw  # MAVLink definition 0 := not available
        msg['yaw'] = yaw

        return msg

    #def _generate_sensor_gps(self, fixed_camera: FixedCamera) -> Optional[SensorGps]:
    def _generate_sensor_gps(self, lat, lon, altitude_amsl, altitude_ellipsoid, heading, timestamp) -> Optional[SensorGps]:
        """Generates a :class:`.SensorGps` message to send over PX4 microRTPS brige"""
        #position = fixed_camera.position

        #if position.altitude.amsl is None:
        #    self.get_logger().warn(f'AMSL altitude not estimated ({position.altitude}).')
        #    return

        msg = SensorGps()
        msg.timestamp = timestamp  # self._bridge.synchronized_time  # position.timestamp
        # msg.timestamp_sample = msg.timestamp
        msg.timestamp_sample = 0
        # msg.device_id = self._generate_device_id()
        msg.device_id = 0
        msg.fix_type = 3
        msg.s_variance_m_s = np.nan
        msg.c_variance_rad = np.nan
        msg.lat = int(lat * 1e7)
        msg.lon = int(lon * 1e7)
        msg.alt = int(altitude_amsl * 1e3)
        msg.alt_ellipsoid = int(altitude_ellipsoid * 1e3)
        msg.eph = 10.0  # position.eph
        msg.epv = 3.0  # position.epv
        msg.hdop = 0.0
        msg.vdop = 0.0
        msg.noise_per_ms = 0
        msg.automatic_gain_control = 0
        msg.jamming_state = 0
        msg.jamming_indicator = 0
        msg.vel_m_s = np.nan
        msg.vel_n_m_s = np.nan
        msg.vel_e_m_s = np.nan
        msg.vel_d_m_s = np.nan
        msg.cog_rad = np.nan
        msg.vel_ned_valid = False
        msg.timestamp_time_relative = 0
        msg.time_utc_usec = int(time.time() * 1e6)
        msg.satellites_used = np.iinfo(np.uint8).max
        msg.time_utc_usec = int(time.time() * 1e6)
        msg.heading = heading
        msg.heading_offset = np.nan
        # msg.heading_accuracy = np.nan
        # msg.rtcm_injection_rate = np.nan
        # msg.selected_rtcm_instance = np.nan

        return msg

    def _generate_device_id(self) -> int:
        """Generates a device ID for the outgoing `px4_msgs.SensorGps` message"""
        # For reference, see:
        # https://docs.px4.io/main/en/middleware/drivers.html and
        # https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/drv_sensor.h
        # https://docs.px4.io/main/en/gps_compass/

        # DRV_GPS_DEVTYPE_SIM (0xAF) + dev 1 + bus 1 + DeviceBusType_UNKNOWN
        # = 10101111 00000001 00001 000
        # = 11469064
        return 11469064

    def unsubscribe_topics(self) -> None:
        """Unsubscribes ROS topics and closes GPS_INPUT MAVLink UDP socket"""
        #super().unsubscribe_topics()
        if not self._px4_micrortps:
            self.get_logger().info('Closing UDP socket.')
            self._socket.close()
