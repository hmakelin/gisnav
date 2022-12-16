"""Extends :class:`.BridgeNode` to publish mock GPS (GNSS) messages that can substitute real GPS"""
import time
import numpy as np
import socket
import json

from rclpy.qos import QoSPresetProfiles

from typing import Optional
from datetime import datetime

from px4_msgs.msg import SensorGps
from mavros_msgs.msg import GPSINPUT, Altitude
from geographic_msgs.msg import GeoPoseStamped

from gps_time import GPSTime

from . import messaging
from gisnav.nodes.base.base_node import BaseNode
from gisnav.data import Attitude
from gisnav.assertions import assert_type


class MockGPSNode(BaseNode):
    """A node that publishes a mock GPS message over the microRTPS bridge"""
    ROS_D_UDP_IP = "127.0.0.1"
    """MAVProxy GPSInput plugin default host"""

    ROS_D_UDP_PORT = 25100
    """MAVProxy GPSInput plugin default port"""

    ROS_D_PX4_MICRORTPS = True
    """True for PX4 microRTPS bridge, False for ArduPilot MAVROS"""

    ROS_PARAM_DEFAULTS = [
        ('udp_ip', ROS_D_UDP_IP, True),
        ('udp_port', ROS_D_UDP_PORT, True),
        ('px4_micrortps', ROS_D_PX4_MICRORTPS, True),
    ]
    """List containing ROS parameter name, default value and read_only flag tuples"""

    def __init__(self, name: str):
        """Class initializer

        :param name: Node name
        """
        super().__init__(name)

        # MAVROS only
        self._udp_ip = self.get_parameter('udp_ip').get_parameter_value().string_value
        self._udp_port = self.get_parameter('udp_port').get_parameter_value().integer_value

        self._px4_micrortps = self.get_parameter('px4_micrortps').get_parameter_value().bool_value
        if self._px4_micrortps:
            self._mock_gps_pub = self.create_publisher(SensorGps,
                                                       messaging.ROS_TOPIC_SENSOR_GPS,
                                                       QoSPresetProfiles.SENSOR_DATA.value)
            self._socket = None
        else:
            # TODO: try to get MAVROS to work for GPSINPUT message and get rid of UDP socket
            #self._mock_gps_pub = self.create_publisher(GPSINPUT,
            #                                           messaging.ROS_TOPIC_GPS_INPUT,
            #                                           QoSPresetProfiles.SENSOR_DATA.value)
            self._mock_gps_pub = None
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._vehicle_geopose_estimate_sub = self.create_subscription(GeoPoseStamped,
                                                                      messaging.ROS_TOPIC_VEHICLE_GEOPOSE_ESTIMATE,
                                                                      self._vehicle_geopose_estimate_callback,
                                                                      QoSPresetProfiles.SENSOR_DATA.value)
        self._vehicle_altitude_estimate_sub = self.create_subscription(Altitude,
                                                                       messaging.ROS_TOPIC_VEHICLE_ALTITUDE_ESTIMATE,
                                                                       self._vehicle_altitude_estimate_callback,
                                                                       QoSPresetProfiles.SENSOR_DATA.value)
        self._geopose_estimate = None
        self._altitude_estimate = None

    def _vehicle_geopose_estimate_callback(self, msg: GeoPoseStamped) -> None:
        """Handles latest geopose estimate

        :param msg: Latest :class:`geographic_msgs.msg.GeoPose` message
        """
        self._geopose_estimate = msg
        if self._altitude_estimate is not None:
            lat, lon = msg.pose.position.latitude, msg.pose.position.longitude
            alt_amsl = self._altitude_estimate.amsl
            alt_ellipsoid = msg.pose.position.altitude
            q = messaging.as_np_quaternion(msg.pose.orientation)
            yaw = Attitude(q=q).yaw
            timestamp = messaging.usec_from_header(msg.header)
            self._publish(lat, lon, alt_amsl, alt_ellipsoid, yaw, timestamp)
        else:
            self.get_logger().warn('Altitude estimate not yet received, skipping publishing mock GPS message.')

    def _vehicle_altitude_estimate_callback(self, msg: Altitude) -> None:
        """Handles latest altitude message

        :param msg: Latest :class:`mavros_msgs.msg.Altitude` message
        """
        self._altitude_estimate = msg

    def _publish(self, lat, lon, altitude_amsl, altitude_ellipsoid, heading, timestamp) -> None:
        """Publishes drone position as a :class:`px4_msgs.msg.SensorGps` message

        :param lat: Vehicle latitude
        :param lon: Vehicle longitude
        :param altitude_amsl: Vehicle altitude AMSL
        :param altitude_ellipsoid: Vehicle altitude above WGS 84 ellipsoid
        :param heading: Vehicle heading in radians
        :param timestamp: Timestamp for outgoing mock GPS message (e.g. system time)

        """
        if self._px4_micrortps:
            msg: SensorGps = self._generate_sensor_gps(lat, lon, altitude_amsl, altitude_ellipsoid, heading, timestamp)
        else:
            msg: dict = self._generate_gps_input(lat, lon, altitude_amsl, heading, timestamp)

        if msg is not None:
            if self._px4_micrortps:
                assert_type(msg, SensorGps)
                assert self._socket is None
                self._mock_gps_pub.publish(msg)
            else:
                assert_type(msg, dict)
                assert self._mock_gps_pub is None
                self._socket.sendto(f'{json.dumps(msg)}'.encode('utf-8'), (self._udp_ip, self._upd_port))
        else:
            self.get_logger().info('Could not create GPS message, skipping publishing.')

    def _generate_gps_input(self, lat, lon, altitude_amsl, heading, timestamp) -> Optional[dict]:
        """Generates a :class:`.GPSINPUT` message to send over MAVROS

        .. note::
            Currently the message is sent directly over a UDP socket so the GPS_INPUT is returned as a Python dict,
            not as a :class:`mavros_msgs.msg.GPSINPUT`

        .. seealso:
            `GPS_INPUT_IGNORE_FLAGS <https://mavlink.io/en/messages/common.html#GPS_INPUT_IGNORE_FLAGS>`_

        :param lat: Vehicle latitude
        :param lon: Vehicle longitude
        :param altitude_amsl: Vehicle altitude in meters AMSL
        :param heading: Vehicle heading in radians
        :param timestamp: System time in microseconds
        :return: MAVLink GPS_INPUT message as Python dict
        """
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

    def _generate_sensor_gps(self, lat, lon, altitude_amsl, altitude_ellipsoid, heading, timestamp) -> Optional[SensorGps]:
        """Generates a :class:`.SensorGps` message to send over PX4 microRTPS brige

        :param lat: Vehicle latitude
        :param lon: Vehicle longitude
        :param altitude_amsl: Vehicle altitude in meters AMSL
        :param altitude_ellipsoid: Vehicle altitude in meters above WGS 84 ellipsoid
        :param heading: Vehicle heading in radians
        :param timestamp: System time in microseconds
        """
        msg = SensorGps()
        msg.timestamp = timestamp
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

    def destroy_node(self) -> None:
        """Closes GPS_INPUT MAVLink UDP socket when node is destroyed"""
        if not self._px4_micrortps:
            assert self._socket is not None
            self.get_logger().info('Closing UDP socket...')
            self._socket.close()

        # socket closed - call parent method
        super().destroy_node()
