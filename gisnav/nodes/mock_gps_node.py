"""Extends :class:`.BaseNode` to publish mock GPS (GNSS) messages that can substitute real GPS"""
import time
import numpy as np
import rclpy

from datetime import datetime

from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from px4_msgs.msg import SensorGps
from mavros_msgs.msg import GPSINPUT

from gps_time import GPSTime

from gisnav.nodes.base_node import BaseNode
from gisnav.data import FixedCamera
from gisnav.assertions import assert_type


class MockGPSNode(BaseNode):
    """A node that publishes a mock GPS message over the microRTPS bridge"""

    GPS_INPUT_TOPIC_NAME = '/mavros/gps_input/gps_input'
    """Name of ROS topic for outgoing :class:`mavros_msgs.msg.GPSINPUT` messages over MAVROS"""

    SENSOR_GPS_TOPIC_NAME = '/fmu/sensor_gps/in'
    """Name of ROS topic for outgoing :class:`px4_msgs.msg.SensorGps` messages over PX4 microRTPS bridge"""

    def __init__(self, name: str, package_share_dir: str, px4_micrortps: bool = True):
        """Class initializer

        :param name: Node name
        :param package_share_dir: Package share directory
        :param px4_micrortps: Set True to use PX4 microRTPS bridge, MAVROS otherwise
        """
        super().__init__(name, package_share_dir)
        self._px4_micrortps = px4_micrortps
        if self._px4_micrortps:
            self._gps_publisher = self.create_publisher(SensorGps,
                                                        self.SENSOR_GPS_TOPIC_NAME,
                                                        rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)
        else:
            self._gps_publisher = self.create_publisher(GPSINPUT,
                                                        self.GPS_INPUT_TOPIC_NAME,
                                                        rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)

    def publish(self, fixed_camera: FixedCamera) -> None:
        """Publishes drone position as a :class:`px4_msgs.msg.SensorGps` message

        :param fixed_camera: Estimated fixed camera
        """
        assert_type(fixed_camera, FixedCamera)
        msg = self._generate_sensor_gps(fixed_camera) if self._px4_micrortps else self._generate_gps_input(fixed_camera)
        self._gps_publisher.publish(msg)

    def _generate_gps_input(self, fixed_camera: FixedCamera) -> GPSINPUT:
        """Generates a :class:`.GPSINPUT` message to send over MAVROS

        .. seealso:
            `GPS_INPUT_IGNORE_FLAGS <https://mavlink.io/en/messages/common.html#GPS_INPUT_IGNORE_FLAGS>`_
        """
        position = fixed_camera.position
        gps_time = GPSTime.from_datetime(datetime.now())

        ns = time.time_ns()
        sec = int(ns / 1e9)
        nanosec = int(ns - (1e9 * sec))
        header = Header()
        time_ = Time()
        time_.sec = sec
        time_.nanosec = nanosec
        header.stamp = time_

        msg = GPSINPUT()
        msg.header = header
        msg.gps_id = 0
        msg.ignore_flags = 56  # vel_horiz + vel_vert + speed_accuracy
        msg.time_week = gps_time.week_number
        msg.time_week_ms = int(gps_time.time_of_week * 1e3)
        msg.fix_type = 3  # 3D position
        msg.lat = int(position.lat * 1e7)
        msg.lon = int(position.lon * 1e7)
        msg.alt = float(position.z_ellipsoid)
        msg.horiz_accuracy = 10.0  # position.eph
        msg.vert_accuracy = 3.0  # position.epv
        msg.speed_accuracy = np.nan
        msg.hdop = 0.0
        msg.vdop = 0.0
        msg.vn = np.nan
        msg.ve = np.nan
        msg.vd = np.nan
        msg.satellites_visible = np.iinfo(np.uint8).max
        msg.yaw = int(np.degrees(position.attitude.yaw % (2 * np.pi)) * 100)

        return msg

    def _generate_sensor_gps(self, fixed_camera: FixedCamera) -> SensorGps:
        """Generates a :class:`.SensorGps` message to send over PX4 microRTPS brige"""
        position = fixed_camera.position

        msg = SensorGps()
        msg.timestamp = self._bridge.synchronized_time  # position.timestamp
        # msg.timestamp_sample = msg.timestamp
        msg.timestamp_sample = 0
        # msg.device_id = self._generate_device_id()
        msg.device_id = 0
        msg.fix_type = 3
        msg.s_variance_m_s = np.nan
        msg.c_variance_rad = np.nan
        msg.lat = int(position.lat * 1e7)
        msg.lon = int(position.lon * 1e7)
        msg.alt = int(position.z_amsl * 1e3)
        msg.alt_ellipsoid = int(position.z_ellipsoid * 1e3)
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
        msg.heading = position.attitude.yaw
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
