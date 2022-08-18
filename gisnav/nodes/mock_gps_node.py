"""Extends :class:`.BaseNode` to publish mock GPS (GNSS) messages that can substitute real GPS"""
import time
import numpy as np
import rclpy

from px4_msgs.msg import SensorGps

from gisnav.nodes.base_node import BaseNode
from gisnav.data import FixedCamera
from gisnav.assertions import assert_type


class MockGPSNode(BaseNode):
    """A node that publishes a mock GPS message over the microRTPS bridge"""

    SENSOR_GPS_TOPIC_NAME = '/fmu/sensor_gps/in'
    """Name of ROS topic for outgoing :class:`px4_msgs.msg.SensorGps` messages"""

    MISC_MOCK_GPS_SELECTION = 1
    """GPS selection parameter for outgoing :class:`px4_msgs.msg.VehicleGpsPosition` messages"""

    def __init__(self, name: str, package_share_dir: str):
        """Class initializer

        :param name: Node name
        :param package_share_dir: Package share directory
        """
        super().__init__(name, package_share_dir)
        self._declare_ros_params()
        self._sensor_gps_publisher = self.create_publisher(SensorGps,
                                                           self.SENSOR_GPS_TOPIC_NAME,
                                                           rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)

    def publish(self, fixed_camera: FixedCamera) -> None:
        """Publishes drone position as a :class:`px4_msgs.msg.SensorGps` message

        :param fixed_camera: Estimated fixed camera
        """
        assert_type(fixed_camera, FixedCamera)
        position = fixed_camera.position

        #mock_gps_selection = self.get_parameter('misc.mock_gps_selection').get_parameter_value().integer_value

        msg = SensorGps()
        msg.timestamp = self._synchronized_time  #position.timestamp
        msg.timestamp_sample = msg.timestamp
        msg.device_id = self._generate_device_id()
        msg.fix_type = 3
        msg.s_variance_m_s = np.nan
        msg.c_variance_rad = np.nan
        msg.lat = int(position.lat * 1e7)
        msg.lon = int(position.lon * 1e7)
        msg.alt = int(position.z_amsl * 1e3)
        msg.alt_ellipsoid = msg.alt
        msg.eph = 10.0 #position.eph
        msg.epv = 3.0 #position.epv
        msg.hdop = 0.0
        msg.vdop = 0.0
        msg.noise_per_ms = np.nan
        msg.automatic_gain_control = np.nan
        msg.jamming_state = 0
        msg.jamming_indicator = 0
        msg.vel_m_s = np.nan
        msg.vel_n_m_s = np.nan
        msg.vel_e_m_s = np.nan
        msg.vel_d_m_s = np.nan
        msg.cog_rad = np.nan
        msg.vel_ned_valid = False
        msg.timestamp_time_relative = 0
        msg.time_utc_usec = time.time() * 1e6
        msg.satellites_used = np.iinfo(np.uint8).max
        msg.time_utc_usec = int(time.time() * 1e6)
        msg.heading = position.attitude.yaw + fixed_camera.image_pair.ref.rotation # np.nan
        msg.heading_offset = np.nan
        msg.heading_accuracy = np.nan
        msg.rtcm_injection_rate = np.nan
        msg.selected_rtcm_instance = np.nan

        self._sensor_gps_publisher.publish(msg)

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

    def _declare_ros_params(self) -> None:
        """Declares ROS parameters"""
        try:
            namespace = 'misc'
            self.declare_parameters(namespace, [
                ('mock_gps_selection', self.MISC_MOCK_GPS_SELECTION)
            ])
            self.get_logger().debug(f'Using default value "{self.MISC_MOCK_GPS_SELECTION}" for ROS parameter '
                                    f'"mock_gps_selection".')
        except rclpy.exceptions.ParameterAlreadyDeclaredException as _:
            # This means parameter is declared from YAML file
            pass
