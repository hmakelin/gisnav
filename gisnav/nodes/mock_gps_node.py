"""Extends :class:`.BaseNode` to publish mock GPS (GNSS) messages that can substitute real GPS"""
import rclpy
import time
import numpy as np

from px4_msgs.msg import VehicleGpsPosition

from gisnav.nodes.base_node import BaseNode
from gisnav.data import Position
from gisnav.assertions import assert_type


class MockGPSNode(BaseNode):
    """A node that publishes a mock GPS message over the microRTPS bridge"""

    VEHICLE_GPS_POSITION_TOPIC_NAME = 'VehicleGpsPosition_PubSubTopic'

    MISC_MOCK_GPS_SELECTION = 1
    """GPS selection parameter for outgoing :class:`px4_msgs.msg.VehicleGpsPosition` messages"""

    def __init__(self, name: str, package_share_dir: str):
        """Class initializer

        :param name: Node name
        :param package_share_dir: Package share directory
        """
        super().__init__(name, package_share_dir)
        self._declare_ros_params()
        self._vehicle_gps_position_publisher = self.create_publisher(VehicleGpsPosition,
                                                                     self.VEHICLE_GPS_POSITION_TOPIC_NAME,
                                                                     rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)

    def publish(self, position: Position) -> None:
        """Publishes position as :class:`px4_msgs.msg.VehicleGpsPosition` message"""
        assert_type(position, Position)

        mock_gps_selection = self.get_parameter('misc.mock_gps_selection').get_parameter_value().integer_value

        msg = VehicleGpsPosition()
        msg.timestamp = position.timestamp
        msg.fix_type = 3
        msg.s_variance_m_s = np.nan
        msg.c_variance_rad = np.nan
        msg.lat = int(position.lat * 1e7)
        msg.lon = int(position.lon * 1e7)
        msg.alt = int(position.z_amsl * 1e3)
        msg.alt_ellipsoid = msg.alt
        msg.eph = position.eph
        msg.epv = position.epv
        msg.hdop = 0.0
        msg.vdop = 0.0
        msg.vel_m_s = np.nan
        msg.vel_n_m_s = np.nan
        msg.vel_e_m_s = np.nan
        msg.vel_d_m_s = np.nan
        msg.cog_rad = np.nan
        msg.vel_ned_valid = False
        msg.satellites_used = np.iinfo(np.uint8).max
        msg.time_utc_usec = int(time.time() * 1e6)
        msg.heading = np.nan
        msg.heading_offset = np.nan
        msg.selected = mock_gps_selection

        self._vehicle_gps_position_publisher.publish(msg)

    def _declare_ros_params(self) -> None:
        """Declares ROS parameters"""
        try:
            namespace = 'misc'
            self.declare_parameters(namespace, [
                ('mock_gps_selection', self.MISC_MOCK_GPS_SELECTION)
            ])
        except rclpy.exceptions.ParameterAlreadyDeclaredException as e:
            self.get_logger().warn(str(e))
