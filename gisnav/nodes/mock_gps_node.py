"""Extends :class:`.BaseNode` to publish mock GPS (GNSS) messages that can substitute real GPS"""
import rclpy
import time
import numpy as np

from px4_msgs.msg import VehicleGpsPosition

from gisnav.nodes.base_node import BaseNode
from gisnav.data import Position


class MockGPSNode(BaseNode):
    """A node that publishes a mock GPS message over the microRTPS bridge"""

    VEHICLE_GPS_POSITION_TOPIC_NAME = 'VehicleGpsPosition_PubSubTopic'

    # ROS 2 QoS profiles for topics
    # TODO: add duration to match publishing frequency, and publish every time (even if NaN)s.
    # If publishing for some reason stops, it can be assumed that something has gone very wrong
    PUBLISH_QOS_PROFILE = rclpy.qos.QoSProfile(history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                               reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                                               depth=1)

    MISC_MOCK_GPS_SELECTION = 1
    """GPS selection to include in mock GPS messages"""

    def __init__(self, name: str, package_share_dir: str):
        """Class initializer"""
        super().__init__(name, package_share_dir)
        self._declare_ros_params()
        self._vehicle_gps_position_publisher = self._create_publisher(self.VEHICLE_GPS_POSITION_TOPIC_NAME,
                                                                      VehicleGpsPosition)

    def publish(self, position: Position) -> None:
        """Publishes position as :class:`px4_msgs.msg.VehicleGpsPosition message and as GeoJSON data"""
        if not isinstance(position, Position) or not all([position.epv, position.eph]):
            self.get_logger().warn('Some required fields required for publishing mock GPS message were None, '
                                   'skipping publishing.')
            return None

        mock_gps_selection = self.get_parameter('misc.mock_gps_selection').get_parameter_value().integer_value
        try:
            self._publish_mock_gps_msg(position, mock_gps_selection)
        except AssertionError as ae:
            self.get_logger().error(f'Assertion error when trying to publish:\n{ae}')

    def _declare_ros_params(self) -> None:
        """Declares ROS parameters

        :return:
        """
        try:
            namespace = 'misc'
            self.declare_parameters(namespace, [
                ('mock_gps_selection', self.MISC_MOCK_GPS_SELECTION)
            ])
        except rclpy.exceptions.ParameterAlreadyDeclaredException as e:
            self.get_logger().warn(str(e))

    def _create_publisher(self, topic_name: str, class_: object) -> rclpy.publisher.Publisher:
        """Sets up an rclpy publisher.

        :param topic_name: Name of the microRTPS topic
        :param class_: Message definition class (e.g. px4_msgs.msg.VehicleGpsPosition)
        :return: The publisher instance
        """
        return self.create_publisher(class_, topic_name, self.PUBLISH_QOS_PROFILE)

    # TODO: get camera_yaw/course estimate?
    def _publish_mock_gps_msg(self, position: Position, selection: int) -> None:
        """Publishes a mock :class:`px4_msgs.msg.VehicleGpsPosition` out of estimated position, velocities and errors.

        :param position: Visually estimated vehicle position and attitude
        :param selection: GPS selection (see :class:`px4_msgs.msg.VehicleGpsPosition` for comment)
        :return:
        """
        assert all([position.eph, position.epv, position.z_amsl])
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
        msg.selected = selection
        self._vehicle_gps_position_publisher.publish(msg)
