"""Extends :class:`~MapNavNode` to publish mock GPS (GNSS) messages that can substitute real GPS"""
import rclpy
import time
import traceback
import numpy as np

from typing import Union, get_args
from geojson import Feature, FeatureCollection, Polygon, Point, dump
from px4_msgs.msg import VehicleGpsPosition

from python_px4_ros2_map_nav.assertions import assert_type
from python_px4_ros2_map_nav.nodes.map_nav_node import MapNavNode
from python_px4_ros2_map_nav.data import ImageData, OutputData, LatLon, LatLonAlt
from python_px4_ros2_map_nav.ros_param_defaults import Defaults


class MockGPSNode(MapNavNode):
    """A node that publishes a mock GPS message over the microRTPS bridge"""

    VEHICLE_GPS_POSITION_TOPIC_NAME = 'VehicleGpsPosition_PubSubTopic'

    # ROS 2 QoS profiles for topics
    # TODO: add duration to match publishing frequency, and publish every time (even if NaN)s.
    # If publishing for some reason stops, it can be assumed that something has gone very wrong
    PUBLISH_QOS_PROFILE = rclpy.qos.QoSProfile(history=rclpy.qos.HistoryPolicy.KEEP_LAST,
                                               reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
                                               depth=1)

    def __init__(self, node_name: str):
        super().__init__(node_name)
        self._declare_ros_params()
        self._vehicle_gps_position_publisher = self._create_publisher(self.VEHICLE_GPS_POSITION_TOPIC_NAME,
                                                                      VehicleGpsPosition)

    def publish_position(self, output_data: OutputData) -> None:
        """Publishes position as :class:`px4_msgs.msg.VehicleGpsPosition message and as GeoJSON data"""
        if not all(output_data.position) or any(map(np.isnan, output_data.position)) or \
                not all(output_data.sd) or any(map(np.isnan, output_data.sd)):
            self.get_logger().warn('Some required fields required for publishing mock GPS message were None, '
                                   'skipping publishing.')
            return None

        mock_gps_selection = self.get_parameter('misc.mock_gps_selection').get_parameter_value().integer_value
        try:
            self._publish_mock_gps_msg(output_data.position, output_data.sd, mock_gps_selection)
        except AssertionError as ae:
            self.get_logger().error(f'Assertion error when trying to publish:\n{ae}')
        export_geojson = self.get_parameter('misc.export_position').get_parameter_value().string_value
        if export_geojson is not None:
            self._export_position(output_data.position, output_data.fov, export_geojson)

    def publish_projected_fov(self, fov: np.ndarray, c: np.ndarray) -> None:
        """Writes field of view (FOV) and map center into GeoJSON file"""
        # Export to file in GIS readable format
        export_projection = self.get_parameter('misc.export_projection').get_parameter_value().string_value
        if export_projection is not None:
            self._export_position(c, fov, export_projection)

    def _declare_ros_params(self) -> None:
        """Declares any additional ROS parameters that are not declared by the base class

        Note: does not override parent method which uses name mangling (double underscore).

        :return:
        """
        namespace = 'misc'
        self.declare_parameters(namespace, [
            ('mock_gps_selection', Defaults.MISC_MOCK_GPS_SELECTION),
            ('export_position', Defaults.MISC_EXPORT_POSITION),
            ('export_projection', Defaults.MISC_EXPORT_PROJECTION)
        ])

    def _create_publisher(self, topic_name: str, class_: object) -> rclpy.publisher.Publisher:
        """Sets up an rclpy publisher.

        :param topic_name: Name of the microRTPS topic
        :param class_: Message definition class (e.g. px4_msgs.msg.VehicleGpsPosition)
        :return: The publisher instance
        """
        return self.create_publisher(class_, topic_name, self.PUBLISH_QOS_PROFILE)

    # TODO: get camera_yaw/course estimate?
    def _publish_mock_gps_msg(self, latlonalt: np.ndarray, sd: np.ndarray, selection: int) -> None:
        """Publishes a mock :class:`px4_msgs.msg.VehicleGpsPosition` out of estimated position, velocities and errors.

        :param latlonalt: Estimated vehicle position
        :param sd: Estimated x, y, z position standard deviations
        :param selection: GPS selection (see :class:`px4_msgs.msg.VehicleGpsPosition` for comment)
        :return:
        """
        # TODO: check inputs?
        msg = VehicleGpsPosition()
        msg.timestamp = self._get_ekf2_time()
        msg.fix_type = 3
        msg.s_variance_m_s = np.nan
        msg.c_variance_rad = np.nan
        msg.lat = int(latlonalt[0] * 1e7)
        msg.lon = int(latlonalt[1] * 1e7)
        msg.alt = int(latlonalt[2] * 1e3)
        msg.alt_ellipsoid = msg.alt
        msg.eph = max(sd[0:2])
        msg.epv = sd[2]
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
        #self._vehicle_gps_position_publisher.publish(msg)

    def _export_position(self, position: Union[LatLon, LatLonAlt], fov: np.ndarray, filename: str) -> None:
        """Exports the computed position and field of view (FOV) into a geojson file.

        The GeoJSON file is not used by the node but can be accessed by GIS software to visualize the data it contains.

        :param position: Computed camera position
        :param: fov: Field of view of camera
        :param filename: Name of file to write into
        :return:
        """
        assert_type(position, get_args(Union[LatLon, LatLonAlt]))
        assert_type(fov, np.ndarray)
        assert_type(filename, str)
        point = Feature(geometry=Point((position.lon, position.lat)))
        corners = np.flip(fov.squeeze()).tolist()
        corners = [tuple(x) for x in corners]
        corners = Feature(geometry=Polygon([corners]))
        features = [point, corners]
        feature_collection = FeatureCollection(features)
        try:
            with open(filename, 'w') as f:
                dump(feature_collection, f)
        except Exception as e:
            self.get_logger().error(f'Could not write file {filename} because of exception:'
                                    f'\n{e}\n{traceback.print_exc()}')
