"""Tests :class:`.MockGPSNode"""
import multiprocessing.pool
import unittest
import os
import time

import rclpy
import pytest

from unittest.mock import patch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest

from gisnav.data import PackageData
from gisnav.nodes.mock_gps_node import MockGPSNode


@pytest.mark.launch_test
def generate_test_description():
    """Generates a :class:`.MockGPSNode` launch description"""
    dirname = os.path.dirname(__file__)
    filename = os.path.join(dirname, '../launch/mock_gps_node.launch.py')
    ld = IncludeLaunchDescription(PythonLaunchDescriptionSource(filename))

    return LaunchDescription([
        ld,
        ReadyToTest(),
    ])


class TestInit(unittest.TestCase):
    """Tests that :class:`.MockGPSNode` initializes correctly"""

    NODE_NAME = 'mock_gps_node'
    """This node name should match with node name in mock_gps_node.launch.py"""

    NODE_NAMESPACE = '/'
    """Expected node namespace"""

    SUBSCRIBER_NAMES_AND_TYPES = [
        ('/GimbalDeviceSetAttitude_PubSubTopic', ['px4_msgs/msg/GimbalDeviceSetAttitude']),
        ('/VehicleAttitude_PubSubTopic', ['px4_msgs/msg/VehicleAttitude']),
        ('/VehicleGlobalPosition_PubSubTopic', ['px4_msgs/msg/VehicleGlobalPosition']),
        ('/VehicleLocalPosition_PubSubTopic', ['px4_msgs/msg/VehicleLocalPosition']),
        ('/camera_info', ['sensor_msgs/msg/CameraInfo']),
        ('/image_raw', ['sensor_msgs/msg/Image'])
    ]
    """Expected node subscribe topic names and message types"""

    @staticmethod
    def mock_setup_wms_pool(self) -> multiprocessing.pool.Pool:
        """Returns a mock pool without attempting to connect to WMS endpoint"""
        return multiprocessing.pool.Pool()

    @classmethod
    def setUpClass(cls):
        """Initialize ROS context"""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS context"""
        rclpy.shutdown()

    @patch.multiple(MockGPSNode,
                    _setup_wms_pool=mock_setup_wms_pool
    )
    def setUp(self) -> None:
        """Creates the ROS node"""
        dirname = os.path.dirname(__file__)
        filename = os.path.join(dirname, '../package.xml')
        package_data = PackageData.parse_package_data(os.path.abspath(filename))
        self.node = MockGPSNode(self.NODE_NAME, get_package_share_directory(package_data.package_name))

    def tearDown(self) -> None:
        """Destroys the ROS node"""
        self.node.destroy_timers()
        self.node.terminate_pools()
        self.node.destroy_node()

    def test_node_names_and_namespaces(self):
        """Tests that :class:`.MockGPSNode` is running with the correct name and namespace"""
        names_and_namespaces = None
        timeout_sec = 2
        end_time = time.time() + timeout_sec
        while time.time() < end_time and names_and_namespaces is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            names_and_namespaces = self.node.get_node_names_and_namespaces()

        assert names_and_namespaces is not None, f'Could not determine node name and namespace within timeout of ' \
                                                 f'{timeout_sec} seconds.'

        name, namespace = names_and_namespaces[0]

        assert name == self.NODE_NAME, f'Node name "{name}" did not match expectation "{self.NODE_NAME}".'
        assert namespace == self.NODE_NAMESPACE, f'Node namespace "{namespace}" did not match expectation ' \
                                                 f'"{self.NODE_NAMESPACE}".'

    def test_subscriber_names_and_types(self):
        """Tests that parent class :class:`.BaseNode` subscribes to the correct ROS topics"""
        subscriber_names_and_types = None
        timeout_sec = 2
        end_time = time.time() + timeout_sec
        while time.time() < end_time and subscriber_names_and_types is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            subscriber_names_and_types = self.node.get_subscriber_names_and_types_by_node(self.NODE_NAME,
                                                                                          self.NODE_NAMESPACE)

        assert subscriber_names_and_types is not None, f'Could not determine subscriber names and types within ' \
                                                       f'timeout of {timeout_sec} seconds.'

        for name, type_ in subscriber_names_and_types:
            assert (name, type_) in self.SUBSCRIBER_NAMES_AND_TYPES, f'Unexpected subscription for topic name ' \
                                                                     f'"{name}" with type "{type_}".'

        for name, type_ in self.SUBSCRIBER_NAMES_AND_TYPES:
            assert (name, type_) in subscriber_names_and_types, f'Expected subscription for topic name "{name}" with ' \
                                                                f'type "{type_}" was not found in node subscriptions.'


if __name__ == '__main__':
    unittest.main()