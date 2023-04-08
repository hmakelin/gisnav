"""Tests PX4 launch"""
import os
import time
import unittest
from typing import List, Tuple

import pytest
import rclpy
from geographic_msgs.msg import BoundingBox, GeoPointStamped, GeoPoseStamped
from geometry_msgs.msg import PoseStamped, Quaternion
from launch import LaunchDescription  # type: ignore
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest
from mavros_msgs.msg import Altitude, GimbalDeviceAttitudeStatus, HomePosition
from px4_msgs.msg import SensorGps
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, NavSatFix
from std_msgs.msg import Float32

from gisnav_msgs.msg import OrthoImage3D


@pytest.mark.launch_test
def generate_test_description():
    """Generates a PX4 launch description"""
    dirname = os.path.dirname(__file__)
    filename = os.path.join(dirname, "../../launch/px4.launch.py")
    ld = IncludeLaunchDescription(PythonLaunchDescriptionSource(filename))
    return LaunchDescription(
        [
            ld,
            ReadyToTest(),
        ]
    )


class TestPX4Launch(unittest.TestCase):
    """Test that all nodes initialize with correct ROS topics"""

    GISNAV_TOPIC_NAMES_AND_TYPES = [
        ("/gisnav/orthoimage_3d", OrthoImage3D),
        ("/gisnav/bounding_box", BoundingBox),
        ("/gisnav/vehicle_geopose", GeoPoseStamped),
        ("/gisnav/vehicle_geopose/estimate", GeoPoseStamped),
        ("/gisnav/vehicle_altitude", Altitude),
        ("/gisnav/vehicle_altitude/estimate", Altitude),
        ("/gisnav/gimbal_quaternion", Quaternion),
        ("/gisnav/home_geopoint", GeoPointStamped),
        ("/gisnav/terrain_altitude", Altitude),
        ("/gisnav/terrain_geopoint", GeoPointStamped),
        ("/gisnav/egm96_height", Float32),
    ]
    """List of GISNav internal topic names and types"""

    CAMERA_TOPIC_NAMES_AND_TYPES = [
        ("/camera/camera_info", CameraInfo),
        ("/camera/image_raw", Image),
    ]
    """List of camera topic names and types"""

    AUTOPILOT_TOPIC_NAMES_AND_TYPES = [
        ("/mavros/global_position/global", NavSatFix),
        ("/mavros/local_position/pose", PoseStamped),
        ("/mavros/home_position/home", HomePosition),
        ("/mavros/gimbal_control/device/attitude_status", GimbalDeviceAttitudeStatus),
        ("/fmu/in/sensor_gps", SensorGps),
    ]
    """List of autopilot topic names and types"""

    TOPIC_NAMES_AND_TYPES = (
        GISNAV_TOPIC_NAMES_AND_TYPES
        + CAMERA_TOPIC_NAMES_AND_TYPES
        + AUTOPILOT_TOPIC_NAMES_AND_TYPES
    )
    """List of all expected topic names and types"""

    NODE_NAMES_AND_NAMESPACES = {
        ("mock_gps_node", "/"),
        ("bbox_node", "/"),
        ("map_node", "/"),
        ("pose_estimation_node", "/"),
        ("autopilot_node", "/"),
    }
    """List of tuples of node names and namespaces"""

    def __init__(self, *args, **kwargs):
        """Initializer override for declaring attributes used in the test case"""
        super(TestPX4Launch, self).__init__(*args, **kwargs)
        self.test_node = None

    def _get_names_and_namespaces_within_timeout(
        self, timeout: int = 5
    ) -> List[Tuple[str, str]]:
        """Returns node names and namespaces found within timeout period

        :param timeout: Timeout in seconds
        :return: List of tuples containing node name and type
        :raise: :class:`.AssertionError` if names and namespaces are not
            returned within :param timeout_sec:
        """
        names_and_namespaces = None
        end_time = time.time() + timeout
        while time.time() < end_time:  # and names_and_namespaces is None:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
            names_and_namespaces = self.test_node.get_node_names_and_namespaces()

        assert (
            names_and_namespaces is not None
        ), f"Could not get node names and namespaces within {timeout}s timeout."
        return names_and_namespaces

    def _get_topic_names_and_types_within_timeout(
        self, timeout: int = 5
    ) -> List[Tuple[str, str]]:
        """Returns topic names and types found within timeout period

        :param timeout: Timeout in seconds
        :return: List of tuples containing topic name and type
        :raise: :class:`.AssertionError` if names and types are not returned
            within :param timeout_sec:
        """
        names_and_types = None
        end_time = time.time() + timeout
        while time.time() < end_time:  # and names_and_types is None:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
            names_and_types = self.test_node.get_topic_names_and_types()

        assert (
            names_and_types is not None
        ), f"Could not get topic names and types within {timeout}s timeout."
        return names_and_types

    @classmethod
    def setUpClass(cls):
        """Initialize ROS context"""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS context"""
        rclpy.shutdown()

    def setUp(self) -> None:
        """Creates a ROS node for testing"""
        self.test_node = Node("test_node")

    def tearDown(self) -> None:
        """Destroys the ROS node"""
        self.test_node.destroy_node()

    def test_node_names_and_namespaces(self):
        """Tests that nodes are running with correct name and namespace"""
        names, _ = tuple(zip(*self.NODE_NAMES_AND_NAMESPACES))

        found_names_and_namespaces = self._get_names_and_namespaces_within_timeout(10)
        found_names, found_namespaces = tuple(zip(*found_names_and_namespaces))

        assert set(names).issubset(
            found_names
        ), f"Not all nodes ({names}) were discovered ({found_names})."
        for name, namespace in self.NODE_NAMES_AND_NAMESPACES:
            self.assertEqual(
                namespace, dict(found_names_and_namespaces).get(name, None)
            )

    def test_topic_names_and_types(self):
        """Tests that nodes subscribe to and publish the expected ROS topics"""
        names, _ = tuple(zip(*self.TOPIC_NAMES_AND_TYPES))

        found_names_and_types = self._get_topic_names_and_types_within_timeout()
        found_names, found_types = tuple(zip(*found_names_and_types))

        assert set(names).issubset(
            found_names
        ), f"Not all topics ({names}) were discovered ({found_names})."
        for name, type_ in self.TOPIC_NAMES_AND_TYPES:
            types = dict(found_names_and_types).get(name)
            assert types is not None
            self.assertEqual(
                type_.__class__.__name__.replace("Metaclass_", ""),
                types[0].split("/")[-1],
            )


if __name__ == "__main__":
    unittest.main()
