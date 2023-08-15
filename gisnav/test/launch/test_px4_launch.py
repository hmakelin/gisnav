"""Tests :term:`ROS` launch file for :term:`PX4` configuration"""
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
from nav_msgs.msg import Path
from px4_msgs.msg import SensorGps
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, NavSatFix
from std_msgs.msg import Float32

from gisnav.static_configuration import (
    CV_NODE_NAME,
    GIS_NODE_NAME,
    ROS_NAMESPACE,
    ROS_TOPIC_RELATIVE_CAMERA_QUATERNION,
    ROS_TOPIC_RELATIVE_GROUND_TRACK_ELEVATION,
    ROS_TOPIC_RELATIVE_GROUND_TRACK_GEOPOSE,
    ROS_TOPIC_RELATIVE_ORTHOIMAGE,
    ROS_TOPIC_RELATIVE_VEHICLE_ALTITUDE,
    ROS_TOPIC_RELATIVE_VEHICLE_ESTIMATED_ALTITUDE,
    ROS_TOPIC_RELATIVE_VEHICLE_ESTIMATED_GEOPOSE,
    ROS_TOPIC_RELATIVE_VEHICLE_GEOPOSE,
)
from gisnav_msgs.msg import OrthoImage3D  # type: ignore


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


class TestComputationalGraphCase(unittest.TestCase):
    """Tests that all nodes initialize with the correct :term:`ROS` computational
    graph structure"""

    GIS_NODE_TOPIC_NAMES_AND_TYPES = [
        (
            f'/{ROS_NAMESPACE}/{ROS_TOPIC_RELATIVE_ORTHOIMAGE.replace("~", GIS_NODE_NAME)}',
            OrthoImage3D,
        ),
        (
            f'/{ROS_NAMESPACE}/{ROS_TOPIC_RELATIVE_VEHICLE_GEOPOSE.replace("~", GIS_NODE_NAME)}',
            GeoPoseStamped,
        ),
        (
            f'/{ROS_NAMESPACE}/{ROS_TOPIC_RELATIVE_VEHICLE_ALTITUDE.replace("~", GIS_NODE_NAME)}',
            Altitude,
        ),
        (
            f'/{ROS_NAMESPACE}/{ROS_TOPIC_RELATIVE_CAMERA_QUATERNION.replace("~", GIS_NODE_NAME)}',
            Quaternion,
        ),
        (
            f'/{ROS_NAMESPACE}/{ROS_TOPIC_RELATIVE_GROUND_TRACK_ELEVATION.replace("~", GIS_NODE_NAME)}',
            Altitude,
        ),
        (
            f'/{ROS_NAMESPACE}/{ROS_TOPIC_RELATIVE_GROUND_TRACK_GEOPOSE.replace("~", GIS_NODE_NAME)}',
            GeoPoseStamped,
        ),
    ]
    """List of :class:`.GISNode` published topic names and types"""

    CV_NODE_TOPIC_NAMES_AND_TYPES = [
        (
            f'/{ROS_NAMESPACE}/{ROS_TOPIC_RELATIVE_VEHICLE_ESTIMATED_GEOPOSE.replace("~", CV_NODE_NAME)}',
            GeoPoseStamped,
        ),
        (
            f'/{ROS_NAMESPACE}/{ROS_TOPIC_RELATIVE_VEHICLE_ESTIMATED_ALTITUDE.replace("~", CV_NODE_NAME)}',
            Altitude,
        ),
    ]
    """List of :class:`.CVNode` published topic names and types"""

    CAMERA_TOPIC_NAMES_AND_TYPES = [
        ("/camera/camera_info", CameraInfo),
        ("/camera/image_raw", Image),
    ]
    """List of camera topic names and types"""

    MAVROS_TOPIC_NAMES_AND_TYPES = [
        ("/mavros/global_position/global", NavSatFix),
        ("/mavros/local_position/pose", PoseStamped),
        ("/mavros/home_position/home", HomePosition),
        ("/mavros/gimbal_control/device/attitude_status", GimbalDeviceAttitudeStatus),
        ("/fmu/in/sensor_gps", SensorGps),
    ]
    """List of :term:`MAVROS` published topic names and types"""

    UROS_TOPIC_NAMES_AND_TYPES = [
        ("/fmu/in/sensor_gps", SensorGps),
    ]
    """List of :term:`micro-ros-agent` subscribed topic names and types"""

    TOPIC_NAMES_AND_TYPES = (
        GIS_NODE_TOPIC_NAMES_AND_TYPES
        + CV_NODE_TOPIC_NAMES_AND_TYPES
        + CAMERA_TOPIC_NAMES_AND_TYPES
        + MAVROS_TOPIC_NAMES_AND_TYPES
        + UROS_TOPIC_NAMES_AND_TYPES
    )
    """List of all expected topic names and types"""

    NODE_NAMES_AND_NAMESPACES = {
        (GIS_NODE_NAME, ROS_NAMESPACE),
        (CV_NODE_NAME, ROS_NAMESPACE),
    }
    """List of tuples of node names and namespaces"""

    def __init__(self, *args, **kwargs):
        """Initializer override for declaring attributes used in the test case"""
        super(TestComputationalGraphCase, self).__init__(*args, **kwargs)
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
                namespace, dict(found_names_and_namespaces).get(name, None).lstrip("/")
            )

    def test_topic_names_and_types(self):
        """Tests that nodes subscribe to and publish the expected ROS topics"""
        names, _ = tuple(zip(*self.TOPIC_NAMES_AND_TYPES))

        found_names_and_types = self._get_topic_names_and_types_within_timeout()
        found_names, found_types = tuple(zip(*found_names_and_types))

        assert set(names).issubset(
            found_names
        ), f"Not all topics were discovered ({set(names).difference(found_names)})."
        for name, type_ in self.TOPIC_NAMES_AND_TYPES:
            types = dict(found_names_and_types).get(name)
            assert types is not None
            self.assertEqual(
                type_.__class__.__name__.replace("Metaclass_", ""),
                types[0].split("/")[-1],
            )


class TestGISNodeCase(unittest.TestCase):
    """Tests that :class:`.GISNode` produces expected output from given input

    TODO: link external interfaces, mavros, camera, and WMS server here
    """

    def __init__(self, *args, **kwargs):
        """Initializer override for declaring attributes used in the test case"""
        super(TestGISNodeCase, self).__init__(*args, **kwargs)
        self.state_publisher_node = None
        self.state_listener_node = None

    @classmethod
    def setUpClass(cls):
        """Initialize :term:`ROS` context"""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown :term:`ROS` context"""
        rclpy.shutdown()

    def setUp(self) -> None:
        """Creates the :term:`ROS` helper nodes used for the tests"""
        self.state_publisher_node = Node("state_publisher_node")
        self.state_listener_node = Node("state_listener_node")

    def tearDown(self) -> None:
        """Destroys the :term:`ROS` helper nodes used for the tests"""
        self.state_publisher_node.destroy_node()
        self.state_listener_node.destroy_node()

    def test_vehicle_global_position_valid_range(self):
        """
        Tests that output is correct regardless of input :term:`global position`

        Tests full range of expected input latitude (-90, 90), longitude
        (-180, 180), and altitude for the input :class:`NavSatFix` message.

        :raise: :class:`.AssertionError` if output does not match expected
            output within :param timeout_sec:
        """

        def _assert_output_correct(self, lat, lon, alt):
            """
            Checks that the last output message from the GISNode is correct
            for the given latitude, longitude, and altitude
            """

            # Check that a message was received
            self.assertIsNotNone(
                self.last_output_message, "No output message received from GISNode"
            )

            # Check that the message has the expected values
            # You'll need to replace this with the actual checks for your specific output message
            expected_value = self._calculate_expected_value(lat, lon, alt)
            self.assertEqual(
                self.last_output_message.some_field,
                expected_value,
                "Output value does not match expected value",
            )

        def _calculate_expected_value(self, lat, lon, alt):
            """Calculates the expected output value for the given latitude, longitude, and altitude"""
            # Implement this method to calculate the expected output value based on the input values
            # The details will depend on how the GISNode processes the NavSatFix message

        # Define the valid range for latitude, longitude, and altitude
        latitudes = range(-90, 91, 10)
        longitudes = range(-180, 181, 10)
        amsl_altitudes = range(-1000, 10001, 1000)

        # Iterate through the valid range and test each combination
        for lat in latitudes:
            for lon in longitudes:
                for alt in amsl_altitudes:
                    # GISNode expects input from camera and MAVROS
                    self.state_publisher_node.publish_camera_state()
                    self.state_publisher_node.publish_mavros_state(
                        vehicle_lat=lat, vehicle_lon=lon, vehicle_alt_agl_meters=alt
                    )

                    # Wait for the GISNode to process the message
                    rclpy.spin_once(self.state_publisher_node, timeout_sec=1)

                    # Wait for a message to be received
                    rclpy.spin_once(self.state_listener_node, timeout_sec=1)

                    # Check the output of the GISNode
                    self.state_listener_node.assert_output(
                        vehicle_lat=lat, vehicle_lon=lon, vehicle_alt_amsl_meters=alt
                    )

    """
    def test_vehicle_global_position_invalid_range(self):
        raise NotImplementedError

    def test_vehicle_local_position_valid_range(self):
        raise NotImplementedError

    def test_vehicle_local_position_invalid_range(self):
        raise NotImplementedError

    def test_gimbal_device_attitude_status_valid_range(self):
        raise NotImplementedError

    def test_gimbal_device_attitude_status_invalid_range(self):
        raise NotImplementedError

    def test_home_position_valid_range(self):
        raise NotImplementedError

    def test_home_position_invalid_range(self):
        raise NotImplementedError

    def test_camera_info_valid_range(self):
        raise NotImplementedError

    def test_camera_info_invalid_range(self):
        raise NotImplementedError

    def test_path_invariance(self):
        raise NotImplementedError

    def test_wms_not_available(self):
        raise NotImplementedError

    def test_wms_disconnect(self):
        raise NotImplementedError

    def test_wms_reconnect(self):
        raise NotImplementedError

    def test_get_map_not_available(self):
        raise NotImplementedError

    def test_get_map_timeout(self):
        raise NotImplementedError

    def test_get_feature_info_not_available(self):
        raise NotImplementedError

    def test_get_feature_info_timeout(self):
        raise NotImplementedError

    def test_orthoimagery_not_available(self):
        raise NotImplementedError

    def test_dem_not_available(self):
        raise NotImplementedError
    """


class TestCVNodeCase(unittest.TestCase):
    """Tests that :class:`.CVNode` produces expected output from given input"""


if __name__ == "__main__":
    unittest.main()
