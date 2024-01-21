"""Tests case for topic subscriptions"""
import time
import unittest
from typing import List, Tuple

import rclpy


class TestTopographyCase(unittest.TestCase):
    """Tests that all nodes initialize with the correct :term:`ROS` topic
    subscriptions
    """

    EXPECTED_NODES = (
        "gis_node",
        "transform_node",
        "bbox_node",
        "pose_node",
        "mock_gps_node",
        "test_node",
    )
    """Names of nodes that should be found when running the launch configuration"""

    EXPECTED_NAMESPACES = ("/gisnav", "/gisnav", "/gisnav", "/gisnav", "/gisnav", "/")
    """Expected namespaces of :py:data:`~EXPECTED_NODES` in corresponding order"""

    EXPECTED_TOPICS = (
        "/camera/camera_info",
        "/camera/image_raw",
        "/fmu/in/sensor_gps",
        "/gisnav/bbox_node/fov/bounding_box",
        "/gisnav/gis_node/geotransform",
        "/gisnav/gis_node/orthoimage",
        "/gisnav/transform_node/image",
        "/mavros/gimbal_control/device/attitude_status",
        "/mavros/global_position/global",
        "/mavros/local_position/pose",
        "/mavros/time_reference",
        "/parameter_events",
        "/rosout",
        "/tf",
        "/tf_static",
    )
    """Names of topics that should be found when running the launch configuration"""

    EXPECTED_TYPES = (
        "sensor_msgs/msg/CameraInfo",
        "sensor_msgs/msg/Image",
        "px4_msgs/msg/SensorGps",
        "geographic_msgs/msg/BoundingBox",
        "sensor_msgs/msg/PointCloud2",
        "sensor_msgs/msg/Image",
        "sensor_msgs/msg/Image",
        "mavros_msgs/msg/GimbalDeviceAttitudeStatus",
        "sensor_msgs/msg/NavSatFix",
        "geometry_msgs/msg/PoseStamped",
        "sensor_msgs/msg/TimeReference",
        "rcl_interfaces/msg/ParameterEvent",
        "rcl_interfaces/msg/Log",
        "tf2_msgs/msg/TFMessage",
        "tf2_msgs/msg/TFMessage",
    )
    """Types of :py:data:`~EXPECTED_TOPICS` in corresponding order"""

    def __init__(self, *args, **kwargs):
        """Initializer override for declaring attributes used in the test case"""
        super(TestTopographyCase, self).__init__(*args, **kwargs)
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
        self.test_node = rclpy.node.Node("test_node")

    def tearDown(self) -> None:
        """Destroys the ROS node"""
        self.test_node.destroy_node()

    def test_node_names_and_namespaces(self):
        """Tests that nodes are running with the correct name and namespace.

        This test allows for more nodes than expected to be running without failing.
        """
        expected_nodes_with_ns = set(zip(self.EXPECTED_NODES, self.EXPECTED_NAMESPACES))

        found_names_and_namespaces = self._get_names_and_namespaces_within_timeout(5)

        found_nodes_with_ns = set(found_names_and_namespaces)

        missing_nodes_with_ns = expected_nodes_with_ns - found_nodes_with_ns
        assert not missing_nodes_with_ns, (
            f"Not all expected nodes with namespaces were discovered. "
            f"Missing: {missing_nodes_with_ns}"
        )

    def test_topic_names_and_types(self):
        """Tests that nodes subscribe to and publish the expected ROS topics

        This test allows for more nodes than expected to be running without failing.
        """
        expected_topics_with_types = set(zip(self.EXPECTED_TOPICS, self.EXPECTED_TYPES))

        found_topics_and_types = self._get_topic_names_and_types_within_timeout(5)

        # the list is unhashable (using set() below)
        found_topics_and_types_unlisted = [
            (topic, type_[0]) for topic, type_ in found_topics_and_types
        ]
        found_topics_and_types = set(found_topics_and_types_unlisted)

        missing_topics_and_types = expected_topics_with_types - found_topics_and_types
        assert not missing_topics_and_types, (
            f"Not all expected topics and types were discovered. "
            f"Missing: {missing_topics_and_types}"
        )


class TestTopographyArduPilotCase(TestTopographyCase):

    EXPECTED_TOPICS = (
        "/camera/camera_info",
        "/camera/image_raw",
        "/gisnav/bbox_node/fov/bounding_box",
        "/gisnav/gis_node/geotransform",
        "/gisnav/gis_node/orthoimage",
        "/gisnav/transform_node/image",
        "/mavros/gimbal_control/device/attitude_status",
        "/mavros/global_position/global",
        "/mavros/local_position/pose",
        "/mavros/time_reference",
        "/parameter_events",
        "/rosout",
        "/tf",
        "/tf_static",
    )
    """Names of topics that should be found when running the launch configuration"""

    EXPECTED_TYPES = (
        "sensor_msgs/msg/CameraInfo",
        "sensor_msgs/msg/Image",
        "geographic_msgs/msg/BoundingBox",
        "sensor_msgs/msg/PointCloud2",
        "sensor_msgs/msg/Image",
        "sensor_msgs/msg/Image",
        "mavros_msgs/msg/GimbalDeviceAttitudeStatus",
        "sensor_msgs/msg/NavSatFix",
        "geometry_msgs/msg/PoseStamped",
        "sensor_msgs/msg/TimeReference",
        "rcl_interfaces/msg/ParameterEvent",
        "rcl_interfaces/msg/Log",
        "tf2_msgs/msg/TFMessage",
        "tf2_msgs/msg/TFMessage",
    )
    """Types of :py:data:`~EXPECTED_TOPICS` in corresponding order"""
