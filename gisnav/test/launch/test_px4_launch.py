"""Tests :term:`ROS` launch file for :term:`PX4` configuration"""
import logging
import os
import threading
import time
import unittest
from test.launch.mock_state_publisher import MockStatePublisherNode
from test.launch.state_listener import StateListenerNode
from typing import List, Optional, Tuple

import numpy as np
import pytest
import rclpy
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import PoseStamped, Quaternion
from launch import LaunchDescription  # type: ignore
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest
from mavros_msgs.msg import Altitude, GimbalDeviceAttitudeStatus, HomePosition
from px4_msgs.msg import SensorGps
from pygeodesy import ellipsoidalVincenty as ev
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, NavSatFix

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


# Setup logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class TestComputationalGraphCase(unittest.TestCase):
    """Tests that all nodes initialize with the correct :term:`ROS` computational
    graph structure"""

    GIS_NODE_TOPIC_NAMES_AND_TYPES = [
        (
            f"/{ROS_NAMESPACE}"
            f'/{ROS_TOPIC_RELATIVE_ORTHOIMAGE.replace("~", GIS_NODE_NAME)}',
            OrthoImage3D,
        ),
        (
            f"/{ROS_NAMESPACE}"
            f'/{ROS_TOPIC_RELATIVE_VEHICLE_GEOPOSE.replace("~", GIS_NODE_NAME)}',
            GeoPoseStamped,
        ),
        (
            f"/{ROS_NAMESPACE}"
            f'/{ROS_TOPIC_RELATIVE_VEHICLE_ALTITUDE.replace("~", GIS_NODE_NAME)}',
            Altitude,
        ),
        (
            f"/{ROS_NAMESPACE}"
            f'/{ROS_TOPIC_RELATIVE_CAMERA_QUATERNION.replace("~", GIS_NODE_NAME)}',
            Quaternion,
        ),
        (
            f"/{ROS_NAMESPACE}"
            f'/{ROS_TOPIC_RELATIVE_GROUND_TRACK_ELEVATION.replace("~", GIS_NODE_NAME)}',
            Altitude,
        ),
        (
            f"/{ROS_NAMESPACE}"
            f'/{ROS_TOPIC_RELATIVE_GROUND_TRACK_GEOPOSE.replace("~", GIS_NODE_NAME)}',
            GeoPoseStamped,
        ),
    ]
    """List of :class:`.GISNode` published topic names and types"""

    CV_NODE_TOPIC_NAMES_AND_TYPES = [
        (
            f"/{ROS_NAMESPACE}"
            f'/{ROS_TOPIC_RELATIVE_VEHICLE_ESTIMATED_GEOPOSE.replace("~", CV_NODE_NAME)}',  # noqa: E501
            GeoPoseStamped,
        ),
        (
            f"/{ROS_NAMESPACE}"
            f'/{ROS_TOPIC_RELATIVE_VEHICLE_ESTIMATED_ALTITUDE.replace("~", CV_NODE_NAME)}',  # noqa: E501
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

    DOCKER_COMPOSE_SERVICES = "mapserver torch-serve"
    """:term:`Docker Compose` services required to support this test case"""

    DOCKER_COMPOSE_FILE_PATH = os.path.join(
        os.path.dirname(__file__), "../../../docker/docker-compose.yaml"
    )
    """Path to :term:`Docker Compose` configuration

    Needed to launch :attr:`.DOCKER_COMPOSE_SERVICES`
    """

    def __init__(self, *args, **kwargs):
        """Initializer override for declaring attributes used in the test case"""
        super(TestGISNodeCase, self).__init__(*args, **kwargs)
        self.state_publisher_node: Optional[MockStatePublisherNode] = None
        self.state_listener_node: Optional[StateListenerNode] = None
        self.executor: Optional[MultiThreadedExecutor] = None

        # Tear down test case on Ctrl-C (do not leave Docker containers running)
        import signal

        def custom_sigint_handler(signal, frame):
            self.tearDown()
            self.tearDownClass()
            self.fail(
                "Test failure caused by keyboard interrupt. Tear down "
                "methods called."
            )

        signal.signal(signal.SIGINT, custom_sigint_handler)

    @classmethod
    def setUpClass(cls):
        """Ensure that supporting :term:`Docker Compose` service containers
        are available and initialize :term:`ROS` context
        """
        if not os.path.exists(cls.DOCKER_COMPOSE_FILE_PATH):
            raise FileNotFoundError(
                f"Could not find Docker Compose configuration at "
                f"{cls.DOCKER_COMPOSE_FILE_PATH}. {cls.__name__} requires the "
                f"configuration to start the "
                f"{' and '.join(cls.DOCKER_COMPOSE_SERVICES.split(' '))} "
                f"services to support launch testing."
            )

        logger.info(
            f"Creating docker containers ({cls.DOCKER_COMPOSE_SERVICES}) to support "
            f"launch testing. This may take several minutes..."
        )
        os.system(
            f"docker compose -p gisnav -f {cls.DOCKER_COMPOSE_FILE_PATH} "
            f"create {cls.DOCKER_COMPOSE_SERVICES}"
        )

        logger.info(
            f"Starting launch testing docker services "
            f"({cls.DOCKER_COMPOSE_SERVICES})..."
        )
        os.system(
            f"docker compose -p gisnav -f {cls.DOCKER_COMPOSE_FILE_PATH} "
            f"up -d {cls.DOCKER_COMPOSE_SERVICES}"
        )

    @classmethod
    def tearDownClass(cls):
        """Shutdown :term:`ROS` context"""
        logger.info(
            f"Tearing down test case. Will not delete containers for "
            f"Docker Compose services ({cls.DOCKER_COMPOSE_SERVICES}). You "
            f"can delete all unused containers by typing "
            f"'docker system prune' into your system shell."
        )

        logger.info(
            f"Shutting down launch testing docker services "
            f"({cls.DOCKER_COMPOSE_SERVICES})..."
        )
        os.system(f"docker compose -p gisnav -f {cls.DOCKER_COMPOSE_FILE_PATH} down")

    @staticmethod
    def _add_meters_to_coordinates(
        lat_lon: Tuple[float, float], meters_north: float, meters_east: float
    ):
        """Adds meters to :term:`WGS 84` latitude and longitude degrees"""
        point = ev.LatLon(*lat_lon)
        new_point_north = point.destination(meters_north, 0)  # 0 degrees for North
        new_point = new_point_north.destination(meters_east, 90)  # 90 degrees for East
        return new_point.lat, new_point.lon

    def setUp(self) -> None:
        """Creates the :term:`ROS` helper nodes used for the tests"""
        logger.info("Starting launch testing state publisher and listener nodes...")
        rclpy.init()

        self.state_publisher_node = MockStatePublisherNode("state_publisher_node")
        self.state_listener_node = StateListenerNode("state_listener_node")
        # Create a MultiThreadedExecutor
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.state_publisher_node)
        self.executor.add_node(self.state_listener_node)

        # Spin the publisher and listener nodes in a separate thread
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

    def tearDown(self) -> None:
        """Destroys the :term:`ROS` helper nodes used for the tests"""
        logger.info("Destroying launch testing state publisher and listener nodes...")

        assert self.state_publisher_node is not None
        assert self.state_listener_node is not None
        self.state_publisher_node.destroy_node()
        self.state_listener_node.destroy_node()

        # Call shutdown first before joining to ensure executor thread stops
        # spinning nodes
        rclpy.shutdown()
        self.executor_thread.join()

    def test_valid_vehicle_global_position(self) -> None:
        """Tests that :class:`.GISNode` :term:`vehicle` :term:`global position`
        output is correct when valid :ref:`MAVROS and camera messages
        <Core data flow graph>` are published from :class:`.MockStatePublisherNode`.

        This test varies the sent input by a few meters around the defaults
        defined in :meth:`.publish_mavros_state` and checks that
        :attr:`.GISNode.vehicle_geopose` and :attr:`.GISNode.vehicle_altitude`
        match the input.

        This test assumes the :ref:`mapserver Docker Compose service
        <List of services>` is running and has :term:`orthoimagery` coverage
        for the region defined by the global position input argument default values
        published by :meth:`.publish_mavros_state` (:term:`KSQL airport <KSQl>`).

        :raise: :class:`.AssertionError` if output does not match what is
            expected based on input
        """
        delta_meters = tuple(np.arange(-10.0, 10.0, 10.0))

        # Iterate through the valid range and test each combination
        for delta_lat_meters in delta_meters:
            for delta_lon_meters in delta_meters:
                for delta_alt_meters in delta_meters:
                    lat, lon = self._add_meters_to_coordinates(
                        (
                            MockStatePublisherNode.D_VEHICLE_LAT,
                            MockStatePublisherNode.D_VEHICLE_LON,
                        ),
                        delta_lat_meters,
                        delta_lon_meters,
                    )
                    alt = (
                        MockStatePublisherNode.D_VEHICLE_ALT_AMSL_METERS
                        + delta_alt_meters
                    )

                    # Since the nodes are being spun in a separate thread,
                    # we don't need to manually spin them here.
                    # Just introduce a delay or a condition to wait for the
                    # expected output.
                    for _ in range(2):
                        assert self.state_publisher_node is not None
                        time.sleep(3)
                        # GISNode expects input from camera and MAVROS
                        self.state_publisher_node.publish_camera_state()
                        self.state_publisher_node.publish_mavros_state(
                            vehicle_lat=lat,
                            vehicle_lon=lon,
                            vehicle_alt_amsl_meters=alt,
                        )

                    # Check the output of the GISNode
                    assert self.state_listener_node is not None
                    self.state_listener_node.assert_state(
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

    def test_message_timestamp_too_old(self):
        raise NotImplementedError
    """


class TestCVNodeCase(unittest.TestCase):
    """Tests that :class:`.CVNode` produces expected output from given input"""


if __name__ == "__main__":
    unittest.main()
