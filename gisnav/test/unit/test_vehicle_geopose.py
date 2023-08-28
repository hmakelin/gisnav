"""This module contains unit tests for the :attr:`.GISNode.vehicle_geopose` property"""
import unittest
from unittest.mock import PropertyMock, patch

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import NavSatFix

from gisnav.core import GISNode


class TestVehicleGeoPoseCase(unittest.TestCase):
    """Unit tests the :attr:`.GISNode.vehicle_geopose` property"""

    # Mocked ENU values
    MOCK_ENU_LATITUDE = 37.523640
    MOCK_ENU_LONGITUDE = -122.255122
    MOCK_ENU_ALTITUDE = 100.0
    MOCK_ENU_ROLL_DEG = 45
    MOCK_ENU_PITCH_DEG = 30
    MOCK_ENU_YAW_DEG = 60

    # Expected NED values
    EXPECTED_NED_ROLL_DEG = -MOCK_ENU_PITCH_DEG
    EXPECTED_NED_PITCH_DEG = MOCK_ENU_ROLL_DEG
    EXPECTED_NED_YAW_DEG = (
        MOCK_ENU_YAW_DEG + 90
    ) % 360  # Adding 90 and ensuring it's within [0, 360)
    EXPECTED_NED_QUAT = Rotation.from_euler(
        "XYZ",
        [EXPECTED_NED_ROLL_DEG, EXPECTED_NED_PITCH_DEG, EXPECTED_NED_YAW_DEG],
        degrees=True,
    ).as_quat()

    def __init__(self, *args, **kwargs):
        """Initializes the test case"""
        super().__init__(*args, **kwargs)

    def setUp(self):
        """Initializes rclpy context"""
        rclpy.init()

    def tearDown(self):
        """Shuts down rclpy context"""
        rclpy.shutdown()

    @property
    def _valid_nav_sat_fix(self) -> NavSatFix:
        """Valid mock :class:`sensor_msgs.msg.NavSatFix` message"""
        mock_nav_sat_fix = NavSatFix()
        mock_nav_sat_fix.latitude = self.MOCK_ENU_LATITUDE
        mock_nav_sat_fix.longitude = self.MOCK_ENU_LONGITUDE
        mock_nav_sat_fix.altitude = self.MOCK_ENU_ALTITUDE
        return mock_nav_sat_fix

    @property
    def _valid_vehicle_pose(self) -> PoseStamped:
        """Valid mock :class:`geometry_msgs.msg.PoseStamped` message"""
        mock_vehicle_pose = PoseStamped()

        # Convert Euler angles (roll, pitch, yaw) to quaternion
        roll, pitch, yaw = (
            np.radians(self.MOCK_ENU_ROLL_DEG),
            np.radians(self.MOCK_ENU_PITCH_DEG),
            np.radians(self.MOCK_ENU_YAW_DEG),
        )
        quaternion = Rotation.from_euler("XYZ", [roll, pitch, yaw]).as_quat()

        mock_vehicle_pose.pose.orientation.x = quaternion[0]
        mock_vehicle_pose.pose.orientation.y = quaternion[1]
        mock_vehicle_pose.pose.orientation.z = quaternion[2]
        mock_vehicle_pose.pose.orientation.w = quaternion[3]

        return mock_vehicle_pose

    @property
    def _valid_connected_true(self) -> PoseStamped:
        """Valid :attr:`.GISNode.connected` is True mock

        Using this mock will enable the test to skip trying to have
        :class:`.GISNode` connect to the :term:`WMS` service and assume that
        it's available.
        """
        return True

    def test_valid_inputs(self):
        """Tests that :attr:`.GISNode.vehicle_geopose` is correctly computed
        when valid inputs ( :attr:`.GISNode.nav_sat_fix` and
        :attr:`.GISNode.vehicle_pose`) are available.

        Tests both :term:`global position` and :term:`orientation` outputs
        of :attr:`.GISNode.vehicle_geopose`.
        """
        # Mock the my_property of the MyNode class
        with patch.object(
            GISNode, "nav_sat_fix", new_callable=PropertyMock
        ) as mock_nav_sat_fix, patch.object(
            GISNode, "vehicle_pose", new_callable=PropertyMock
        ) as mock_vehicle_pose, patch.object(
            GISNode, "connected", new_callable=PropertyMock
        ) as mock_connected:
            mock_nav_sat_fix.return_value = self._valid_nav_sat_fix
            mock_vehicle_pose.return_value = self._valid_vehicle_pose
            mock_connected.return_value = self._valid_connected_true

            # Create an instance of the node whose properties are under test.
            # Only after patching to make sure any patches also apply to
            # initialization
            gis_node = GISNode("gis_node")
            pose = gis_node.vehicle_geopose.pose
            # Assert global position
            self.assertEqual(pose.position.latitude, self.MOCK_ENU_LATITUDE)
            self.assertEqual(pose.position.longitude, self.MOCK_ENU_LONGITUDE)
            self.assertEqual(pose.position.altitude, self.MOCK_ENU_ALTITUDE)

            # Assert NED orientation
            expected_ned_quaternion = Rotation.from_euler(
                "XYZ",
                [
                    self.EXPECTED_NED_ROLL_DEG,
                    self.EXPECTED_NED_PITCH_DEG,
                    self.EXPECTED_NED_YAW_DEG,
                ],
                degrees=True,
            ).as_quat()
            computed_orientation = np.array(
                [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ]
            )
            np.testing.assert_array_almost_equal(
                computed_orientation, expected_ned_quaternion, decimal=5
            )

    # TODO: test attitude edge cases (e.g. gimbal lock, flying upside down, etc.)


if __name__ == "__main__":
    unittest.main()
