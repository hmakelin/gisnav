"""This module contains unit tests for :class:`.GISNode`"""
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
    MOCK_ENU_QUATERNION = np.array([0.0, 0.0, 0.0, 1.0])  # x, y, z, w

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
        quaternion = self.MOCK_ENU_QUATERNION
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

    @staticmethod
    def _enu_to_ned_quaternion(enu_quat: np.ndarray):
        """Converts an :term:`ENU` quaternion to a :term:`NED` quaternion

        Expects quaternion in (x, y, z, w) order.
        """
        # TODO: check if this is correct, alternative implementation below
        # Define the transformation from ENU to NED
        # enu_to_ned_transform = Rotation.from_euler("ZY", [np.pi, np.pi / 2])
        # Convert the orientation from ENU to NED
        # attitude_ned = Rotation.from_quat(enu_quat) * enu_to_ned_transform
        # Return the quaternion representation of the orientation in NED
        # return attitude_ned.as_quat()

        enu_to_ned_transform = Rotation.from_euler(
            "XYZ", np.array([np.pi, 0, np.pi / 2])
        )
        attitude_ned = Rotation.from_quat(enu_quat) * enu_to_ned_transform.inv()

        rpy = attitude_ned.as_euler("XYZ", degrees=True)
        rpy[0] = (rpy[0] + 180) % 360
        attitude_ned = Rotation.from_euler("XYZ", rpy, degrees=True)
        return attitude_ned.as_quat()

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
            expected_ned_quaternion = self._enu_to_ned_quaternion(
                self.MOCK_ENU_QUATERNION
            )
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

    def test_invalid_inputs(self):
        raise NotImplementedError

    def test_missing_inputs(self):
        raise NotImplementedError

    def test_orientation_edge_cases(self):
        raise NotImplementedError

    def test_global_position_edge_cases(self):
        raise NotImplementedError


if __name__ == "__main__":
    unittest.main()
