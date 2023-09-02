"""This module contains unit tests for :class:`.GISNode`"""
import test._mocks as mocks
import unittest
from unittest.mock import patch

import numpy as np
import rclpy
from scipy.spatial.transform import Rotation

from gisnav.core import GISNode


class TestVehicleGeoPoseCase(unittest.TestCase):
    """Unit tests the :attr:`.GISNode.vehicle_geopose` property"""

    def __init__(self, *args, **kwargs):
        """Initializes the test case"""
        super().__init__(*args, **kwargs)

    def setUp(self):
        """Initializes rclpy context"""
        rclpy.init()

    def tearDown(self):
        """Shuts down rclpy context"""
        rclpy.shutdown()

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

    @patch.object(GISNode, "nav_sat_fix", mocks.navsatfix())
    @patch.object(GISNode, "vehicle_pose", mocks.vehicle_pose())
    def test_valid_inputs(self):
        """Tests that :attr:`.GISNode.vehicle_geopose` is correctly computed
        when valid inputs ( :attr:`.GISNode.nav_sat_fix` and
        :attr:`.GISNode.vehicle_pose`) are available.

        Tests both :term:`global position` and :term:`orientation` outputs
        of :attr:`.GISNode.vehicle_geopose`.
        """
        # Create an instance of GISNode whose vehicle_geopose property is
        # under test.
        gis_node = GISNode("gis_node")
        pose = gis_node.vehicle_geopose.pose

        # Assert vehicle global position
        self.assertEqual(pose.position.latitude, mocks.VEHICLE_LATITUDE_DEGREES)
        self.assertEqual(pose.position.longitude, mocks.VEHICLE_LONGITUDE_DEGREES)
        self.assertEqual(
            pose.position.altitude, mocks.VEHICLE_ELLIPSOID_ALTITUDE_METERS
        )

        # Assert vehicle NED frame orientation
        expected_ned_quaternion = self._enu_to_ned_quaternion(
            mocks.VEHICLE_ENU_QUATERNION
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
