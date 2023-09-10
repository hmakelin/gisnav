"""This module contains unit tests for :class:`.GISNode`"""
import test._mocks as mocks
import unittest
from unittest.mock import PropertyMock, patch

import numpy as np
import rclpy
from scipy.spatial.transform import Rotation

from gisnav.core import GISNode


class TestVehicleGeoPoseCase(unittest.TestCase):
    """Unit tests the :attr:`.GISNode.vehicle_geopose` property"""

    def __init__(self, *args, **kwargs):
        """Initializes the test case"""
        super().__init__(*args, **kwargs)

    @classmethod
    def setUpClass(cls):
        """Initializes rclpy context"""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
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

    @staticmethod
    def _generate_random_quaternions(n):
        quaternions = []
        for _ in range(n):
            rand_nums = np.random.randn(4)
            norm = np.linalg.norm(rand_nums)
            quaternion = rand_nums / norm  # Normalize to have a magnitude of 1
            quaternions.append(quaternion)
        return np.array(quaternions)

    @patch.object(GISNode, "vehicle_pose", mocks.vehicle_pose())
    def test_valid_navsatfix_input(self):
        """Tests that :attr:`.GISNode.vehicle_geopose` is correctly computed
        given various valid :attr:`.GISNode.nav_sat_fix` values when a valid
        for :attr:`.GISNode.vehicle_pose` is available.
        """
        with patch(
            "gisnav.core.GISNode.nav_sat_fix", new_callable=PropertyMock
        ) as mock_navsatfix:
            mock_navsatfix.return_value = mocks.navsatfix()
            # Create an instance of GISNode whose vehicle_geopose property is
            # under test.
            gis_node = GISNode("gis_node")
            pose = gis_node.vehicle_geopose.pose
            self.assertEqual(pose.position.latitude, mocks.VEHICLE_LATITUDE_DEGREES)
            self.assertEqual(pose.position.longitude, mocks.VEHICLE_LONGITUDE_DEGREES)
            self.assertEqual(
                pose.position.altitude, mocks.VEHICLE_ELLIPSOID_ALTITUDE_METERS
            )

    @patch.object(GISNode, "nav_sat_fix", mocks.navsatfix())
    def test_valid_pose_input(self):
        """Tests that :attr:`.GISNode.vehicle_geopose` is correctly computed
        given various valid :attr:`.GISNode.vehicle_pose` values when a valid
        for :attr:`.GISNode.nav_sat_fix` is available.
        """
        with patch(
            "gisnav.core.GISNode.vehicle_pose", new_callable=PropertyMock
        ) as mock_vehicle_pose:
            for q in self._generate_random_quaternions(10):
                mock_vehicle_pose.return_value = mocks.vehicle_pose(q)
                # Create an instance of GISNode whose vehicle_geopose property is
                # under test.
                gis_node = GISNode("gis_node")
                pose = gis_node.vehicle_geopose.pose

                expected_ned_quaternion = self._enu_to_ned_quaternion(q)
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

    def test_enu_orientation_conversion(self):
        """Tests that :attr:`.GISNode.vehicle_geopose` :term:`NED` orientation
        is correctly computed when :attr:`.GISNode.vehicle_pose` :term:`ENU`
        orientation changes.
        """
        raise NotImplementedError

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
