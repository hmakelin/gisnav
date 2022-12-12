"""Tests ArduPilot launch"""
import unittest
import os

import pytest
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import HomePosition, MountControl
from sensor_msgs.msg import NavSatFix

from test.launch.test_px4_launch import TestPX4Launch


@pytest.mark.launch_test
def generate_test_description():
    """Generates a PX4 launch description"""
    dirname = os.path.dirname(__file__)
    filename = os.path.join(dirname, '../../launch/ardupilot.launch.py')
    ld = IncludeLaunchDescription(PythonLaunchDescriptionSource(filename))
    return LaunchDescription([
        ld,
        ReadyToTest(),
    ])


class TestArduPilotLaunch(TestPX4Launch):
    """Test that all nodes initialize with correct ROS topics"""

    # Override
    AUTOPILOT_TOPIC_NAMES_AND_TYPES = [
        ('/mavros/global_position/global', NavSatFix),
        ('/mavros/pose_stamped/pose', PoseStamped),
        ('/mavros/home_position/home', HomePosition),
        ('/mavros/mount_control/comand', MountControl),
        # ('/mavros/gps_input/gps_input', GPSINPUT)  # not currently used - uses UDP socket instead of MAVROS topic
    ]
    """List of autopilot topic names and types"""

    TOPIC_NAMES_AND_TYPES = TestPX4Launch.GISNAV_TOPIC_NAMES_AND_TYPES + TestPX4Launch.CAMERA_TOPIC_NAMES_AND_TYPES \
                            + AUTOPILOT_TOPIC_NAMES_AND_TYPES
    """List of all expected topic names and types"""

    # Override
    NODE_NAMES_AND_NAMESPACES = {
        ('mock_gps_node', '/'),
        ('bbox_node', '/'),
        ('map_node', '/'),
        ('pose_estimation_node', '/'),
        ('ardupilot_node', '/'),
    }
    """List of tuples of node names and namespaces"""


if __name__ == '__main__':
    unittest.main()
