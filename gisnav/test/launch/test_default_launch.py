"""Tests :term:`ROS` launch file for :term:`PX4` configuration"""
import os

import pytest
from launch import LaunchDescription  # type: ignore
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_testing.actions import ReadyToTest

from test.launch.testcases import TestTopographyCase  # isort: skip  # noqa: F401


@pytest.mark.launch_test
def generate_test_description():
    """Generates a PX4 launch description"""
    dirname = os.path.dirname(__file__)
    filename = os.path.join(dirname, "../../launch/default.launch.py")
    ld = IncludeLaunchDescription(PythonLaunchDescriptionSource(filename))
    return LaunchDescription(
        [
            ld,
            ReadyToTest(),
        ]
    )