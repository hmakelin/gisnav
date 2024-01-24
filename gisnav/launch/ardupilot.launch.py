"""Launches GISNav with ArduPilot SITL simulation configuration."""
import os
from typing import Final

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription  # type: ignore
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from launch_ros.actions import Node

_PACKAGE_NAME: Final = "gisnav"


def generate_launch_description():
    """Generates launch description with ArduPilot MAVROS adapter"""
    package_share_dir = get_package_share_directory(_PACKAGE_NAME)

    ld = LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/base.launch.py"])
            ),
        ]
    )
    ld.add_action(
        Node(
            package=_PACKAGE_NAME,
            name="mock_gps_node",
            namespace=_PACKAGE_NAME,
            executable="mock_gps_node",
            parameters=[
                os.path.join(
                    package_share_dir, "launch/params/mock_gps_node_ardupilot.yaml"
                )
            ],
        )
    )
    return ld
