"""Launches GISNav with ArduPilot SITL simulation configuration."""
import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from gisnav.data import PackageData
from launch import LaunchDescription  # type: ignore
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    """Generates launch description with ArduPilot MAVROS adapter"""
    dirname = os.path.dirname(__file__)
    filename = os.path.join(dirname, "../package.xml")
    package_data = PackageData.parse_package_data(os.path.abspath(filename))
    package_share_dir = get_package_share_directory(package_data.package_name)

    ld = LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/base.launch.py"])
            ),
        ]
    )
    ld.add_action(
        Node(
            package="gisnav",
            name="ardupilot_node",
            executable="ardupilot_node",
            parameters=[
                os.path.join(package_share_dir, "launch/params/ardupilot_node.yaml")
            ],
        )
    )
    ld.add_action(
        Node(
            package="gisnav",
            name="mock_gps_node",
            executable="mock_gps_node",
            parameters=[
                os.path.join(
                    package_share_dir, "launch/params/mock_gps_node_ardupilot.yaml"
                )
            ],
        )
    )
    return ld
