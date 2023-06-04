"""Launches GISNav's autopilot agnostic nodes

The :class:`.PoseEstimationNode`,  :class:`.BBoxNode`, and :class:`.MapNode`
nodes are autopilot agnostic and are launched from a shared description
defined in this file.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription  # type: ignore
from launch_ros.actions import Node

from gisnav.data import PackageData


def generate_launch_description():
    """Generates shared autopilot agnostic launch description"""
    dirname = os.path.dirname(__file__)
    filename = os.path.join(dirname, "../package.xml")
    package_data = PackageData.parse_package_data(os.path.abspath(filename))
    package_share_dir = get_package_share_directory(package_data.package_name)

    ld = LaunchDescription()
    ld.add_action(
        Node(
            package="gisnav",
            name="map_node",
            executable="map_node",
            parameters=[os.path.join(package_share_dir, "launch/params/map_node.yaml")],
        )
    )
    ld.add_action(
        Node(
            package="gisnav",
            name="pose_estimation_node",
            executable="pose_estimation_node",
            parameters=[
                os.path.join(
                    package_share_dir, "launch/params/pose_estimation_node.yaml"
                )
            ],
        )
    )
    return ld
