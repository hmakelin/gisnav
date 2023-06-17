"""Launches GISNav's autopilot agnostic nodes

The :class:`.PoseEstimationNode`,  :class:`.BBoxNode`, and :class:`.GISNode`
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
            name="gis_node",
            executable="gis_node",
            parameters=[os.path.join(package_share_dir, "launch/params/gis_node.yaml")],
        )
    )
    ld.add_action(
        Node(
            package="gisnav",
            name="cv_node",
            executable="cv_node",
            parameters=[os.path.join(package_share_dir, "launch/params/cv_node.yaml")],
        )
    )
    ld.add_action(
        Node(
            package="gisnav",
            name="rviz_node",
            executable="rviz_node",
            parameters=[
                os.path.join(package_share_dir, "launch/params/rviz_node.yaml")
            ],
        )
    )
    return ld
