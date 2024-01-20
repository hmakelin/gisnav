"""Launches GISNav :term:`core` nodes"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription  # type: ignore
from launch_ros.actions import Node

from gisnav._data import PackageData
from gisnav.core.constants import ROS_IMAGE_TOPIC, ROS_CAMERA_INFO_TOPIC


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
            name="transform_node",
            executable="transform_node",
            parameters=[
                os.path.join(package_share_dir, "launch/params/transform_node.yaml")
            ],
        )
    )
    ld.add_action(
        Node(
            package="gisnav",
            name="bbox_node",
            executable="bbox_node",
            parameters=[
                os.path.join(package_share_dir, "launch/params/bbox_node.yaml")
            ],
        )
    )
    ld.add_action(
        Node(
            package="gisnav",
            name="pose_node",
            executable="pose_node",
            parameters=[
                os.path.join(package_share_dir, "launch/params/pose_node.yaml")
            ],
            remappings=[
                (ROS_IMAGE_TOPIC, "image"),
                (ROS_CAMERA_INFO_TOPIC, "camera_info"),
            ],
        )
    )
    return ld
