"""Launches GISNav with PX4 SITL simulation configuration."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from gisnav.data import PackageData


def _create_node_action(package_share_dir: str, name: str, yaml_path: str) -> Node:
    """Returns a node action

    :param package_share_dir: Package share directory
    :param name: Node and executable name (must be same)
    :param yaml_path: Configuration YAML file path in package share directory
    """
    yaml_path = yaml_path.strip('/').split('/')
    config = os.path.join(package_share_dir, *yaml_path)
    node = Node(
        package='gisnav',
        name=name,
        executable=name,
        parameters=[config]
    )
    return node


def generate_launch_description():
    """Generates launch description with PX4 Fast DDS bridge adapter"""
    dirname = os.path.dirname(__file__)
    filename = os.path.join(dirname, '../package.xml')
    package_data = PackageData.parse_package_data(os.path.abspath(filename))
    package_share_dir = get_package_share_directory(package_data.package_name)

    ld = LaunchDescription()

    ld.add_action(_create_node_action(package_share_dir, 'px4_node', 'launch/params/px4_node.yaml'))
    ld.add_action(_create_node_action(package_share_dir, 'mock_gps_node', 'launch/params/mock_gps_node.yaml'))
    ld.add_action(_create_node_action(package_share_dir, 'map_node', 'launch/params/map_node.yaml'))
    ld.add_action(_create_node_action(package_share_dir, 'bbox_node', 'launch/params/bbox_node.yaml'))
    ld.add_action(_create_node_action(package_share_dir, 'pose_estimation_node',
                                      'launch/params/pose_estimation_node.yaml'))

    return ld
