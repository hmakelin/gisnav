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
    """Generates launch description with typhoon_h480__ksql_airport config file"""
    dirname = os.path.dirname(__file__)
    filename = os.path.join(dirname, '../package.xml')
    package_data = PackageData.parse_package_data(os.path.abspath(filename))
    package_share_dir = get_package_share_directory(package_data.package_name)

    ld = LaunchDescription()

    ld.add_action(_create_node_action(package_share_dir, 'ardupilot_node', 'config/ardupilot_node.yaml'))
    ld.add_action(_create_node_action(package_share_dir, 'mock_gps_node', 'config/mock_gps_node.yaml'))
    ld.add_action(_create_node_action(package_share_dir, 'map_node', 'config/map_node.yaml'))
    ld.add_action(_create_node_action(package_share_dir, 'bbox_node', 'config/bbox_node.yaml'))
    ld.add_action(_create_node_action(package_share_dir, 'pose_estimation_node',
                                      'config/pose_estimation_node_ardupilot.yaml'))

    return ld
