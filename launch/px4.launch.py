"""Launches GISNav with PX4 SITL simulation configuration."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

from gisnav.data import PackageData


def generate_launch_description():
    """Generates launch description with typhoon_h480__ksql_airport config file"""
    dirname = os.path.dirname(__file__)
    filename = os.path.join(dirname, '../package.xml')
    package_data = PackageData.parse_package_data(os.path.abspath(filename))

    ld = LaunchDescription()

    # Bridge node
    base_node_config = os.path.join(
        get_package_share_directory(package_data.package_name),
        'config',
        'typhoon_h480__ksql_airport.yaml'
    )
    base_node = Node(
        package='gisnav',
        name='base_node',
        executable='base_node',
        parameters=[base_node_config]
    )
    ld.add_action(base_node)

    # Mock GPS node
    mock_gps_node_config = os.path.join(
        get_package_share_directory(package_data.package_name),
        'config',
        'mock_gps_node.yaml'
    )
    mock_gps_node = Node(
        package='gisnav',
        name='mock_gps_node',
        executable='mock_gps_node',
        parameters=[mock_gps_node_config]
    )
    ld.add_action(mock_gps_node)

    # Map node
    map_node_config = os.path.join(
        get_package_share_directory(package_data.package_name),
        'config',
        'map_node.yaml'
    )
    map_node = Node(
        package='gisnav',
        name='map_node',
        executable='map_node',
        parameters=[map_node_config]
    )
    ld.add_action(map_node)

    # BBox node
    bbox_node_config = os.path.join(
        get_package_share_directory(package_data.package_name),
        'config',
        'bbox_node.yaml'
    )
    bbox_node = Node(
        package='gisnav',
        name='bbox_node',
        executable='bbox_node',
        parameters=[bbox_node_config]
    )
    ld.add_action(bbox_node)

    # Pose estimation node
    pose_estimation_node_config = os.path.join(
        get_package_share_directory(package_data.package_name),
        'config',
        'pose_estimation_node.yaml'
    )
    pose_estimation_node = Node(
        package='gisnav',
        name='pose_estimation_node',
        executable='pose_estimation_node',
        parameters=[pose_estimation_node_config]
    )
    ld.add_action(pose_estimation_node)

    return ld
