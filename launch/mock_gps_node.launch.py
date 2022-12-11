"""Launches :class:`.MockGPSNode` with the typhoon_h480__ksql_airport parameters"""
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
    config = os.path.join(
        get_package_share_directory(package_data.package_name),
        'config',
        'px4_node.yaml'
    )
    node = Node(
        package='gisnav',
        name='mock_gps_node',
        executable='mock_gps_node',
        parameters=[config]
    )
    ld.add_action(node)
    return ld
