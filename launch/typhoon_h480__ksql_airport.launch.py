import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


# TODO: have import package info util function (same logic at least in setup.py and docs/conf.py)
import xml.etree.ElementTree as ET

folder = os.path.dirname(os.path.realpath(__file__))

# Parse info from package.xml
package_file = os.path.join(folder, '..', 'package.xml')
if os.path.isfile(package_file):
    tree = ET.parse(package_file)
    root = tree.getroot()
    package_name = root.find('name').text
    version = root.find('version').text
    description = root.find('description').text
    author = root.find('author').text
    author_email = root.find('author').attrib.get('email', '')
    maintainer = root.find('maintainer').text
    maintainer_email = root.find('maintainer').attrib.get('email', '')
    license_name = root.find('license').text
else:
    raise FileNotFoundError(f'Could not find package file at {package_file}.')


def generate_launch_description():
    """Generates launch description with typhoon_h480__ksql_airport config file."""
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'typhoon_h480__ksql_airport.yaml'
    )

    node = Node(
        package='gisnav',
        name='map_nav_node',
        executable='map_nav_node',
        parameters=[config]
    )
    ld.add_action(node)
    return ld
