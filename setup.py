import os
from glob import glob
from setuptools import setup

import xml.etree.ElementTree as ET

folder = os.path.dirname(os.path.realpath(__file__))

# Parse info from package.xml
package_file = os.path.join(folder, 'package.xml')
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

# Read requirements file
requirements_file = os.path.join(folder, 'requirements.txt')
install_requires = []
if os.path.isfile(requirements_file):
    with open(requirements_file) as f:
        install_requires = f.read().splitlines()
else:
    raise FileNotFoundError(f'Could not find requirements file at {requirements_file}.')

setup(
    name=package_name,
    version=version,
    packages=[
        package_name,
        package_name + '.keypoint_matchers',
        'LoFTR.loftr',
        'LoFTR.loftr.backbone',
        'LoFTR.loftr.loftr_module',
        'LoFTR.loftr.utils',
        'SuperGluePretrainedNetwork.models',
    ],
    package_dir={
        'LoFTR': 'LoFTR/src',
    },
    package_data={
        'SuperGluePretrainedNetwork.models': ['weights/*.pth'],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'LoFTR', 'weights'), glob('LoFTR/weights/*.ckpt')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch*')),
    ],
    install_requires=install_requires,
    zip_safe=True,
    author=author,
    author_email=author_email,
    maintainer=maintainer,
    maintainer_email=maintainer_email,
    description=description,
    license=license_name,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_nav_node = python_px4_ros2_map_nav.__main__:main'
        ],
    },
)
