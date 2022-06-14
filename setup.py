import os
from glob import glob
from setuptools import setup

from python_px4_ros2_map_nav.nodes.data import PackageData
package_data = PackageData.parse_package_data(os.path.abspath('package.xml'))

# Read requirements file
requirements_file = os.path.abspath('requirements.txt')
install_requires = []
if os.path.isfile(requirements_file):
    with open(requirements_file) as f:
        install_requires = f.read().splitlines()
else:
    raise FileNotFoundError(f'Could not find requirements file at {requirements_file}.')

setup(
    name=package_data.package_name,
    version=package_data.version,
    packages=[
        package_data.package_name,
        package_data.package_name + '.pose_estimators',
        package_data.package_name + '.nodes',
        package_data.package_name + '.filters',
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
        ('share/ament_index/resource_index/packages', ['resource/' + package_data.package_name]),
        ('share/' + package_data.package_name, ['package.xml']),
        # Need to download weights separately, here in LoFTR/weights folder
        (os.path.join('share', package_data.package_name, 'LoFTR', 'weights'), glob('LoFTR/weights/*.ckpt')),
        (os.path.join('share', package_data.package_name, 'config'), glob('config/*.yml')),
        (os.path.join('share', package_data.package_name, 'launch'), glob('launch/*.launch*')),
    ],
    install_requires=install_requires,
    zip_safe=True,
    author=package_data.author,
    author_email=package_data.author_email,
    maintainer=package_data.maintainer,
    maintainer_email=package_data.maintainer_email,
    description=package_data.description,
    license=package_data.license_name,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_nav_node = python_px4_ros2_map_nav.__main__:main'
        ],
    },
)
