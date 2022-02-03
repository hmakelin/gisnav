import os
from glob import glob
from setuptools import setup

from python_px4_ros2_map_nav.util import PackageInfo
package_info = PackageInfo(os.path.abspath('package.xml'))

# Read requirements file
requirements_file = os.path.abspath('requirements.txt')
install_requires = []
if os.path.isfile(requirements_file):
    with open(requirements_file) as f:
        install_requires = f.read().splitlines()
else:
    raise FileNotFoundError(f'Could not find requirements file at {requirements_file}.')

setup(
    name=package_info.package_name,
    version=package_info.version,
    packages=[
        package_info.package_name,
        package_info.package_name + '.keypoint_matchers',
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
        ('share/ament_index/resource_index/packages', ['resource/' + package_info.package_name]),
        ('share/' + package_info.package_name, ['package.xml']),
        (os.path.join('share', package_info.package_name, 'LoFTR', 'weights'), glob('LoFTR/weights/*.ckpt')),
        (os.path.join('share', package_info.package_name, 'config'), glob('config/*.yml')),
        (os.path.join('share', package_info.package_name, 'launch'), glob('launch/*.launch*')),
    ],
    install_requires=install_requires,
    zip_safe=True,
    author=package_info.author,
    author_email=package_info.author_email,
    maintainer=package_info.maintainer,
    maintainer_email=package_info.maintainer_email,
    description=package_info.description,
    license=package_info.license_name,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_nav_node = python_px4_ros2_map_nav.__main__:main'
        ],
    },
)
