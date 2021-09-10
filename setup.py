import os
from glob import glob
from setuptools import setup

package_name = 'ng_ransac_wms_map_matching'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'ngransac'), glob('ngransac/ngransac_demo.py')),
        (os.path.join('share', package_name, 'ngransac/models'), glob('ngransac/models/weights_e2e_F_orb_r0.80_.net')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Harri Mäkelin',
    maintainer_email='hmakelin@protonmail.com',
    description='NG-RANSAC based matching of airborne drone video to map retrieved from OGC WMS endpoint.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'matching_node = ng_ransac_wms_map_matching.matching_node:main'
        ],
    },
)
