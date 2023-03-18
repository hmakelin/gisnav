import os
from glob import glob

from setuptools import setup

from gisnav.data import PackageData

pdata = PackageData.parse_package_data(os.path.abspath("package.xml"))

# Setup packages depending on what submodules have been downloaded
packages_ = [
    pdata.package_name,
    pdata.package_name + ".nodes",
    pdata.package_name + ".nodes.base",
    "test",
    "test.launch",
]

setup(
    name=pdata.package_name,
    version=pdata.version,
    packages=packages_,
    package_dir={},
    package_data={},
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + pdata.package_name],
        ),
        ("share/" + pdata.package_name, ["package.xml"]),
        (
            os.path.join("share", pdata.package_name, "launch/params"),
            glob("launch/params/*.yaml"),
        ),
        (os.path.join("share", pdata.package_name, "launch"), glob("launch/*.launch*")),
        (
            os.path.join("share", pdata.package_name, "launch/examples"),
            glob("launch/examples/*.launch*"),
        ),
    ],
    zip_safe=True,
    author=pdata.author,
    author_email=pdata.author_email,
    maintainer=pdata.maintainer,
    maintainer_email=pdata.maintainer_email,
    description=pdata.description,
    license=pdata.license_name,
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "px4_node = gisnav.nodes:run_px4_node",
            "ardupilot_node = gisnav.nodes:run_ardupilot_node",
            "mavros_node = gisnav.nodes:run_mavros_node",
            "mock_gps_node = gisnav.nodes:run_mock_gps_node",
            "map_node = gisnav.nodes:run_map_node",
            "bbox_node = gisnav.nodes:run_bbox_node",
            "pose_estimation_node = gisnav.nodes:run_pose_estimation_node",
        ],
    },
)
