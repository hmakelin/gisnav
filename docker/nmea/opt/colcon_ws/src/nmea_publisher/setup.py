from setuptools import find_packages, setup

package_name = "nmea_publisher"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hmakelin",
    maintainer_email="hmakelin@protonmail.com",
    description="NMEA ROS to serial bridge",
    license="MIT",
    entry_points={
        "console_scripts": [
            "nmea_publisher_node = nmea_publisher.nmea_publisher_node:main"
        ],
    },
)
