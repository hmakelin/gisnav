from setuptools import find_packages, setup

package_name = "ubx_publisher"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "pyserial", "pyubx2"],
    zip_safe=True,
    maintainer="hmakelin",
    maintainer_email="hmakelin@protonmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    entry_points={
        "console_scripts": [
            "ubx_publisher_node = ubx_publisher.ubx_publisher_node:main"
        ],
    },
)
