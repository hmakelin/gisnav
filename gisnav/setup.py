from __future__ import annotations

import os
from dataclasses import dataclass
from glob import glob
from typing import Optional, Union
from xml.etree import ElementTree

from setuptools import setup


@dataclass(frozen=True)
class PackageData:
    """Stores data parsed from package.xml (not comprehensive)"""

    package_name: str
    version: str
    description: Optional[str]
    author: Optional[str]
    author_email: Optional[str]
    maintainer: Optional[str]
    maintainer_email: Optional[str]
    license_name: Optional[str]

    @staticmethod
    def require_not_none(text: Optional[Union[str, ElementTree.Element]]):
        """Raises ValueError if input is not a string but None"""
        if text is None:
            raise ValueError("Expected not None")
        return text

    @classmethod
    def parse_package_data(cls, package_file: str) -> PackageData:
        """Parses package.xml in current folder

        :param package_file: Absolute path to package.xml file
        :return: Parsed package data
        :raise FileNotFoundError: If package.xml file is not found
        """
        if os.path.isfile(package_file):
            tree = ElementTree.parse(package_file)
            root = tree.getroot()

            author = root.find("author")
            maintainer = root.find("maintainer")
            kwargs = {
                "package_name": cls.require_not_none(root.findtext("name")),
                "version": cls.require_not_none(root.findtext("version")),
                "description": root.findtext("description"),
                "author": root.findtext("author"),
                "author_email": author.attrib.get("email")
                if author is not None
                else None,
                "maintainer": root.findtext("maintainer"),
                "maintainer_email": maintainer.attrib.get("email")
                if maintainer is not None
                else None,
                "license_name": root.findtext("license"),
            }
            package_data = PackageData(**kwargs)
            return package_data
        else:
            raise FileNotFoundError(f"Could not find package file at {package_file}.")


pdata = PackageData.parse_package_data(os.path.abspath("package.xml"))

# Setup packages depending on what submodules have been downloaded
packages_ = [
    pdata.package_name,
    pdata.package_name + ".core",
    pdata.package_name + ".extensions",
    "test",
    "test.unit",
    "test.launch",
    "test.sitl",
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
    python_requires=">=3.7",
    install_requires=[
        # "numpy>=1.24.2",
        # "opencv-python",
        # "pyproj>=3.2.1",
        # "requests",
        # "setuptools",
        # "shapely>=1.8.2",
        "OWSLib>=0.25.0",
        "torch>=2.1.0",
        "kornia==0.6.10",
        "transforms3d",
    ],
    tests_require=["pytest"],
    extras_require={
        "mock_gps_node": ["gps-time"],
        "qgis_node": ["psycopg2"],
        "dev": [
            "aiohttp",
            "autodocsumm",
            "coverage",
            "docutils>=0.17",
            "jupyter",
            "lxml",
            "mavsdk",
            "myst_parser",
            "pre-commit",
            "pydata-sphinx-theme",
            "pygments",
            "pytest",
            "python-dateutil>=2.8.2",
            "pyyaml",
            "sphinx-copybutton",
            "sphinx-design",
            "Sphinx-Substitution-Extensions",
            "sphinxcontrib-mermaid",
            "sphinxcontrib-video",
            "Sphinx",
            "types-PyYAML",
        ],
    },
    entry_points={
        "console_scripts": [
            "mock_gps_node = gisnav:run_mock_gps_node",
            "gis_node = gisnav:run_gis_node",
            "stereo_node = gisnav:run_stereo_node",
            "pose_node = gisnav:run_pose_node",
            "bbox_node = gisnav:run_bbox_node",
            "rviz_node = gisnav:run_rviz_node",
            "qgis_node = gisnav:run_qgis_node",
        ],
    },
)
