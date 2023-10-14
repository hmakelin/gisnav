from __future__ import annotations

import os
from dataclasses import dataclass
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
]

setup(
    name=pdata.package_name,
    version=pdata.version,
    packages=packages_,
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + pdata.package_name]),
        ("share/" + pdata.package_name, ["package.xml"]),
    ],
    install_requires=[
        "setuptools",
        "kornia",
        "numpy>=1.24.2",
        "opencv-python",
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
            "pose_node = gisnav_gpu:run_pose_node",
        ],
    },
)
