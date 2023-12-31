"""Immutable dataclasses to protect atomicity of related information

The classes here may return a :class:`.DataValueError` upon instantiation to
prevent invalid data structures from being instantiated. Example usage that
handles the exception:

.. code-block:: python

    try:
        position = Position(
            ...
        )
        return position
    except DataValueError as dve:
        self.get_logger().error('Error determining vehicle position.')
        return None
"""
from __future__ import annotations  # Python version 3.7+

import os
import warnings
from collections import namedtuple
from dataclasses import dataclass, field
from typing import Optional, Union
from xml.etree import ElementTree

import numpy as np
from scipy.spatial.transform import Rotation

warnings.filterwarnings(
    action="ignore", category=UserWarning, message="Gimbal lock detected."
)

BBox = namedtuple("BBox", "left bottom right top")


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class Attitude:
    """Attitude (orientation) in 3D space, typically in FRD or NED frame
    depending on context"""

    # (x, y, z, w) SciPy format! :class:`px4_msgs.VehicleAttitude` q is (w, x, y, z)
    q: np.ndarray
    roll: float = field(init=False)
    pitch: float = field(init=False)
    yaw: float = field(init=False)
    r: float = field(init=False)
    extrinsic: bool = False

    def __post_init__(self):
        """Post-initialization validity checks"""
        assert len(self.q) == 4
        rotation = Rotation.from_quat(self.q)
        roll, pitch, yaw = tuple(rotation.as_euler("xyz" if self.extrinsic else "XYZ"))
        object.__setattr__(self, "roll", roll)
        object.__setattr__(self, "pitch", pitch)
        object.__setattr__(self, "yaw", yaw)
        object.__setattr__(self, "r", rotation.as_matrix())

    def to_esd(self) -> Attitude:
        """Converts attitude from NED to solvePnP ESD world frame

        :return: Attitude in SED frame
        """
        nadir_pitch = np.array(
            [0, np.sin(np.pi / 4), 0, np.sin(np.pi / 4)]
        )  # Adjust origin to nadir facing camera
        r = Rotation.from_quat(self.q) * Rotation.from_quat(nadir_pitch)
        q = r.as_quat()
        q = np.array([q[1], -q[0], q[2], -q[3]])  # NED to ESD
        att = Attitude(q, self.extrinsic)
        return att

    def as_rotation(self) -> Rotation:
        """Attitude aa :class:`scipy.spatial.transform.Rotation` instance"""
        return Rotation.from_quat(self.q)


# noinspection PyClassHasNoInit
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

    @staticmethod
    def parse_package_data(package_file: str) -> PackageData:
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
                "package_name": PackageData.require_not_none(root.findtext("name")),
                "version": PackageData.require_not_none(root.findtext("version")),
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
