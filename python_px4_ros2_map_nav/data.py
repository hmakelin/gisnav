"""Module containing utility functions for map_nav_node."""
from __future__ import annotations  # Python version 3.7+

import cv2
import numpy as np
import os

from xml.etree import ElementTree
from typing import Union, get_args
from collections import namedtuple
from dataclasses import dataclass

from python_px4_ros2_map_nav.assertions import assert_type, assert_ndim, assert_shape

BBox = namedtuple('BBox', 'left bottom right top')  # Convention: https://wiki.openstreetmap.org/wiki/Bounding_Box
LatLon = namedtuple('LatLon', 'lat lon')
LatLonAlt = namedtuple('LatLonAlt', 'lat lon alt')
Dim = namedtuple('Dim', 'height width')
RPY = namedtuple('RPY', 'roll pitch yaw')
TimePair = namedtuple('TimePair', 'local foreign')


# noinspection PyClassHasNoInit
@dataclass
class ImageData:
    """Keeps image frame related data in one place and protects it from corruption."""
    image: np.ndarray
    frame_id: str
    timestamp: int
    fov: np.ndarray
    fov_pix: np.ndarray
    position: LatLonAlt
    terrain_altitude: float
    attitude: np.ndarray
    c: np.ndarray
    sd: np.ndarray


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class MapData:
    """Keeps map frame related data in one place and protects it from corruption."""
    image: np.ndarray
    center: Union[LatLon, LatLonAlt]
    radius: Union[int, float]
    bbox: BBox


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class Pose:
    """Represents camera pose (rotation and translation)"""
    r: np.ndarray
    t: np.ndarray

    @property
    def e(self) -> np.ndarray:
        """Extrinsic matrix"""
        #return -self.r.T @ self.t
        return np.hstack((self.r, self.t))

    def h(self, k: np.ndarray) -> np.ndarray:
        """Homography matrix for given intrinsics

        :param k: Intrinsic matrix
        """
        return k @ np.delete(self.e, 2, 1)

    def __matmul__(self, pose: Pose) -> Pose:  # Python version 3.5+
        """Matrix multiplication operator for convenience

        Returns a new pose by applying matrix multiplication to the rotation matrix, and addition to the translation
        vector:

        pose1 @ pose2 =: Pose(pose1.r @ pose2.r, pose1.t + pose2.t)
        """
        return Pose(self.r @ pose.r, self.t + pose.t)


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class PackageData:
    """Stores data parsed from package.xml (not comprehensive)"""
    package_name: str
    version: str
    description: str
    author: str
    author_email: str
    maintainer: str
    maintainer_email: str
    license_name: str


def parse_package_data(package_file: str) -> PackageData:
    """Parses package.xml in current folder

    :param package_file: Absolute path to package.xml file
    :return: Parsed package data
    :raise FileNotFoundError: If package.xml file is not found
    """
    if os.path.isfile(package_file):
        tree = ElementTree.parse(package_file)
        root = tree.getroot()
        package_data = PackageData(
            package_name=root.find('name').text,
            version=root.find('version').text,
            description=root.find('description').text,
            author=root.find('author').text,
            author_email=root.find('author').attrib.get('email', ''),
            maintainer=root.find('maintainer').text,
            maintainer_email=root.find('maintainer').attrib.get('email', ''),
            license_name=root.find('license').text
        )
        return package_data
    else:
        raise FileNotFoundError(f'Could not find package file at {package_file}.')
