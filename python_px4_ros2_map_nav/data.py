"""Module containing utility functions for map_nav_node."""
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
