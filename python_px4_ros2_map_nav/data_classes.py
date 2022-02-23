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


def _make_keypoint(pt: np.ndarray, sz: float = 1.0) -> cv2.KeyPoint:
    """Converts input numpy array to a cv2.KeyPoint.

    :param pt: Keypoint x and y coordinates
    :param sz: Keypoint size
    :return:
    """
    assert_type(pt, np.ndarray)
    assert_shape(pt, (2,))
    return cv2.KeyPoint(pt[0], pt[1], sz)


def visualize_homography(figure_name: str, display_text: str, img_arr: np.ndarray, map_arr: np.ndarray,
                         kp_img: np.ndarray, kp_map: np.ndarray, dst_corners: np.ndarray) -> np.ndarray:
    """Visualizes a homography including keypoint matches and field of view.

    :param figure_name: Display name of visualization
    :param display_text: Display text on top left of image
    :param img_arr: Image array
    :param map_arr: Map array
    :param kp_img: Image keypoints
    :param kp_map: Map keypoints
    :param dst_corners: Field of view corner pixel coordinates on map
    :return: Visualized image as numpy array
    """
    # Make a list of cv2.DMatches that match mkp_img and mkp_map one-to-one
    kp_count = len(kp_img)
    assert kp_count == len(kp_map), 'Keypoint counts for img and map did not match.'
    matches = list(map(lambda i_: cv2.DMatch(i_, i_, 0), range(0, kp_count)))

    # Need cv2.KeyPoints for keypoints
    kp_img = np.apply_along_axis(_make_keypoint, 1, kp_img)
    kp_map = np.apply_along_axis(_make_keypoint, 1, kp_map)

    map_with_fov = cv2.polylines(map_arr, [np.int32(dst_corners)], True, 255, 3, cv2.LINE_AA)
    draw_params = dict(matchColor=(0, 255, 0), singlePointColor=None, matchesMask=None, flags=2)
    out = cv2.drawMatches(img_arr, kp_img, map_with_fov, kp_map, matches, None, **draw_params)

    # Add text (need to manually handle newlines)
    for i, text_line in enumerate(display_text.split('\n')):
        y = (i + 1) * 30
        cv2.putText(out, text_line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, 2)

    cv2.imshow(figure_name, out)
    cv2.waitKey(1)

    return out
