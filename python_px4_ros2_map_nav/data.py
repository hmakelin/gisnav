"""Module containing utility functions for map_nav_node."""
from __future__ import annotations  # Python version 3.7+

import cv2
import numpy as np
import os

from xml.etree import ElementTree
from typing import Optional, Union, get_args
from collections import namedtuple
from dataclasses import dataclass, field

from python_px4_ros2_map_nav.assertions import assert_type, assert_ndim, assert_shape

BBox = namedtuple('BBox', 'left bottom right top')  # Convention: https://wiki.openstreetmap.org/wiki/Bounding_Box
LatLon = namedtuple('LatLon', 'lat lon')
LatLonAlt = namedtuple('LatLonAlt', 'lat lon alt')
Dim = namedtuple('Dim', 'height width')
RPY = namedtuple('RPY', 'roll pitch yaw')
TimePair = namedtuple('TimePair', 'local foreign')


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class ImageData:
    """Keeps image frame related data in one place and protects it from corruption."""
    image: np.ndarray
    frame_id: str
    timestamp: int


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
    """Represents camera pose (rotation and translation) along with camera intrinsics"""
    k: np.ndarray
    r: np.ndarray
    t: np.ndarray
    e: np.ndarray = field(init=False)
    h: np.ndarray = field(init=False)
    inv_h: np.ndarray = field(init=False)
    fx: float = field(init=False)
    fy: float = field(init=False)
    cx: float = field(init=False)  # TODO: int?
    cy: float = field(init=False)  # TODO: int?
    camera_position: np.ndarray = field(init=False)
    camera_center: np.ndarray = field(init=False)
    camera_position_difference: np.ndarray = field(init=False)

    def __post_init__(self):
        """Set computed fields after initialization."""
        # Data class is frozen so need to use object.__setattr__ to assign values
        object.__setattr__(self, 'e', np.hstack((self.r, self.t)))  # -self.r.T @ self.t
        object.__setattr__(self, 'h', self.k @ np.delete(self.e, 2, 1))  # Remove z-column, making the matrix square
        try:
            object.__setattr__(self, 'inv_h', np.linalg.inv(self.h))
        except np.linalg.LinAlgError as _:
            #object.__setattr__(self, 'inv_h', None)  # TODO: pass error on?
            object.__setattr__(self, 'inv_h', np.identity(3))  # TODO: pass error on? using identity here is not transparent/intuitive, need to refactor later
        object.__setattr__(self, 'fx', self.k[0][0])
        object.__setattr__(self, 'fy', self.k[1][1])
        object.__setattr__(self, 'cx', self.k[0][2])
        object.__setattr__(self, 'cy', self.k[1][2])
        object.__setattr__(self, 'camera_position', -self.r.T @ self.t)
        object.__setattr__(self, 'camera_center', np.array((self.cx, self.cy, -self.fx)).reshape((3, 1)))  # TODO: assumes fx == fy
        object.__setattr__(self, 'camera_position_difference', self.camera_position - self.camera_center)

    def __matmul__(self, pose: Pose) -> Pose:  # Python version 3.5+
        """Matrix multiplication operator for convenience

        Returns a new pose by combining two camera relative poses:

        pose1 @ pose2 =: Pose(pose1.r @ pose2.r, pose1.t + pose1.r @ pose2.t)
        """
        assert (self.k == pose.k).all(), 'Camera intrinsic matrices are not equal'  # TODO: validation, not assertion
        return Pose(self.k, self.r @ pose.r, self.t + self.r @ (pose.t + pose.camera_center))  # TODO: need to fix sign somehow? Would think minus sign is needed here?


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class InputData:
    """InputData of vehicle state and other variables needed for postprocessing both map and visual odometry matches.

    :param image_data: The drone image
    :param map_data: The map raster
    :param k: Camera intrinsics matrix from CameraInfo from time of match (from _match_inputs)
    :param camera_yaw: Camera yaw in radians from time of match (from _match_inputs)  # TODO: Rename map rotation so less confusion with gimbal attitude stuff extractd from rotation matrix?
    :param vehicle_attitude: Vehicle attitude
    :param map_dim_with_padding: Map dimensions with padding from time of match (from _match_inputs)
    :param img_dim: Drone image dimensions from time of match (from _match_inputs)
    :param map_cropped: - np.ndarray Rotated and cropped map raster from map_data.image
    :param previous_image: - np.ndarray Previous image in case needed for visual odometry visualization
    :return:
    """
    image_data: ImageData
    map_data: Union[ImageData, MapData]  # TODO: if vo, this should just be a another ImageData instead of MapData?
    k: np.ndarray
    camera_yaw: np.ndarray
    vehicle_attitude: np.ndarray
    map_dim_with_padding: Dim
    img_dim: Dim
    map_cropped: np.ndarray
    previous_image: Optional[np.ndarray]

    def __post_init__(self):
        """Validate the data structure"""
        # TODO: Enforce types
        pass


# noinspection PyClassHasNoInit
@dataclass
class OutputData:
    # TODO: add extrinsic matrix / pose, pix_to_wgs84 transformation?
    # TODO: freeze this data structure to reduce unintentional re-assignment?
    """Algorithm output passed onto publish method.

    :param input: The input data used for the match
    :param mkp_img: Matching keypoints in the image
    :param mkp_map: Matching keypoints in the map
    :param pose: Estimated pose for the image frame vs. the map frame
    :param pose_map: Estimated pose for the image frame vs. the map frame (in WGS84)
    :param fov: Camera field of view projected to WGS84 coordinates
    :param fov_pix: Camera field of view in pixels in reference frame (map or previous frame)
    :param position: Vehicle position in WGS84 (elevation or z coordinate in meters above mean sea level)
    :param terrain_altitude: Vehicle altitude in meters from ground (assumed starting altitude)
    :param attitude: Camera attitude quaternion
    :param c: Principal point projected to ground in WGS84 coordinates
    :param sd: Standard deviation of position estimate
    :return:
    """
    input: InputData
    mkp_img: np.ndarray
    mkp_map: np.ndarray
    pose: Pose
    pose_map: Pose
    fov: Optional[np.ndarray]  # TODO: rename fov_wgs84? Can be None if can't be projected to WGS84?
    fov_pix: np.ndarray
    position: LatLonAlt
    terrain_altitude: float
    attitude: np.ndarray
    c: np.ndarray
    sd: np.ndarray

    def __post_init__(self):
        """Validate the data structure"""
        # TODO: Enforce types
        pass


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
