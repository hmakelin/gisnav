"""Module containing data structures to protect atomicity of related information"""
from __future__ import annotations  # Python version 3.7+

import cv2
import numpy as np
import os

from xml.etree import ElementTree
from typing import Optional, Union, get_args
from collections import namedtuple
from dataclasses import dataclass, field
from multiprocessing.pool import AsyncResult

from python_px4_ros2_map_nav.assertions import assert_type, assert_ndim, assert_shape

BBox = namedtuple('BBox', 'left bottom right top')  # Convention: https://wiki.openstreetmap.org/wiki/Bounding_Box
LatLon = namedtuple('LatLon', 'lat lon')
LatLonAlt = namedtuple('LatLonAlt', 'lat lon alt')
Dim = namedtuple('Dim', 'height width')
RPY = namedtuple('RPY', 'roll pitch yaw')
TimePair = namedtuple('TimePair', 'local foreign')


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class _Image:
    """Parent dataclass for image holders

    Should not be instantiated directly.
    """
    image: np.ndarray


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class ImageData(_Image):
    """Keeps image frame related data in one place and protects it from corruption."""
    #image: np.ndarray
    frame_id: str
    timestamp: int


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class MapData(_Image):
    """Keeps map frame related data in one place and protects it from corruption."""
    #image: np.ndarray
    center: Union[LatLon, LatLonAlt]
    radius: Union[int, float]
    bbox: BBox


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class ContextualMapData(_Image):
    """Contains the rotated and cropped map image for pose estimation"""
    #image: np.ndarray
    rotation: Union[float, int]
    img_dim: Dim
    map_data: MapData


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class ImagePair:
    """Atomic image pair to represent a matched pair of images"""
    img: ImageData
    ref: Union[ImageData, ContextualMapData]  # TODO: _Image? Or exclude MapData?

    def mapful(self) -> bool:
        """Returns True if this image pair is for a map match

        :return: True for map match, False for visual odometry match
        """
        return isinstance(self.ref, ContextualMapData)  # TODO: get_args(Union[ContextualMapData, MapData]) ?


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class AsyncQuery:
    """Atomic pair that stores a :py:class:`multiprocessing.pool.AsyncResult` instance along with its input data

    The intention is to keep the result of the query in the same place along with the inputs so that they can be
    easily reunited again in the callback function. The :meth:`python_px4_ros2_map_nav.matchers.matcher.Matcher.worker`
    interface expects an image_pair and an input_data context as arguments (along with a guess which is not stored
    since it is no longer needed after the pose estimation).
    """
    result: AsyncResult
    #query: Union[ImagePair, ]  # TODO: what is used for WMS?
    image_pair: ImagePair  # TODO: what is used for WMS?
    input_data: InputData


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class Pose:
    """Represents camera pose (rotation and translation) along with camera intrinsics"""
    image_pair: ImagePair
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
        object.__setattr__(self, 'inv_h', np.linalg.inv(self.h))
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
        # TODO: what is image pair if this is a 'chained' pose? Need to reconstruct a new image pair from the two poses?
        return Pose(self.image_pair, self.k, self.r @ pose.r, self.t + self.r @ (pose.t + pose.camera_center))  # TODO: need to fix sign somehow? Would think minus sign is needed here?


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class InputData:
    """InputData of vehicle state and other variables needed for postprocessing both map and visual odometry matches.

    :param k: Camera intrinsics matrix from CameraInfo from time of match (from _match_inputs)
    :param camera_yaw: Camera yaw in radians from time of match (from _match_inputs)  # TODO: Rename map rotation so less confusion with gimbal attitude stuff extractd from rotation matrix?
    :param vehicle_attitude: Vehicle attitude
    :param map_dim_with_padding: Map dimensions with padding from time of match (from _match_inputs)
    :param img_dim: Drone image dimensions from time of match (from _match_inputs)
    :param vo_output_data_fix_map_pose: - Pose (chained) map pose from previous vo output data fix
    :param map_output_data_prev_pose: - Pose (unchained) (map) pose from previous map output data
    :return:
    """
    k: np.ndarray
    camera_yaw: np.ndarray
    vehicle_attitude: np.ndarray
    map_dim_with_padding: Dim
    img_dim: Dim
    vo_output_data_fix_map_pose: Optional[Pose]
    map_output_data_prev_pose: Optional[Pose]

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

    :param image_pair: The matching image pair
    :param input: The input data used for the match
    :param pose: Estimated pose for the image frame vs. the map frame
    :param pose_map: Estimated pose for the image frame vs. the map frame (in WGS84)
    :param fov: Camera field of view projected to WGS84 coordinates
    :param fov_pix: Camera field of view in pixels in reference frame (map or previous frame)
    :param position: Vehicle position in WGS84 (elevation or z coordinate in meters above mean sea level)
    :param terrain_altitude: Vehicle altitude in meters from ground (assumed starting altitude)
    :param attitude: Camera attitude quaternion
    :param c: Principal point projected to ground in WGS84 coordinates
    :param c_pix: Principal point projected in pixel coordinates
    :param sd: Standard deviation of position estimate
    :return:
    """
    input: InputData
    pose: Pose
    pose_map: Pose
    fov: Optional[np.ndarray]  # TODO: rename fov_wgs84? Can be None if can't be projected to WGS84?
    fov_pix: np.ndarray
    position: LatLonAlt
    terrain_altitude: float
    attitude: np.ndarray
    c: np.ndarray
    c_pix: np.ndarray
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
