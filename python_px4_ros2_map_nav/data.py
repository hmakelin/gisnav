"""Module containing data structures to protect atomicity of related information"""
from __future__ import annotations  # Python version 3.7+

import cv2
import numpy as np
import os
import math

from xml.etree import ElementTree
from typing import Optional, Union, get_args
from collections import namedtuple
from dataclasses import dataclass, field
from multiprocessing.pool import AsyncResult

from python_px4_ros2_map_nav.assertions import assert_type, assert_ndim, assert_shape
from python_px4_ros2_map_nav.transform import create_src_corners, get_fov_and_c
from python_px4_ros2_map_nav.proj import Proj

BBox = namedtuple('BBox', 'left bottom right top')  # Convention: https://wiki.openstreetmap.org/wiki/Bounding_Box
LatLon = namedtuple('LatLon', 'lat lon')
LatLonAlt = namedtuple('LatLonAlt', 'lat lon alt')
Dim = namedtuple('Dim', 'height width')
#RPY = namedtuple('RPY', 'roll pitch yaw')
TimePair = namedtuple('TimePair', 'local foreign')


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class RPY:
    """Roll-pitch-yaw"""
    roll: float
    pitch: float
    yaw: float

    def axes_ned_to_image(self, degrees: bool = True) -> RPY:
        """Converts roll-pitch-yaw euler angles from NED to image frame"""
        straight_angle = 180 if degrees else np.pi
        right_angle = 90 if degrees else np.pi / 2
        pitch = -self.pitch - right_angle
        if pitch < 0:
            # Gimbal pitch and yaw flip over when abs(gimbal_yaw) should go over 90, adjust accordingly
            pitch += straight_angle
        roll = pitch
        pitch = self.roll
        yaw = self.yaw
        rpy = RPY(roll, pitch, yaw)
        return rpy

# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class _ImageHolder:
    """Parent dataclass for image holders

    Should not be instantiated directly.
    """
    image: np.ndarray


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class Img:
    """Image class to hold image raster and related metadata"""
    arr: np.ndarray
    dim: Dim = field(init=False)

    def __post_init__(self):
        """Set computed fields after initialization."""
        # Data class is frozen so need to use object.__setattr__ to assign values
        object.__setattr__(self, 'dim', Dim(*self.arr.shape[0:2]))  # TODO: correct order of unpack?


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class ImageData(_ImageHolder):
    """Keeps image frame related data in one place and protects it from corruption."""
    #image: Img
    frame_id: str
    timestamp: int
    k: np.ndarray  # TODO: make new "CameraIntrinsics" structure
    fx: float = field(init=False)
    fy: float = field(init=False)
    cx: float = field(init=False)  # TODO: int?
    cy: float = field(init=False)  # TODO: int?

    def __post_init__(self):
        """Set computed fields after initialization."""
        # Data class is frozen so need to use object.__setattr__ to assign values
        object.__setattr__(self, 'fx', self.k[0][0])
        object.__setattr__(self, 'fy', self.k[1][1])
        object.__setattr__(self, 'cx', self.k[0][2])
        object.__setattr__(self, 'cy', self.k[1][2])


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class MapData(_ImageHolder):
    """Keeps map frame related data in one place and protects it from corruption."""
    #image: Img
    center: Union[LatLon, LatLonAlt]
    radius: Union[int, float]
    bbox: BBox


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class ContextualMapData(_ImageHolder):
    """Contains the rotated and cropped map image for _match estimation"""
    image: Img = field(init=False)  # This is the map_cropped image which is same size as the camera frames, init in __post_init__
    rotation: Union[float, int]
    crop: Dim  # TODO: Redundant with .image.dim but need this to generate .image
    map_data: MapData   # This is the original larger (square) map with padding
    pix_to_wgs84: np.ndarray = field(init=False)

    def _pix_to_wgs84(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Returns tuple of affine 2D transformation matrix for converting matched pixel coordinates to WGS84 coordinates
        along with intermediate transformations

        These transformations can be used to reverse the rotation and cropping that :func:`~rotate_and_crop_map` did to
        the original map.

        :return: Tuple containing 2D affinre transformations from 1. pixel coordinates to WGS84, 2. from original unrotated
        and uncropped map pixel coordinates to WGS84, 3. from rotated map coordinates to unrotated map coordinates, and 4.
        from cropped map coordinates to uncropped (but still rotated) map pixel coordinates.
        """
        map_dim_arr = np.array(self.map_data.image.dim)
        img_dim_arr = np.array(self.image.dim)
        crop_padding = map_dim_arr - img_dim_arr
        crop_translation = (crop_padding / 2)
        pix_to_uncropped = np.identity(3)
        # Invert order x<->y in translation vector since height comes first in Dim tuple (inputs should be Dims)
        pix_to_uncropped[0:2][:, 2] = crop_translation[::-1]

        rotation_center = map_dim_arr / 2
        rotation = cv2.getRotationMatrix2D(rotation_center, np.degrees(-self.rotation), 1.0)
        rotation_padding = np.array([[0, 0, 1]])
        uncropped_to_unrotated = np.vstack((rotation, rotation_padding))

        src_corners = create_src_corners(*self.map_data.image.dim)
        dst_corners = self._bbox_to_polygon(self.map_data.bbox)
        unrotated_to_wgs84 = cv2.getPerspectiveTransform(np.float32(src_corners).squeeze(),
                                                         np.float32(dst_corners).squeeze())

        pix_to_wgs84_ = unrotated_to_wgs84 @ uncropped_to_unrotated @ pix_to_uncropped
        return pix_to_wgs84_  # , unrotated_to_wgs84, uncropped_to_unrotated, pix_to_uncropped

    # TODO: make bbox dataclass, make this public method for that class
    @staticmethod
    def _bbox_to_polygon(bbox: BBox) -> np.ndarray:
        """Converts BBox to a np.ndarray polygon format

        :param bbox: BBox to convert
        :return: bbox corners in np.ndarray"""
        return np.array([[bbox.top, bbox.left],
                         [bbox.bottom, bbox.left],
                         [bbox.bottom, bbox.right],
                         [bbox.top, bbox.right]]).reshape(-1, 1, 2)

    def _rotate_and_crop_map(self) -> np.ndarray:
        """Rotates map counter-clockwise and then crops a dimensions-sized part from the middle.

        Map needs padding so that a circle with diameter of the diagonal of the img_size rectangle is enclosed in map.

        :return: Rotated and cropped map raster
        """
        cx, cy = tuple(np.array(self.map_data.image.arr.shape[0:2]) / 2)  # TODO: Use k, dim etc?
        degrees = math.degrees(self.rotation)
        r = cv2.getRotationMatrix2D((cx, cy), degrees, 1.0)
        map_rotated = cv2.warpAffine(self.map_data.image.arr, r, self.map_data.image.arr.shape[1::-1])  # TODO: use .dim?
        map_cropped = self._crop_center(map_rotated, self.crop)  # TODO: just pass img_dim when initializing ContextualMapData?
        #if visualize:
        #if True:
            #cv2.imshow('padded', self.map_data.image.arr)
            #cv2.waitKey(1)
            #cv2.imshow('rotated', map_rotated)
            #cv2.waitKey(1)
            #cv2.imshow('cropped', map_cropped)
            #cv2.waitKey(1)
        # TODO: below assertion should not be!
        assert map_cropped.shape[0:2] == self.crop, f'Cropped shape {map_cropped.shape} did not match dims {self.crop}.'
        return map_cropped

    @staticmethod
    def _crop_center(img: np.ndarray, dimensions: Dim) -> np.ndarray:
        """Crops dimensions sized part from center.

        :param img: Image to crop
        :param dimensions: Dimensions of area to crop (not of image itself)
        :return: Cropped image
        """
        cx, cy = tuple(np.array(img.shape[0:2]) / 2)
        img_cropped = img[math.floor(cy - dimensions.height / 2):math.floor(cy + dimensions.height / 2),
                      math.floor(cx - dimensions.width / 2):math.floor(cx + dimensions.width / 2)]
        assert (
        img_cropped.shape[0:2] == dimensions.height, dimensions.width), 'Something went wrong when cropping the ' \
                                                                        'map raster. '
        return img_cropped

    def __post_init__(self):
        """Set computed fields after initialization."""
        # Data class is frozen so need to use object.__setattr__ to assign values
        #super().__post_init__()
        object.__setattr__(self, 'image', Img(self._rotate_and_crop_map()))  # TODO: correct order of unpack?
        object.__setattr__(self, 'pix_to_wgs84', self._pix_to_wgs84())  # TODO: correct order of unpack?


# TODO: enforce types for ImagePair (qry cannot be MapData, can happen if _match.__matmul__ is called in the wrong order! E.g. inside _estimate_map_pose
# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class ImagePair:
    """Atomic image pair to represent a matched pair of images"""
    qry: ImageData
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
    since it is no longer needed after the _match estimation).
    """
    result: AsyncResult
    #query: Union[ImagePair, ]  # TODO: what is used for WMS?
    image_pair: ImagePair  # TODO: what is used for WMS?
    input_data: InputData


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class Pose:
    """Represents camera match (rotation and translation)"""
    r: np.ndarray
    t: np.ndarray
    e: np.ndarray = field(init=False)

    def __post_init__(self):
        """Set computed fields after initialization."""
        # Data class is frozen so need to use object.__setattr__ to assign values
        object.__setattr__(self, 'e', np.hstack((self.r, self.t)))  # -self.r.T @ self.t


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class Match:
    """Represents a matched image pair with estimated match and camera position"""
    image_pair: ImagePair
    pose: Pose
    h: np.ndarray = field(init=False)
    inv_h: np.ndarray = field(init=False)
    camera_position: np.ndarray = field(init=False)
    camera_center: np.ndarray = field(init=False)
    camera_position_difference: np.ndarray = field(init=False)

    def __post_init__(self):
        """Set computed fields after initialization."""
        # Data class is frozen so need to use object.__setattr__ to assign values
        img = self.image_pair.qry
        object.__setattr__(self, 'h', img.k @ np.delete(self.pose.e, 2, 1))  # Remove z-column, making the matrix square
        object.__setattr__(self, 'inv_h', np.linalg.inv(self.h))
        object.__setattr__(self, 'camera_position', -self.pose.r.T @ self.pose.t)
        object.__setattr__(self, 'camera_center', np.array((img.cx, img.cy, -img.fx)).reshape((3, 1)))  # TODO: assumes fx == fy
        object.__setattr__(self, 'camera_position_difference', self.camera_position - self.camera_center)

    def __matmul__(self, match: Match) -> Match:  # Python version 3.5+
        """Matrix multiplication operator for convenience

        Returns a new Match by combining two matches by chaining the poses and image pairs: a new 'synthetic' image
        pair is created by combining the two others.
        """
        assert match.image_pair.mapful()  # Not ideal assumption, map match must always be rhs of operation
        assert (self.image_pair.qry.k == match.image_pair.qry.k).all(), 'Camera intrinsic matrices are not equal'  # TODO: validation, not assertion
        H = match.inv_h @ self.inv_h
        H_inv = np.linalg.inv(H)  # TODO: do not invert, do self.h @ match.h instead
        num, Rs, Ts, Ns = cv2.decomposeHomographyMat(H_inv, self.image_pair.qry.k)
        index = 0  # TODO: how to pick index? Need to compare camera normals?
        r, t = Rs[index], Ts[index]
        t = match.image_pair.qry.fx * t  # scale by focal length  # TODO: assume fx == fy
        return Match(
                image_pair=ImagePair(qry=self.image_pair.qry, ref=match.image_pair.ref),
                pose=Pose(
                    r,
                    -(r @ match.camera_center + t)
                )  # TODO: need to fix sign somehow? Would think minus sign is needed here?
        )


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class InputData:
    """InputData of vehicle state and other variables needed for postprocessing both map and visual odometry matches.

    :param vo_fix: - The WGS84-fixed FixedCamera for the VO reference frame, or None if not available
    :return:
    """
    vo_fix: Optional[FixedCamera]  # None if successful map match has not yet happened
    ground_elevation: Optional[float]  # Assumed elevation of ground plane above mean sea level (AMSL)

    def __post_init__(self):
        """Validate the data structure"""
        # TODO: Enforce types
        pass


# noinspection PyClassHasNoInit
@dataclass
class FOV:
    """Camera field of view related attributes"""
    fov_pix: np.ndarray
    fov: Optional[np.ndarray]  # TODO: rename fov_wgs84? Can be None if can't be projected to WGS84?
    c: np.ndarray
    c_pix: np.ndarray
    fov_pix_map: np.ndarray  # Fov in pixels against the map image if the original image_pair was a VO match (not mapful)
    c_pix_map: np.ndarray  # Fov in pixels against the map image if the original image_pair was a VO match (not mapful)
    scaling: float = field(init=False)

    # TODO: how to estimate if fov_wgs84 is zero (cannot be projected because camera pitch too high)?
    def _estimate_altitude_scaling(self) -> float:
        """Estimates altitude scaling factor from field of view matched against known map

        Altitude in t is in rotated and cropped map raster pixel coordinates. We can use fov_pix and fov_wgs84 to
        find out the right scale in meters. Distance in pixels is computed from lower left and lower right corners
        of the field of view (bottom of fov assumed more stable than top), while distance in meters is computed from
        the corresponding WGS84 latitude and latitude coordinates.

        :return: Altitude scaling factor
        """
        proj = Proj.instance()  # Get cached geod instance
        distance_in_pixels = np.linalg.norm(self.fov_pix_map[1]-self.fov_pix_map[2])  # fov_pix[1]: lower left, fov_pix[2]: lower right
        distance_in_meters = proj.distance(LatLon(*self.fov[1].squeeze().tolist()),
                                           LatLon(*self.fov[2].squeeze().tolist()))
        altitude_scaling = abs(distance_in_meters / distance_in_pixels)

        return altitude_scaling

    def __post_init__(self):
        """Set computed fields after initialization."""
        # Data class is frozen so need to use object.__setattr__ to assign values
        object.__setattr__(self, 'scaling', self._estimate_altitude_scaling())


# noinspection PyClassHasNoInit
@dataclass
class FixedCamera:
    """WGS84-fixed camera attributes

    Colletcts field of view and map_match under a single structure that is intended to be stored in input data context as
    visual odometry fix reference. Includes the needed map_match and pix_to_wgs84 transformation for the vo fix.
    """
    fov: FOV = field(init=False)  # TODO: move down to Match and get rid of FixedCamera? Use 'Match' for storing the vo fix instead
    ground_elevation: Optional[float]
    position: np.ndarray = field(init=False)
    terrain_altitude: np.ndarray = field(init=False)
    map_match: Match
    _match: Match  # should not be accessed directly except e.g. for debug visualization (fov_pix), use fixed_camera.map_match instead

    def _estimate_fov(self) -> FOV:
        """Estimates field of view and principal point in both pixel and WGS84 coordinates

        :return: Field of view and principal point in pixel and WGS84 coordinates, respectively
        """
        # TODO: what if wgs84 coordinates are not valid? H projects FOV to horizon?
        assert_type(self.map_match.image_pair.ref, ContextualMapData)  # Need pix_to_wgs84, FixedCamera should have map data match
        h_wgs84 = self.map_match.image_pair.ref.pix_to_wgs84 @ self.map_match.inv_h
        fov_pix_map, c_pix_map = get_fov_and_c(self.map_match.image_pair.qry.image.dim, self.map_match.inv_h)
        fov_pix, c_pix = get_fov_and_c(self.map_match.image_pair.qry.image.dim, self._match.inv_h)
        fov_wgs84, c_wgs84 = get_fov_and_c(self.map_match.image_pair.ref.image.dim, h_wgs84)

        fov = FOV(fov_pix=fov_pix,
                  fov=fov_wgs84,  # TODO: rename these just "pix" and "wgs84", redundancy in calling them fov_X
                  c_pix=c_pix,
                  c=c_wgs84,
                  fov_pix_map=fov_pix_map,
                  c_pix_map=c_pix_map
                  )

        return fov

    def _estimate_position(self, ground_elevation: Optional[float]) -> Tuple[LatLonAlt, float]:
        """Estimates camera position (WGS84 coordinates + altitude in meters above mean sea level (AMSL)) as well as
        terrain altitude in meters.

        :param ground_elevation: Optional ground elevation (needed to estimate altitude from sea level)
        :return: Camera position LatLonAlt, and altitude from ground in meters
        """
        assert self.fov is not None  # Call _estimate_fov before _estimate_position!
        # Translation in WGS84 (and altitude or z-axis translation in meters above ground)
        assert_type(self.map_match.image_pair.ref, ContextualMapData)  # need pix_to_wgs84
        t_wgs84 = self.map_match.image_pair.ref.pix_to_wgs84 @ np.append(self.map_match.camera_position[0:2], 1)
        t_wgs84[2] = -self.fov.scaling * self.map_match.camera_position[2]  # In NED frame z-coordinate is negative above ground, make altitude >0
        position = t_wgs84.squeeze().tolist()
        position = LatLonAlt(*position)

        # Check that we have all the values needed for a global position
        # if not all(position) or any(map(np.isnan, position)):
        if not all([(isinstance(x, float) or np.isnan(x)) for x in position]):
            self.get_logger().warn(f'Could not determine global position, some fields were empty: {position}.')
            return None

        # Get altitude above mean sea level (AMSL)
        terrain_altitude = position.alt
        if ground_elevation is None:
            # TODO: need to give warning of lack of altitude above amsl estimate?
            #self.get_logger().debug('Ground plane elevation (AMSL) unavailable. Setting position.alt as None.')  # TODO: or return LatLon instead?
            position = LatLonAlt(*position[0:2], None)
        else:
            position = LatLonAlt(*position[0:2], position.alt + ground_elevation)

        return position, terrain_altitude

    def __post_init__(self):
        """Set computed fields after initialization."""
        # Data class is frozen so need to use object.__setattr__ to assign values
        object.__setattr__(self, 'fov', self._estimate_fov())  # Need to do before calling self._estimate_position
        position, terrain_altitude = self._estimate_position(self.ground_elevation)
        object.__setattr__(self, 'position', position)
        object.__setattr__(self, 'terrain_altitude', terrain_altitude)


# noinspection PyClassHasNoInit
@dataclass
class OutputData:
    # TODO: add extrinsic matrix / _match, pix_to_wgs84 transformation?
    # TODO: freeze this data structure to reduce unintentional re-assignment?
    """Algorithm output passed onto publish method.

    :param input: The input data used for the match
    :param _match: Estimated _match for the image frame vs. the map frame
    :param fixed_camera: Camera that is fixed to wgs84 coordinates (map_match and field of view)
    :param position: Vehicle position in WGS84 (elevation or z coordinate in meters above mean sea level)
    :param terrain_altitude: Vehicle altitude in meters from ground (assumed starting altitude)
    :param attitude: Camera attitude quaternion
    :param sd: Standard deviation of position estimate
    :return:
    """
    input: InputData
    fixed_camera: FixedCamera
    #position: LatLonAlt
    #terrain_altitude: float
    attitude: np.ndarray
    sd: np.ndarray  # TODO This should be part of Position? Keep future position dataclass mutable so this can be assigned while outputdata itself is immutable

    # Target structure:
    # input
    # vehicle (position, attitude, terrain_altitude, sd)
    # camera (map_match, fov)  +  camera attitude which is actually what we have now

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
