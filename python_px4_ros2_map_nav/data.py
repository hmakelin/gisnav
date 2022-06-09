"""Module containing data structures to protect atomicity of related information"""
from __future__ import annotations  # Python version 3.7+

import cv2
import numpy as np
import os
import math

from functools import lru_cache
from xml.etree import ElementTree
from typing import Optional, Union, get_args
from collections import namedtuple
from dataclasses import dataclass, field
from multiprocessing.pool import AsyncResult
from scipy.spatial.transform import Rotation

from python_px4_ros2_map_nav.assertions import assert_type, assert_ndim, assert_shape, assert_len
from python_px4_ros2_map_nav.geo import GeoPoint, GeoTrapezoid
from python_px4_ros2_map_nav.transform import create_src_corners, get_fov_and_c

Dim = namedtuple('Dim', 'height width')
TimePair = namedtuple('TimePair', 'local foreign')

# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class Position:
    """Represents a 3D position with geographical xy-coordinates and an altitude expressed in meters

    Ground altitude is required while altitude above mean sea level (AMSL) is optional. If position is e.g. output from
    a Kalman filter, epx, epy and epz properties can also be provided.

    Note: In GeoPandas (x,y) is (lon, lat), while WGS84 coordinates are (lat, lon) by convention
    """
    xy: GeoPoint  # XY coordinates (e.g. longitude & latitude in WGS84)
    z_ground: float              # altitude above ground plane in meters (positive)
    z_amsl: Optional[float]      # altitude above mean sea level (AMSL) in meters if known (positive)
    x_sd: Optional[float]        # Standard deviation of error in x (latitude) dimension
    y_sd: Optional[float]        # Standard deviation of error in y (longitude) dimension
    z_sd: Optional[float]        # Standard deviation of error in z (altitude) dimension

    def __post_init__(self):
        """Set computed fields after initialization."""
        # Data class is frozen so need to use object.__setattr__ to assign values
        # TODO: enforce these checks instead of just asserting?
        assert all([self.eph, self.epv, self.x_sd, self.y_sd, self.z_sd]) \
               or not any([self.eph, self.epv, self.x_sd, self.y_sd, self.z_sd])

        # Need at least one kind of altitude info (should use GeoPoint otherwise)
        assert self.z_ground is not None or self.z_amsl is not None
        # assert self.z_ground is not None  # TODO: enable this once _position_from_vehicle_global_position usage is fixed (*NEED* ground altitude! only in the simulator scenario the amsl and ground altitude are the same)

    @property
    def eph(self) -> Optional[float]:
        """Standard deviation of horizontal error in meters (for GNSS/GPS)"""
        return max(self.x_sd, self.y_sd) if all([self.x_sd, self.y_sd]) else None

    @property
    def epv(self) -> Optional[float]:
        """Standard deviation of vertical error in meters (for GNSS/GPS)"""
        return self.z_sd if self.z_sd is not None else None

    @property
    def lat(self) -> float:
        """Convenience property to get latitude in WGS 84"""
        return self.xy.lat

    @property
    def lon(self) -> float:
        """Convenience property to get longitude in WGS 84"""
        return self.xy.lon

# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class Attitude:
    """Attitude (orientation) in 3D space, typically in FRD or NED frame depending on context"""
    q: np.ndarray  # (x, y, z, w) SciPy format! :class:`px4_msgs.VehicleAttitude` q has different (w, x, y, z) format
    roll: float = field(init=False)
    pitch: float = field(init=False)
    yaw: float = field(init=False)
    r: float = field(init=False)

    def __post_init__(self):
        """Post-initialization validity checks"""
        assert_len(self.q, 4)
        rotation = Rotation.from_quat(self.q)
        roll, pitch, yaw = tuple(rotation.as_euler('xyz'))
        object.__setattr__(self, 'roll', roll)
        object.__setattr__(self, 'pitch', pitch)
        object.__setattr__(self, 'yaw', yaw)
        object.__setattr__(self, 'r', rotation.as_matrix())

    def to_esd(self) -> Attitude:
        """Converts attitude from NED to solvePnP ESD world frame

        :return: Attitude in SED frame
        """
        nadir_pitch = np.array([0, np.sin(np.pi/4), 0, np.sin(np.pi/4)])  # Adjust origin to nadir facing camera
        r = Rotation.from_quat(self.q) * Rotation.from_quat(nadir_pitch)
        att = Attitude(r.as_quat())
        q = r.as_quat()
        q = np.array([q[1], -q[0], q[2], -q[3]])  # NED to ESD
        att = Attitude(q)
        return att

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
    camera_data: CameraData

    def __post_init__(self):
        """Post-initialization validity checks"""
        assert_type(self.camera_data, CameraData)


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class CameraData:
    """Camera intrinsics matrix"""
    k: np.ndarray
    height: float
    width: float
    fx: float = field(init=False)
    fy: float = field(init=False)
    cx: float = field(init=False)
    cy: float = field(init=False)

    def __post_init__(self):
        """Post-initialization validity checks"""
        assert_shape(self.k, (3, 3))
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
    bbox: GeoBox


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class ContextualMapData(_ImageHolder):
    """Contains the rotated and cropped map image for _match estimation"""
    image: Img = field(init=False)  # This is the map_cropped image which is same size as the camera frames, init in __post_init__
    rotation: float
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
        dst_corners = self.map_data.bbox.get_coordinates('epsg:4326')
        unrotated_to_wgs84 = cv2.getPerspectiveTransform(np.float32(src_corners).squeeze(),
                                                         np.float32(dst_corners).squeeze())

        pix_to_wgs84_ = unrotated_to_wgs84 @ uncropped_to_unrotated @ pix_to_uncropped
        return pix_to_wgs84_  # , unrotated_to_wgs84, uncropped_to_unrotated, pix_to_uncropped

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
    ref: ContextualMapData

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
    """Represents camera match (rotation and translation)

    :raise: ValueError if r or t is invalid (np.isnan)
    """
    r: np.ndarray
    t: np.ndarray
    e: np.ndarray = field(init=False)

    def __post_init__(self):
        """Set computed fields after initialization."""
        # Data class is frozen so need to use object.__setattr__ to assign values
        object.__setattr__(self, 'e', np.hstack((self.r, self.t)))  # -self.r.T @ self.t

        # Validity check
        if np.isnan(self.r).any() or np.isnan(self.t).any():
            raise ValueError(f'Rotation matrix or translation vector contained NaNs:\n{self.r}, {self.t}')


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class Match:
    """Represents a matched image pair with estimated match and camera position

    :raise: np.linalg.LinAlgError if homography matrix is not invertible
    """
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
        object.__setattr__(self, 'h', img.camera_data.k @ np.delete(self.pose.e, 2, 1))  # Remove z-column, making the matrix square
        object.__setattr__(self, 'inv_h', np.linalg.inv(self.h))
        object.__setattr__(self, 'camera_position', -self.pose.r.T @ self.pose.t)
        object.__setattr__(self, 'camera_center', np.array((img.camera_data.cx, img.camera_data.cy, -img.camera_data.fx)).reshape((3, 1)))  # TODO: assumes fx == fy
        object.__setattr__(self, 'camera_position_difference', self.camera_position - self.camera_center)

    def __matmul__(self, match: Match) -> Match:  # Python version 3.5+
        """Matrix multiplication operator for convenience

        Returns a new Match by combining two matches by chaining the poses and image pairs: a new 'synthetic' image
        pair is created by combining the two others.
        """
        assert (self.image_pair.qry.camera_data.k == match.image_pair.qry.camera_data.k).all(), 'Camera intrinsic matrices are not equal'  # TODO: validation, not assertion
        H = self.h @ match.h
        num, Rs, Ts, Ns = cv2.decomposeHomographyMat(H, self.image_pair.qry.camera_data.k)  # TODO: try to do this without decomposing H
        index = 0  # TODO: how to pick index? Need to compare camera normals?
        r, t = Rs[index], Ts[index]
        t = match.image_pair.qry.camera_data.fx * t  # scale by focal length  # TODO: assume fx == fy
        # TODO: handle pose ValueError
        return Match(
                image_pair=ImagePair(qry=self.image_pair.qry, ref=match.image_pair.ref),
                pose=Pose(
                    r,
                    -(r @ match.camera_center + t)
                )
        )


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class InputData:
    """InputData of vehicle state and other variables needed for postprocessing both map and visual odometry matches.

    :return:
    """
    ground_elevation: Optional[float]  # Assumed elevation of ground plane above mean sea level (AMSL)

    def __post_init__(self):
        """Validate the data structure"""
        # TODO: Enforce types
        pass


# noinspection PyClassHasNoInit
@dataclass
class FOV:
    """Camera field of view related attributes"""
    fov_pix: GeoTrapezoid  # np.ndarray  # TODO: not real GEOtrapezoid, just trapezoid but need shapely functions!
    fov: Optional[GeoTrapezoid]  #Optional[np.ndarray]  # TODO: rename fov_wgs84? Can be None if can't be projected to WGS84?
    c: GeoPoint
    c_pix: np.ndarray  # TODO: try to get proj string for fov_pix, then can also make this a GeoPoint
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
        distance_in_pixels = self.fov_pix.length
        distance_in_meters = self.fov.meter_length

        # TODO: this is vulnerable to the top of the FOV 'escaping' into the horizon, should just use bottom of FOV
        altitude_scaling = abs(distance_in_meters / distance_in_pixels)

        return altitude_scaling

    # TODO: make this private, and use a new "is_valid()" to combine this condition with the shapely built in is_valid flag
    def is_convex_isosceles_trapezoid(self, diagonal_length_tolerance: float = 0.1) -> bool:
        """Returns True if provided quadrilateral is a convex isosceles trapezoid

        If the estimated field of view (FOV) is not a convex isosceles trapezoid, it is a sign that (1) the match was bad or
        (2) the gimbal the camera is mounted on has not had enough time to stabilize (camera has non-zero roll). Matches
        where the FOV is not a convex isosceles trapezoid should be rejected assuming we can't determine (1) from (2) and
        that it is better to wait for a good position estimate than to use a bad one.

        See also :func:`~create_src_corners` for the assumed order of the quadrilateral corners.

        :param diagonal_length_tolerance: Tolerance for relative length difference between trapezoid diagonals
        :return: True if the quadrilateral is a convex isosceles trapezoid
        """
        fov_pix = self.fov_pix.get_coordinates()
        assert_len(fov_pix, 4)
        ul, ll, lr, ur = tuple(map(lambda pt: pt.squeeze().tolist(), fov_pix))

        # Check convexity (exclude self-crossing trapezoids)
        # Note: inverted y-axis, origin at upper left corner of image
        if not (ul[0] < ur[0] and ul[1] < ll[1] and lr[0] > ll[0] and lr[1] > ur[1]):
            return False

        # Check diagonals same length within tolerance
        ul_lr_length = math.sqrt((ul[0] - lr[0]) ** 2 + (ul[1] - lr[1]) ** 2)
        ll_ur_length = math.sqrt((ll[0] - ur[0]) ** 2 + (ll[1] - ur[1]) ** 2)
        if abs((ul_lr_length / ll_ur_length) - 1) > diagonal_length_tolerance:
            return False

        return True

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
    position: Position = field(init=False)
    map_match: Match

    def _estimate_fov(self) -> FOV:
        """Estimates field of view and principal point in both pixel and WGS84 coordinates

        :return: Field of view and principal point in pixel and WGS84 coordinates, respectively
        """
        # TODO: what if wgs84 coordinates are not valid? H projects FOV to horizon?
        assert_type(self.map_match.image_pair.ref, ContextualMapData)  # Need pix_to_wgs84, FixedCamera should have map data match
        h_wgs84 = self.map_match.image_pair.ref.pix_to_wgs84 @ self.map_match.inv_h
        fov_pix, c_pix = get_fov_and_c(self.map_match.image_pair.qry.image.dim, self.map_match.inv_h)
        fov_wgs84, c_wgs84 = get_fov_and_c(self.map_match.image_pair.ref.image.dim, h_wgs84)

        fov = FOV(fov_pix=GeoTrapezoid(np.flip(fov_pix, axis=2), crs=''),  # TODO: can we give it a crs? Or edit GeoTrapezoid to_crs so that it returns an error if crs not given
                  fov=GeoTrapezoid(np.flip(fov_wgs84, axis=2), crs='epsg:4326'),  # TODO: rename these just "pix" and "wgs84", redundancy in calling them fov_X
                  c_pix=c_pix,
                  c=GeoPoint(*c_wgs84.squeeze()[::-1], crs='epsg:4326')
                  )

        return fov

    def _estimate_position(self, ground_elevation: Optional[float], crs: str = 'epsg:4326') -> Position:
        """Estimates camera position (WGS84 coordinates + altitude in meters above mean sea level (AMSL)) as well as
        terrain altitude in meters.

        :param ground_elevation: Optional ground elevation (needed to estimate altitude from sea level)
        :param crs: CRS to use for the Position
        :return: Camera position
        """
        assert self.fov is not None  # Call _estimate_fov before _estimate_position!
        # Translation in WGS84 (and altitude or z-axis translation in meters above ground)
        assert_type(self.map_match.image_pair.ref, ContextualMapData)  # need pix_to_wgs84
        t_wgs84 = self.map_match.image_pair.ref.pix_to_wgs84 @ np.append(self.map_match.camera_position[0:2], 1)
        t_wgs84[2] = -self.fov.scaling * self.map_match.camera_position[2]  # In NED frame z-coordinate is negative above ground, make altitude >0

        # Check that we have all the values needed for a global position
        if not all([(isinstance(x, float) or np.isnan(x)) for x in t_wgs84.squeeze()]):
            self.get_logger().warn(f'Could not determine global position, some fields were empty: {t_wgs84}.')
            return None

        lon, lat = t_wgs84.squeeze()[1::-1]
        alt = t_wgs84[2]
        position = Position(
            xy=GeoPoint(lon, lat, crs),  # lon-lat order
            z_ground=alt,
            z_amsl=alt + ground_elevation if ground_elevation is not None else None,
            x_sd=None,
            y_sd=None,
            z_sd=None
        )

        return position

    def __post_init__(self):
        """Set computed fields after initialization."""
        # Data class is frozen so need to use object.__setattr__ to assign values
        object.__setattr__(self, 'fov', self._estimate_fov())  # Need to do before calling self._estimate_position
        object.__setattr__(self, 'position', self._estimate_position(self.ground_elevation))


# noinspection PyClassHasNoInit
@dataclass
class OutputData:
    # TODO: add extrinsic matrix / _match, pix_to_wgs84 transformation?
    # TODO: freeze this data structure to reduce unintentional re-assignment?
    """Algorithm output passed onto publish method.

    :param input: The input data used for the match
    :param fixed_camera: Camera that is fixed to wgs84 coordinates (map_match and field of view)
    :param filtered_position: Filtered position from the Kalman filter
    :param attitude: Camera attitude quaternion
    :return:
    """
    input: InputData
    fixed_camera: FixedCamera
    filtered_position: Optional[Position]  # TODO: currently added post init, thence Optional
    attitude: np.ndarray

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
