"""Module containing immutable dataclasses to protect atomicity of related information (object-based 'contexts')"""
from __future__ import annotations  # Python version 3.7+

import cv2
import numpy as np
import os
import math
import warnings
warnings.filterwarnings(action='ignore', category=UserWarning, message='Gimbal lock detected.')

from xml.etree import ElementTree
from typing import Optional
from dataclasses import dataclass, field
from collections import namedtuple
from multiprocessing.pool import AsyncResult
from scipy.spatial.transform import Rotation

from gisnav.assertions import assert_type, assert_ndim, assert_shape, assert_len
from gisnav.geo import GeoPoint, GeoTrapezoid, GeoValueError

Dim = namedtuple('Dim', 'height width')
TimePair = namedtuple('TimePair', 'local foreign')


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class Position:
    """Represents a 3D position with geographical xy-coordinates and an altitude expressed in meters

    Ground altitude is required while altitude above mean sea level (AMSL) is optional. If position is e.g. output from
    a Kalman filter, x_sd, y_sd and z_sd properties can also be provided.

    .. note::
        (x, y, z) coordinates are in ENU frame
    """
    xy: GeoPoint                # XY coordinates (e.g. longitude & latitude in WGS84)
    z_ground: float             # altitude above ground plane in meters (positive)
    attitude: Attitude          # attitude in NED frame
    timestamp: int              # Reference timestamp of position
    z_amsl: Optional[float]     # altitude above mean sea level (AMSL) in meters if known (positive)
    x_sd: Optional[float]       # Standard deviation of error in x (latitude) dimension
    y_sd: Optional[float]       # Standard deviation of error in y (longitude) dimension
    z_sd: Optional[float]       # Standard deviation of error in z (altitude) dimension

    _KALMAN_FILTER_EPSG_CODE = 'epsg:3857'
    """Used for converting into an array that can be passed to :class:`.SimpleFilter"""

    def __post_init__(self):
        """Set computed fields after initialization

        :raise: :class:`.DataValueError` if r or t is invalid
        """
        assert all([self.eph, self.epv, self.x_sd, self.y_sd, self.z_sd]) \
               or not any([self.eph, self.epv, self.x_sd, self.y_sd, self.z_sd])

        assert self.z_ground is not None

        if self.z_ground < 0:
            raise DataValueError(f'Ground altitude was negative {self.z_ground}.')

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

    def to_array(self) -> np.ndarray:
        """Returns position (x, y, z) coordinates in (adjusted EPSG:3857 for xy) meters as numpy array

        Intended to be used to convert the position into an array that can be passed onto :class:`.SimpleFilter`.
        """
        return np.append(np.array(self.xy.to_crs(self._KALMAN_FILTER_EPSG_CODE).coords),
                         np.array(self.z_ground)).reshape(1, 3)


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
        q = r.as_quat()
        q = np.array([q[1], -q[0], q[2], -q[3]])  # NED to ESD
        att = Attitude(q)
        return att


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class _ImageHolder:
    """Parent dataclass for image holders

    .. note::
        This class should not be instantiated directly.
    """
    image: Img


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class Img:
    """Class to hold image raster and related metadata"""
    arr: np.ndarray
    dim: Dim = field(init=False)

    def __post_init__(self):
        """Set computed variables post-initialization"""
        object.__setattr__(self, 'dim', Dim(*self.arr.shape[0:2]))  # order is h, w, c


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class ImageData(_ImageHolder):
    """Keeps image frame related data in one place and protects it from corruption."""
    frame_id: str
    timestamp: int
    camera_data: CameraData


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class CameraData:
    """Camera intrinsics matrix"""
    k: np.ndarray
    dim: Dim
    fx: float = field(init=False)
    fy: float = field(init=False)
    cx: float = field(init=False)
    cy: float = field(init=False)

    def __post_init__(self):
        """Set computed variables post-initialization"""
        assert_shape(self.k, (3, 3))
        object.__setattr__(self, 'fx', self.k[0][0])
        object.__setattr__(self, 'fy', self.k[1][1])
        object.__setattr__(self, 'cx', self.k[0][2])
        object.__setattr__(self, 'cy', self.k[1][2])


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class MapData(_ImageHolder):
    """Keeps map frame related data in one place and protects it from corruption."""
    bbox: GeoBBox


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class ContextualMapData(_ImageHolder):
    """Contains the rotated and cropped map image for _match estimation"""
    image: Img = field(init=False)  # This is the cropped and rotated map which is same size as the camera frames
    rotation: float                 # radians
    crop: Dim                       # Same value will also be found at image.dim
    map_data: MapData               # This is the original (square) map with padding
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
        dst_corners = self.map_data.bbox.to_crs('epsg:4326').coords  # .reshape(-1, 1, 2)
        dst_corners = np.flip(dst_corners, axis=1)  # from ENU frame to WGS 84 axis order
        unrotated_to_wgs84 = cv2.getPerspectiveTransform(np.float32(src_corners).squeeze(),
                                                         np.float32(dst_corners).squeeze())

        # Ratio of boundaries in pixels and meters -> Altitude (z) scaling factor
        vertical_scaling = abs(self.map_data.bbox.meter_length / \
                               (2*self.map_data.image.dim.width + 2*self.map_data.image.dim.height))

        pix_to_wgs84_ = unrotated_to_wgs84 @ uncropped_to_unrotated @ pix_to_uncropped

        pix_to_wgs84_[2][2] = -vertical_scaling * pix_to_wgs84_[2][2]

        # TODO: call it 'affine' instead?
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
            #cv2.imshow('padded', self.map_data.image.arr)
            #cv2.waitKey(1)
            #cv2.imshow('rotated', map_rotated)
            #cv2.waitKey(1)
            #cv2.imshow('cropped', map_cropped)
            #cv2.waitKey(1)
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
        object.__setattr__(self, 'image', Img(self._rotate_and_crop_map()))  # TODO: correct order of unpack?
        object.__setattr__(self, 'pix_to_wgs84', self._pix_to_wgs84())  # TODO: correct order of unpack?


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class ImagePair:
    """Atomic image pair to represent a matched pair of images"""
    qry: ImageData
    ref: ContextualMapData


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class _AsyncQuery:
    """Abstract base class of a structure that stores a :py:class:`multiprocessing.pool.AsyncResult` instance along
    with its input data

    The intention is to keep the result of the query in the same place along with the inputs so that they can be
    easily reunited again in the callback function.

    .. note::
        You should not try to instantiate this class directly. Use child classes instead.
    """
    result: AsyncResult


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class AsyncPoseQuery(_AsyncQuery):
    """Stores a :py:class:`multiprocessing.pool.AsyncResult` instance along with its input data

    The :meth:`.PoseEstimator.worker` interface expects an image_pair (query, reference images and camera intrinsics matrix)
    and an input_data context as arguments (along with a guess which is not stored since it is no longer needed after
    the _match estimation).
    """
    image_pair: ImagePair
    input_data: InputData


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class AsyncWMSQuery(_AsyncQuery):
    """Stores a :py:class:`multiprocessing.pool.AsyncResult` instance along with its input data

    The :meth:`.WMSClient.worker` expects the :class:`.GeoSquare` bounds as input it is needed here
    """
    geobbox: GeoSquare


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class Pose:
    """Represents camera match (rotation and translation)"""
    r: np.ndarray
    t: np.ndarray
    e: np.ndarray = field(init=False)

    def __post_init__(self):
        """Set computed fields and do validity checks after initialization

        :raise: :class:`.DataValueError` if r or t is invalid
        """
        # Data class is frozen so need to use object.__setattr__ to assign values
        object.__setattr__(self, 'e', np.hstack((self.r, self.t)))

        # Validity checks
        if np.isnan(self.r).any() or np.isnan(self.t).any() \
            or self.r.shape != (3, 3) or self.t.shape != (3, 1):
            raise DataValueError(f'Pose input arguments were invalid: {r}, {t}.')

    def __iter__(self):
        """Convenience interface for converting e.g. to tuple"""
        for item in (self.r, self.t):
            yield item


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class InputData:
    """InputData of vehicle state and other variables needed for postprocessing pose estimates"""
    r_guess: Optional[np.ndarray]
    """Camera expected attitude"""

    ground_elevation: Optional[float]
    """Assumed elevation of ground plane above mean sea level (AMSL)"""


# noinspection PyClassHasNoInit
@dataclass
class FOV:
    """Camera field of view related attributes"""
    fov_pix: np.ndarray
    fov: Optional[GeoTrapezoid]
    c: GeoPoint
    c_pix: np.ndarray
    scaling: float = field(init=False)

    # TODO: how to estimate if fov_wgs84 is zero (cannot be projected because camera pitch too high)?
    def _estimate_altitude_scaling(self) -> float:
        """Estimates altitude scaling factor from field of view matched against known map

        Altitude in t is in rotated and cropped map raster pixel coordinates. We can use fov_pix and fov_wgs84 to
        find out the right scale in meters. Distance in pixels is computed from lower left and lower right corners
        of the field of view (bottom of fov assumed more stable than top), while distance in meters is computed from
        the corresponding WGS84 latitude and latitude coordinates.

        :return: Altitude scaling factor
        :raise: DataValueError if FOV is not a valid (convex isoscalar) trapezoid
        """
        try:
            distance_in_pixels = GeoTrapezoid(self.fov_pix, crs='').length
        except GeoValueError as _:
            raise DataValueError('Could not create a valid FOV.')
        distance_in_meters = self.fov.meter_length

        # TODO: this is vulnerable to the top of the FOV 'escaping' into the horizon, should just use bottom of FOV
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

    Collects field of view and map_match under a single structure that is intended to be stored in input data context as
    visual odometry fix reference. Includes the needed map_match and pix_to_wgs84 transformation for the vo fix.
    """
    image_pair: ImagePair
    pose: Pose
    timestamp: int
    ground_elevation: Optional[float]
    fov: FOV = field(init=False)
    position: Position = field(init=False)
    h: np.ndarray = field(init=False)
    inv_h: np.ndarray = field(init=False)
    camera_position: np.ndarray = field(init=False)

    def _estimate_fov(self) -> Optional[FOV]:
        """Estimates field of view and principal point in both pixel and WGS84 coordinates

        :return: Field of view and principal point in pixel and WGS84 coordinates, or None if could not estimate
        """
        assert_type(self.image_pair.ref, ContextualMapData)  # Need pix_to_wgs84, FixedCamera should have map data match
        pix_to_wgs84_2d = self.image_pair.ref.pix_to_wgs84
        pix_to_wgs84_2d[2][2] = 1
        h_wgs84 = pix_to_wgs84_2d @ self.inv_h
        fov_pix, c_pix = self._get_fov_and_c(self.image_pair.qry.image.dim, self.inv_h)
        fov_wgs84, c_wgs84 = self._get_fov_and_c(self.image_pair.ref.image.dim, h_wgs84)
        try:
            fov = FOV(fov_pix=fov_pix,
                      fov=GeoTrapezoid(np.flip(fov_wgs84, axis=2), crs='epsg:4326'),
                      c_pix=c_pix,
                      c=GeoPoint(*c_wgs84.squeeze()[::-1], crs='epsg:4326')
                      )
            return fov
        except GeoValueError as _:
            # Not a valid field of view
            return None
        except DataValueError as _:
            # Could not create a valid FOV
            return None

    def _estimate_attitude(self) -> np.ndarray:
        """Estimates gimbal (not vehicle) attitude in NED frame

        .. note::
            Stabilized gimbal *actual* (not set) attitude relative to vehicle body frame not always known so it is
            currently not computed.

        :return: Gimbal attitude in NED frame
        """
        # Convert estimated rotation to attitude quaternion for publishing
        rT = self.pose.r.T
        assert not np.isnan(rT).any()
        gimbal_estimated_attitude = Rotation.from_matrix(rT)  # rotated map pixel frame
        gimbal_estimated_attitude *= Rotation.from_rotvec(-(np.pi/2) * np.array([1, 0, 0]))  # camera body
        gimbal_estimated_attitude *= Rotation.from_rotvec(
            self.image_pair.ref.rotation * np.array([0, 0, 1]))  # unrotated map pixel frame

        # Re-arrange axes from unrotated (original) map pixel frame to NED frame
        rotvec = gimbal_estimated_attitude.as_rotvec()
        gimbal_estimated_attitude = Rotation.from_rotvec([-rotvec[1], rotvec[0], rotvec[2]])

        return Attitude(gimbal_estimated_attitude.as_quat())

    @staticmethod
    def _get_fov_and_c(img_arr_shape: Tuple[int, int], h_mat: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Calculates field of view (FOV) corners from homography and image shape.

        :param img_arr_shape: Image array shape tuple (height, width)
        :param h_mat: Homography matrix
        :return: Tuple of FOV corner coordinates and prinicpal point np.ndarrays
        """
        assert_type(img_arr_shape, tuple)
        assert_len(img_arr_shape, 2)
        assert_type(h_mat, np.ndarray)
        h, w = img_arr_shape  # height before width in np.array shape
        src_fov = create_src_corners(h, w)

        principal_point_src = np.array([[[w / 2, h / 2]]])
        src_fov_and_c = np.vstack((src_fov, principal_point_src))

        assert_shape(h_mat, (3, 3))
        assert_ndim(src_fov, 3)  # TODO: this is currently not assumed to be squeezed
        dst_fov_and_c = cv2.perspectiveTransform(src_fov_and_c, h_mat)

        dst_fov, principal_point_dst = np.vsplit(dst_fov_and_c, [-1])

        assert_shape(dst_fov, src_fov.shape)
        assert_shape(principal_point_dst, principal_point_src.shape)

        return dst_fov, principal_point_dst

    def _estimate_position(self, ground_elevation: Optional[float], crs: str = 'epsg:4326') -> Optional[Position]:
        """Estimates camera position (WGS84 coordinates + altitude in meters above mean sea level (AMSL)) as well as
        terrain altitude in meters.

        :param ground_elevation: Optional ground elevation (needed to estimate altitude from sea level)
        :param crs: CRS to use for the Position
        :return: Camera position
        """
        assert self.fov is not None  # Call _estimate_fov before _estimate_position!
        # Translation in WGS84 (and altitude or z-axis translation in meters above ground)
        assert_type(self.image_pair.ref, ContextualMapData)  # need pix_to_wgs84
        t_wgs84 = self.image_pair.ref.pix_to_wgs84 @ np.append(self.camera_position[0:2], 1)
        #t_wgs84[2] = -self.image_pair.ref.pix_to_wgs84[2][2] * self.camera_position[2]  # In NED frame z-coordinate is negative above ground, make altitude >0
        t_wgs84[2] = -self.fov.scaling * self.camera_position[2]  # In NED frame z-coordinate is negative above ground, make altitude >0

        # Check that we have all the values needed for a global position
        if not all([(isinstance(x, float) or np.isnan(x)) for x in t_wgs84.squeeze()]):
            # Not a valid position estimate
            return None

        lon, lat = t_wgs84.squeeze()[1::-1]
        alt = t_wgs84[2]
        position = Position(
            xy=GeoPoint(lon, lat, crs),  # lon-lat order
            z_ground=alt,
            z_amsl=alt + ground_elevation if ground_elevation is not None else None,
            x_sd=None,
            y_sd=None,
            z_sd=None,
            attitude=self._estimate_attitude(),
            timestamp=self.image_pair.qry.timestamp
        )

        return position

    def __post_init__(self):
        """Post-initialization computed fields and validity checks

        :raise: DataValueError if a valid FixedCamera could not be initialized
        """
        img = self.image_pair.qry
        object.__setattr__(self, 'h', img.camera_data.k @ np.delete(self.pose.e, 2, 1))  # Remove z-column, making the matrix square
        try:
            object.__setattr__(self, 'inv_h', np.linalg.inv(self.h))
        except np.linalg.LinAlgError as _:
            raise DataValueError('H was not invertible')
        object.__setattr__(self, 'camera_position', -self.pose.r.T @ self.pose.t)

        fov = self._estimate_fov()  # Raises DataValueError if can't estimate valid FOV
        if fov is not None:
            object.__setattr__(self, 'fov', fov)  # Need to do before calling self._estimate_position
        else:
            raise DataValueError('Could not initialize a valid FixedCamera.')

        try:
            position = self._estimate_position(self.ground_elevation)
            if position is not None:
                object.__setattr__(self, 'position', position)
            else:
                raise DataValueError('Could not initialize a valid FixedCamera.')
        except DataValueError as _:
            # This comes from Position.__post_init__
            raise

        camera_data = img.camera_data
        reference = np.array([camera_data.cx, camera_data.cy, camera_data.fx])
        # TODO: The 3 and 6 are an arbitrary thresholds, make configurable?
        if (np.abs(self.pose.t).squeeze() >= 3 * reference).any() or \
                (np.abs(self.pose.t).squeeze() >= 6 * reference).any():
            raise DataValueError(f'pose.t {self.pose.t} & pose.t {self.pose.t} have values too large compared to ' \
                                 f'(cx, cy, fx): {reference}.')


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


class DataValueError(ValueError):
    """Exception returned if a valid dataclass could not be initialized from provided values."""
    pass


def create_src_corners(h: int, w: int) -> np.ndarray:
    """Helper function that returns image corner pixel coordinates in a numpy array.

    :param h: Source image height
    :param w: Source image width
    :return: Source image corner pixel coordinates
    """
    assert_type(h, int)
    assert_type(w, int)
    assert h > 0 and w > 0, f'Height {h} and width {w} are both expected to be positive.'
    return np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)

