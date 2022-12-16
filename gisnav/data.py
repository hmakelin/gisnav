"""Module containing immutable dataclasses to protect atomicity of related information (object-based 'contexts')

The classes here may return a :class:`.DataValueError` upon instantiation to prevent invalid data structures from
being instantiated. Example usage that handles the exception:

.. code-block:: python

    try:
        position = Position(
            ...
        )
        return position
    except DataValueError as dve:
        self.get_logger().warn(f'Error determining vehicle position:\n{dve},\n{traceback.print_exc()}.')
        return None
"""
from __future__ import annotations  # Python version 3.7+

import cv2
import numpy as np
import os
import math
import warnings
warnings.filterwarnings(action='ignore', category=UserWarning, message='Gimbal lock detected.')

from xml.etree import ElementTree
from typing import Optional, Tuple
from dataclasses import dataclass, field
from collections import namedtuple
from scipy.spatial.transform import Rotation
from shapely.geometry import box
from geographic_msgs.msg import GeoPoint

from gisnav.assertions import assert_type, assert_ndim, assert_shape, assert_len
from gisnav.geo import GeoPt, GeoTrapezoid, GeoValueError

Dim = namedtuple('Dim', 'height width')
TimePair = namedtuple('TimePair', 'local foreign')
BBox = namedtuple('BBox', 'left bottom right top')


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class Position:
    """Represents a 3D position with geographical xy-coordinates and an altitude expressed in meters

    .. note::
        (x, y, z) coordinates are in ENU frame
    """
    xy: GeoPt                       # XY coordinates (e.g. longitude & latitude in WGS84)
    altitude: Altitude
    attitude: Optional[Attitude]    # attitude in NED frame
    timestamp: Optional[int]        # Reference timestamp of position

    _KALMAN_FILTER_EPSG_CODE = 'epsg:3857'
    """Used for converting into an array that can be passed to :class:`.SimpleFilter"""

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
    extrinsic: bool = False

    def __post_init__(self):
        """Post-initialization validity checks"""
        assert_len(self.q, 4)
        rotation = Rotation.from_quat(self.q)
        roll, pitch, yaw = tuple(rotation.as_euler('xyz' if self.extrinsic else 'XYZ'))
        object.__setattr__(self, 'roll', roll)
        object.__setattr__(self, 'pitch', pitch)
        object.__setattr__(self, 'yaw', yaw)
        object.__setattr__(self, 'r', rotation.as_matrix())

    def to_esd(self) -> Attitude:
        """Converts attitude from NED to solvePnP ESD world frame

        :return: Attitude in SED frame
        """
        nadir_pitch = np.array([0, np.sin(np.pi / 4), 0, np.sin(np.pi / 4)])  # Adjust origin to nadir facing camera
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
    bbox: BBox
    elevation: Optional[Img] = None  # Optional elevation raster

    def __post_init__(self):
        """Post-initialization validity check."""
        if self.elevation is not None:
            assert self.elevation.arr.shape[0:2], self.image.arr.shape[0:2]
            assert_ndim(self.elevation.arr, 2)  # Grayscale image expected


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class ContextualMapData(_ImageHolder):
    """Contains the rotated and cropped map image pose estimation"""
    image: Img = field(init=False)              # Cropped and rotated map which is the same size as the camera frames
    elevation: Optional[Img] = field(init=None) # Rotated elevation raster (optional) in meters
    rotation: float                             # radians
    crop: Dim                                   # Same value will also be found at image.dim (but not at initialization)
    map_data: MapData                           # This is the original (square) map with padding
    pix_to_wgs84: np.ndarray = field(init=False)
    mock_data: bool = False                     # Indicates that this was used for field of view guess (mock map data)
    altitude_scaling: Optional[float] = None    # altitude scaling (elevation raster meters -> camera pixels)

    # TODO: update docs - only one transformation is returned
    def _pix_to_wgs84(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Returns tuple of affine 2D transformation matrix for converting matched pixel coordinates to WGS84
        coordinates along with intermediate transformations

        These transformations can be used to reverse the rotation and cropping that :func:`._rotate_and_crop_map` did to
        the original map.

        :return: Tuple containing 2D affine transformations from 1. pixel coordinates to WGS84, 2. from original
            unrotated and uncropped map pixel coordinates to WGS84, 3. from rotated map coordinates to unrotated map
            coordinates, and 4. from cropped map coordinates to uncropped (but still rotated) map pixel coordinates.
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
        coords = np.array(box(*self.map_data.bbox).exterior.coords)
        gt = GeoTrapezoid(coords)  # .reshape(-1, 1, 2)
        dst_corners = gt.square_coords
        dst_corners = np.flip(dst_corners, axis=1)  # from ENU frame to WGS 84 axis order
        unrotated_to_wgs84 = cv2.getPerspectiveTransform(np.float32(src_corners).squeeze(),
                                                         np.float32(dst_corners).squeeze())

        # Ratio of boundaries in pixels and meters -> Altitude (z) scaling factor
        vertical_scaling = abs(gt.meter_length / \
                               (2 * self.map_data.image.dim.width + 2 * self.map_data.image.dim.height))

        pix_to_wgs84_ = unrotated_to_wgs84 @ uncropped_to_unrotated @ pix_to_uncropped

        pix_to_wgs84_[2][2] = -vertical_scaling * pix_to_wgs84_[2][2]

        return pix_to_wgs84_  # , unrotated_to_wgs84, uncropped_to_unrotated, pix_to_uncropped

    def _rotate_and_crop_map(self, elevation: bool = False) -> np.ndarray:
        """Rotates map counter-clockwise and then crops a dimensions-sized part from the middle

        :param elevation: Set True to do rotation on elevation raster instead
        :return: Rotated and cropped map raster
        """
        image = self.map_data.image if not elevation else self.map_data.elevation
        cx, cy = tuple(np.array(image.arr.shape[0:2]) / 2)
        degrees = math.degrees(self.rotation)
        r = cv2.getRotationMatrix2D((cx, cy), degrees, 1.0)
        map_rotated = cv2.warpAffine(image.arr, r, image.arr.shape[1::-1])
        map_cropped = self._crop_center(map_rotated, self.crop)
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
        """Crops dimensions sized part from center

        :param img: Image to crop
        :param dimensions: Dimensions of area to crop (not of image itself)
        :return: Cropped image
        """
        cx, cy = tuple(np.array(img.shape[0:2]) / 2)
        img_cropped = img[math.floor(cy - dimensions.height / 2):math.floor(cy + dimensions.height / 2),
                      math.floor(cx - dimensions.width / 2):math.floor(cx + dimensions.width / 2)]
        assert (img_cropped.shape[0:2] == dimensions.height, dimensions.width), \
            'Something went wrong when cropping the map raster.'
        return img_cropped

    def __post_init__(self):
        """Set computed fields after initialization."""
        object.__setattr__(self, 'image', Img(self._rotate_and_crop_map()))
        if self.map_data.elevation is not None:
            object.__setattr__(self, 'elevation', Img(self._rotate_and_crop_map(True)))
        else:
            object.__setattr__(self, 'elevation', None)
        object.__setattr__(self, 'pix_to_wgs84', self._pix_to_wgs84())


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class ImagePair:
    """Atomic image pair to represent a matched pair of images"""
    qry: ImageData
    ref: ContextualMapData


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
class Altitude:
    """Holds different definitions of altitude in one place

    amsl: Above Mean Sea Level (AMSL)
    agl: Above Ground Level (AGL)
    ellipsoid: Above WGS84 ellipsoid
    home: Above home or starting location

    .. note::
        TODO: Altitude AGL should always be known (cannot be None)

    .. seealso::
        `Altitude definitions <https://ardupilot.org/copter/docs/common-understanding-altitude.html>`_
    """
    amsl: Optional[float]
    agl: Optional[float]
    ellipsoid: Optional[float]
    home: Optional[float]

    # TODO
    #def __post_init__(self):
    #    """Post-initialization validity checks"""
    #    assert self.agl is not None


# noinspection PyClassHasNoInit
@dataclass
class FOV:
    """Camera field of view related attributes"""
    fov_pix: np.ndarray
    fov: Optional[GeoTrapezoid]
    c: GeoPt
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

        # TODO: this is vulnerable to the top of the FOV 'escaping' into the horizon
        #  should use bottom side of FOV instead of entire perimeter
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

    # TODO: refactor this class out - it was used earlier for visual odometry but is now redundant

    Collects field of view and map_match under a single structure.
    """
    image_pair: ImagePair
    pose: Pose
    timestamp: int
    terrain_altitude_amsl: Optional[float]
    terrain_altitude_ellipsoid: Optional[float]
    home_position: Optional[GeoPoint]
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
                      c=GeoPt(*c_wgs84.squeeze()[::-1], crs='epsg:4326')
                      )
            return fov
        except GeoValueError as _:
            # Not a valid field of view
            return None
        except DataValueError as _:
            # Could not create a valid FOV
            return None

    def _estimate_attitude(self) -> Attitude:
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

        gimbal_estimated_attitude *= Rotation.from_rotvec(
            self.image_pair.ref.rotation * np.array([0, 0, 1]))  # unrotated map pixel frame

        # Re-arrange axes from unrotated (original) map pixel frame to NED frame
        rotvec = gimbal_estimated_attitude.as_rotvec()
        gimbal_estimated_attitude = Rotation.from_rotvec([-rotvec[1], rotvec[0], rotvec[2]])

        return Attitude(gimbal_estimated_attitude.as_quat(), extrinsic=self.image_pair.ref.mock_data)

    @staticmethod
    def _get_fov_and_c(img_arr_shape: Tuple[int, int], h_mat: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Calculates field of view (FOV) corners from homography and image shape.

        :param img_arr_shape: Image array shape tuple (height, width)
        :param h_mat: Homography matrix
        :return: Tuple of FOV corner coordinates and principal point np.ndarrays
        """
        assert_type(img_arr_shape, tuple)
        assert_len(img_arr_shape, 2)
        assert_type(h_mat, np.ndarray)
        h, w = img_arr_shape  # height before width in np.array shape
        src_fov = create_src_corners(h, w)

        principal_point_src = np.array([[[w / 2, h / 2]]])
        src_fov_and_c = np.vstack((src_fov, principal_point_src))

        assert_shape(h_mat, (3, 3))
        assert_ndim(src_fov, 3)
        dst_fov_and_c = cv2.perspectiveTransform(src_fov_and_c, h_mat)

        dst_fov, principal_point_dst = np.vsplit(dst_fov_and_c, [-1])

        assert_shape(dst_fov, src_fov.shape)
        assert_shape(principal_point_dst, principal_point_src.shape)

        return dst_fov, principal_point_dst

    def _estimate_position(self, terrain_altitude_amsl: Optional[float], terrain_altitude_ellipsoid: Optional[float],
                           crs: str = 'epsg:4326') -> Optional[Position]:
        """Estimates camera position (WGS84 coordinates + altitude in meters above mean sea level (AMSL)) as well as
        terrain altitude in meters.

        :param terrain_altitude_amsl: Optional ground elevation above AMSL in meters
        :param terrain_altitude_ellipsoid: Optional ground elevation above WGS 84 ellipsoid in meters
        :param crs: CRS to use for the Position
        :return: Camera position or None if not available
        """
        assert self.fov is not None  # Call _estimate_fov before _estimate_position!
        # Translation in WGS84 (and altitude or z-axis translation in meters above ground)
        assert_type(self.image_pair.ref, ContextualMapData)  # need pix_to_wgs84
        t_wgs84 = self.image_pair.ref.pix_to_wgs84 @ np.append(self.camera_position[0:2], 1)
        t_wgs84[2] = -self.fov.scaling * self.camera_position[2]  # In NED frame z-coordinate is negative above ground, make altitude >0

        # Check that we have all the values needed for a global position
        if not all([(isinstance(x, float) or np.isnan(x)) for x in t_wgs84.squeeze()]):
            # Not a valid position estimate
            return None

        lon, lat = t_wgs84.squeeze()[1::-1]
        alt = t_wgs84[2]

        altitude = Altitude(
            agl=alt,
            amsl=alt + terrain_altitude_amsl if terrain_altitude_amsl is not None else None,
            ellipsoid=alt + terrain_altitude_ellipsoid if terrain_altitude_ellipsoid is not None else None,
            home=None  # TODO
        )
        position = Position(
            xy=GeoPt(lon, lat, crs),  # lon-lat order
            altitude=altitude,
            attitude=self._estimate_attitude(),
            timestamp=self.image_pair.qry.timestamp
        )

        return position

    def __post_init__(self):
        """Post-initialization computed fields and validity checks

        :raise: DataValueError if a valid FixedCamera could not be initialized
        """
        if self.image_pair is None:
            raise DataValueError('Please provide valid image pair.')

        img = self.image_pair.qry
        if self.terrain_altitude_amsl is not None and self.terrain_altitude_ellipsoid is None or \
            self.terrain_altitude_amsl is not None and self.terrain_altitude_ellipsoid is None:
                raise DataValueError('Please provide terrain altitude in both AMSL and above WGS 84 ellipsoid.')

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
            position = self._estimate_position(self.terrain_altitude_amsl,
                                               self.terrain_altitude_ellipsoid)
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

        # Fix home position
        if self.home_position is not None:
            object.__setattr__(self.position.altitude, 'home', self.position.altitude.ellipsoid - self.home_position.altitude)

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

