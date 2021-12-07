import cv2
import numpy as np
import sys
import os
import math
import geojson

from typing import Any, Union, Tuple, get_args
from functools import partial
from collections import namedtuple, Sequence, Collection

from builtin_interfaces.msg._time import Time  # Need this for type checking in ImageFrame  # TODO: get rid of this

BBox = namedtuple('BBox', 'left bottom right top')  # Convention: https://wiki.openstreetmap.org/wiki/Bounding_Box
LatLon = namedtuple('LatLon', 'lat lon')
LatLonAlt = namedtuple('LatLonAlt', 'lat lon alt')
Dim = namedtuple('Dim', 'height width')  # TODO: Rename this HeightWidth so that order between Height and widht does not accidentally get mixed up?
RPY = namedtuple('RPY', 'roll pitch yaw')


def assert_type(type_: Any, value: object) -> None:
    """Asserts that inputs are of same type."""
    assert isinstance(value, type_), f'Type {type(value)} provided when {type_} was expected.'


def assert_ndim(value: np.ndarray, ndim: int) -> None:
    """Asserts a specific number of dimensions for a numpy array."""
    assert value.ndim == ndim, f'Unexpected number of dimensions: {value.ndim} ({ndim} expected).'


def assert_len(value: Union[Sequence, Collection], len_: int):
    """Asserts a specific length for a sequence or a collection (e.g. a list)."""
    assert len(value) == len_, f'Unexpected length: {len(value)} ({len_} expected).'


def assert_shape(value: np.ndarray, shape: tuple):
    """Asserts a specific shape for np.ndarray."""
    assert value.shape == shape, f'Unexpected shape: {value.shape} ({shape} expected).'


def assert_first_stamp_greater(stamp1: Time, stamp2: Time) -> None:
    """Asserts that the first stamp is higher (later in time) than the second stamp."""
    assertion_error_msg = f'stamp2 {stamp2} was >= than current image frame timestamp {stamp1}.'
    if stamp1.sec == stamp2.sec:
        assert stamp1.nanosec > stamp2.nanosec, assertion_error_msg
    else:
        assert stamp1.sec > stamp2.sec, assertion_error_msg


class ImageFrame(object):
    """Keeps image frame related data in one place and protects it from corruption."""

    def __init__(self, image: np.ndarray, frame_id: str, stamp: Time) -> None:
        assert_type(np.ndarray, image)
        assert_ndim(image, 3)
        assert_type(str, frame_id)
        assert_type(Time, stamp)
        self._image = image
        self._frame_id = frame_id
        self._stamp = stamp

        # fov and position expected to be set only once and some time after instantiation, setter methods provided
        self._fov = None
        self._position = None

    @property
    def image(self) -> np.ndarray:
        return self._image

    @property
    def frame_id(self) -> str:
        return self._frame_id

    @property
    def stamp(self) -> Time:
        return self._stamp

    @property
    def fov(self) -> np.ndarray:
        return self._fov

    @fov.setter
    def fov(self, value: np.ndarray) -> None:
        if self._fov is not None:
            raise AttributeError("Modification of property not allowed.")
        assert_type(np.ndarray, value)
        assert_shape(value, (4, 1, 2))
        self._fov = value

    @property
    def position(self) -> LatLonAlt:
        return self._position

    @position.setter
    def position(self, value: LatLonAlt) -> None:
        if self._position is not None:
            raise AttributeError("Modification of property not allowed.")
        assert_type(LatLonAlt, value)
        self._position = value


class MapFrame(object):
    """Keeps map frame related data in one place and protects it from corruption."""

    def __init__(self, center: LatLon, radius: Union[int, float], bbox: BBox, image: np.ndarray) -> None:
        assert_type(BBox, bbox)
        assert_type(LatLon, center)
        assert_type(get_args(Union[int, float]), radius)
        assert_type(np.ndarray, image)
        assert_ndim(image, 3)
        self._bbox = bbox
        self._center = center
        self._radius = radius
        self._image = image

    @property
    def center(self) -> LatLon:
        return self._center

    @property
    def radius(self) -> int:
        return self._radius

    @property
    def bbox(self) -> BBox:
        return self._bbox

    @property
    def image(self) -> np.ndarray:
        return self._image


def fov_to_bbox(fov_wgs84: np.ndarray) -> BBox:
    """Returns BBox for WGS84 field of view."""
    assert_type(np.ndarray, fov_wgs84)
    assert_shape(fov_wgs84, (4, 2))
    left, bottom, right, top = 180, 90, -180, -90
    for pt in fov_wgs84:
        right = pt[1] if pt[1] >= right else right
        left = pt[1] if pt[1] <= left else left
        top = pt[0] if pt[0] >= top else top
        bottom = pt[0] if pt[0] <= bottom else bottom
    return BBox(left, bottom, right, top)


def _make_keypoint(pt: np.ndarray, sz: float = 1.0) -> cv2.KeyPoint:
    """Converts input numpy array to a cv2.KeyPoint."""
    assert_type(np.ndarray, pt)
    assert_shape(pt, (2,))
    return cv2.KeyPoint(pt[0], pt[1], sz)


def visualize_homography(figure_name: str, img_arr: np.ndarray, map_arr: np.ndarray, kp_img: np.ndarray,
                         kp_map: np.ndarray, dst_corners: np.ndarray) -> np.ndarray:
    """Visualizes a homography including keypoint matches and field of view."""
    # Make a list of cv2.DMatches that match mkp_img and mkp_map one-to-one
    kp_count = len(kp_img)
    assert kp_count == len(kp_map), 'Keypoint counts for img and map did not match.'
    matches = list(map(lambda i: cv2.DMatch(i, i, 0), range(0, kp_count)))

    # Need cv2.KeyPoints for keypoints
    kp_img = np.apply_along_axis(_make_keypoint, 1, kp_img)
    kp_map = np.apply_along_axis(_make_keypoint, 1, kp_map)

    map_with_fov = cv2.polylines(map_arr, [np.int32(dst_corners)], True, 255, 3, cv2.LINE_AA)
    draw_params = dict(matchColor=(0, 255, 0), singlePointColor=None, matchesMask=None, flags=2)
    out = cv2.drawMatches(img_arr, kp_img, map_with_fov, kp_map, matches, None, **draw_params)
    cv2.imshow(figure_name, out)
    cv2.waitKey(1)

    return out


def get_fov(img_arr: np.ndarray, h_mat: np.ndarray) -> np.ndarray:
    """Calculates field of view (FoV) corners from homography and image shape."""
    assert_type(np.ndarray, img_arr)
    assert_type(np.ndarray, h_mat)
    h, w, _ = img_arr.shape  # height before width in np.array shape
    src_corners = create_src_corners(h, w)
    assert_shape(h_mat, (3, 3))
    assert_ndim(src_corners, 3)  # TODO: this is currently not assumed to be squeezed
    dst_corners = cv2.perspectiveTransform(src_corners, h_mat)
    assert_shape(src_corners, dst_corners.shape)
    return dst_corners


def create_src_corners(h: int, w: int) -> np.ndarray:
    """Returns image corner pixel coordinates in a numpy array."""
    assert_type(int, h)
    assert_type(int, w)
    assert h > 0 and w > 0, f'Height {h} and width {w} are both expected to be positive.'
    return np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)


def setup_sys_path() -> Tuple[str, str]:
    """Adds the package share directory to the path so that SuperGlue can be imported."""
    if 'get_package_share_directory' not in sys.modules:
        from ament_index_python.packages import get_package_share_directory
    package_name = 'wms_map_matching'  # TODO: try to read from somewhere (e.g. package.xml)
    share_dir = get_package_share_directory(package_name)
    superglue_dir = os.path.join(share_dir, 'SuperGluePretrainedNetwork')  # todo: set this stuff up in the superglue adapter module
    sys.path.append(os.path.abspath(superglue_dir))
    return share_dir, superglue_dir


def convert_fov_from_pix_to_wgs84(fov_in_pix, map_raster_padded_dim, map_raster_bbox, map_raster_rotation, img_dim,
                                  uncrop=True):
    """Converts the field of view from pixel coordinates to WGS 84.

    Arguments:
        fov_in_pix - Numpy array of field of view corners in pixel coordinates of rotated map raster.
        map_raster_padded_dim - Size of the padded map raster image.
        map_raster_bbox - The WGS84 bounding box of the padded map raster.
        map_raster_rotation - The rotation that was applied to the map raster before matching in radians.
        img_dim - Size of the image.
        uncrop - Flag to determine whether uncropping will be included.
    """
    if uncrop:
        uncrop = partial(uncrop_pixel_coordinates, img_dim, map_raster_padded_dim)
        fov_in_pix_uncropped = np.apply_along_axis(uncrop, 2, fov_in_pix)
    else:
        fov_in_pix_uncropped = fov_in_pix

    rotate = partial(rotate_point, map_raster_rotation, map_raster_padded_dim)   # why not negative here?
    fov_in_pix_unrotated = np.apply_along_axis(rotate, 2, fov_in_pix_uncropped)

    convert = partial(convert_pix_to_wgs84, map_raster_padded_dim, map_raster_bbox)
    fov_in_wgs84 = np.apply_along_axis(convert, 2, fov_in_pix_unrotated)

    return fov_in_wgs84, fov_in_pix_uncropped, fov_in_pix_unrotated  # TODO: only return wgs84


def rotate_point(radians: float, img_dim: Dim, pt: np.ndarray) -> Tuple[float, float]:
    """Rotates point around center of image by radians, counter-clockwise."""
    cx = img_dim[0] / 2
    cy = img_dim[1] / 2  # Should be same as cx (assuming image or map raster is square)
    cos_rads = math.cos(radians)
    sin_rads = math.sin(radians)
    x = cx + cos_rads * (pt[0] - cx) - sin_rads * (pt[1] - cy)
    y = cy + sin_rads * (pt[0] - cx) + cos_rads * (pt[1] - cy)
    return x, y


def convert_pix_to_wgs84(img_dim: Dim, bbox: BBox, pt: np.ndarray) -> Tuple[float, float]:  # TODO: change return type to LatLon?
    """Converts a pixel inside an image to lat lon coordinates based on the image's bounding box.

    In cv2, y is 0 at top and increases downwards. x axis is 'normal' with x=0 at left."""
    assert_type(Dim, img_dim)
    assert_type(BBox, bbox)
    assert_type(np.ndarray, pt)
    assert_shape(pt, (2,))
    lat = bbox.bottom + (bbox.top - bbox.bottom) * (img_dim.height - pt[1]) / img_dim.height  # TODO: use the 'LatLon' named tuple for pt
    lon = bbox.left + (bbox.right - bbox.left) * pt[0] / img_dim.width
    return lat, lon


def get_camera_apparent_altitude(map_radius, map_dimensions, K):
    """Returns camera apparent altitude using the K of the drone's camera and the map's known ground truth size.

    Assumes same K for the hypothetical camera that was used to take a picture of the (ortho-rectified) map raster.

    Arguments:
        map_radius - The radius in meters of the map raster (the raster should be a square enclosing a circle of radius)
        map_dimensions - The image dimensions in pixels of the map raster.
        K - Camera intrinsic matrix (assume same K for map raster as for drone camera).
    """
    focal_length = K[0]
    width_pixels = map_dimensions.width  # [0]
    return map_radius * focal_length / width_pixels


# TODO: use this to replace "get_camera_apparent_altitude"
def get_camera_distance(focal_length, pixels, ground_truth):
    """Calculates distance of camera to object whose ground truth widht is known."""
    return ground_truth * focal_length / pixels


def altitude_from_gimbal_pitch(pitch_degrees, distance):
    """Returns camera altitude if gimbal pitch and distance to center of FoV is known."""
    return distance*math.sin(math.radians(pitch_degrees))


def get_bbox_center(bbox):
    """Returns camera lat-lon location assuming it is in the middle of given bbox (nadir facing camera)."""
    return LatLon(bbox.bottom + (bbox.top - bbox.bottom) / 2, bbox.left + (bbox.right - bbox.left) / 2)


def rotate_and_crop_map(map: np.ndarray, radians: float, dimensions: Dim, visualize: bool = False) -> np.ndarray:
    # TODO: only tested on width>height images.
    """Rotates map counter-clockwise and then crops a dimensions-sized part from the middle.

    Map needs padding so that a circle with diameter of the diagonal of the img_size rectangle is enclosed in map."""
    cx, cy = tuple(np.array(map.shape[0:2]) / 2)
    degrees = math.degrees(radians)
    r = cv2.getRotationMatrix2D((cx, cy), degrees, 1.0)
    map_rotated = cv2.warpAffine(map, r, map.shape[1::-1])
    map_cropped = crop_center(map_rotated, dimensions)
    if visualize:
        cv2.imshow('padded', map)
        cv2.waitKey(1)
        cv2.imshow('rotated', map_rotated)
        cv2.waitKey(1)
        cv2.imshow('cropped', map_cropped)
        cv2.waitKey(1)
    assert map_cropped.shape[0:2] == dimensions, f'Cropped shape {map_cropped.shape} did not match dims {dimensions}.'
    return map_cropped


def crop_center(img, dimensions):
    # TODO: only tested on width>height images.
    """Crops dimensions sized part from center."""
    cx, cy = tuple(np.array(img.shape[0:2]) / 2)  # TODO: could be passed from rotate_and_crop_map instead of computing again
    img_cropped = img[math.floor(cy - dimensions.height / 2):math.floor(cy + dimensions.height / 2),
                      math.floor(cx - dimensions.width / 2):math.floor(cx + dimensions.width / 2)]   # TODO: use floor or smth else?
    assert (img_cropped.shape[0:2] == dimensions.height, dimensions.width), 'Something went wrong when cropping the ' \
                                                                            'map raster. '
    return img_cropped


def uncrop_pixel_coordinates(cropped_dimensions: Dim, dimensions: Dim, pt: np.ndarray) -> np.ndarray:
    """Adjusts the pt x and y coordinates for the original size provided by dimensions."""
    assert_type(np.ndarray, pt)
    assert_shape(pt, (2,))
    assert_type(Dim, cropped_dimensions)
    assert_type(Dim, dimensions)
    pt_out = pt + 0.5*np.array([dimensions.width-cropped_dimensions.width, dimensions.height-cropped_dimensions.height])
    assert_shape(pt_out, pt.shape)
    return pt_out


def get_angle(vec1: np.ndarray, vec2: np.ndarray, normalize=False):
    """Returns angle in radians between two vectors."""
    # TODO: assert that inputs are vectors
    if normalize:
        vec1 = vec1 / np.linalg.norm(vec1)
        vec2 = vec2 / np.linalg.norm(vec2)
    dot_product = np.dot(vec1, vec2)
    angle = np.arccos(dot_product)
    return angle
