"""Helper functions for transforming between image pixel, WGS84 and other coordinates"""
import math
from typing import Tuple, Optional

import cv2
import numpy as np

from python_px4_ros2_map_nav.assertions import assert_type, assert_shape, assert_len, assert_ndim
from python_px4_ros2_map_nav.data_classes import BBox, LatLon, Dim, RPY


def fov_center(fov_wgs84: np.ndarray) -> LatLon:
    """Returns Field of View center coordinates (WGS84).

    :param fov_wgs84: FOV corners in WGS84 coordinates
    :return: The LatLon center
    """
    assert_type(fov_wgs84, np.ndarray)
    assert_shape(fov_wgs84, (4, 2))
    left, bottom, right, top = 180, 90, -180, -90
    for pt in fov_wgs84:
        right = pt[1] if pt[1] >= right else right
        left = pt[1] if pt[1] <= left else left
        top = pt[0] if pt[0] >= top else top
        bottom = pt[0] if pt[0] <= bottom else bottom
    return LatLon((top + bottom) / 2, (left + right) / 2)


def get_fov_and_c(img_arr_shape: Tuple[int, int], h_mat: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
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

    principal_point_src = np.array([[[w/2, h/2]]])
    src_fov_and_c = np.vstack((src_fov, principal_point_src))

    assert_shape(h_mat, (3, 3))
    assert_ndim(src_fov, 3)  # TODO: this is currently not assumed to be squeezed
    dst_fov_and_c = cv2.perspectiveTransform(src_fov_and_c, h_mat)

    dst_fov, principal_point_dst = np.vsplit(dst_fov_and_c, [-1])

    assert_shape(dst_fov, src_fov.shape)
    assert_shape(principal_point_dst, principal_point_src.shape)

    return dst_fov, principal_point_dst


def create_src_corners(h: int, w: int) -> np.ndarray:
    """Returns image corner pixel coordinates in a numpy array.

    :param h: Source image height
    :param w: Source image width
    :return: Source image corner pixel coordinates
    """
    assert_type(h, int)
    assert_type(w, int)
    assert h > 0 and w > 0, f'Height {h} and width {w} are both expected to be positive.'
    return np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)


def pix_to_wgs84_affine(map_raster_padded_dim: Dim, map_raster_bbox: BBox, map_raster_rotation: float, img_dim: Dim) \
        -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Returns tuple of affine 2D transformation matrix for converting matched pixel coordinates to WGS84 coordinates
    along with intermediate transformations

    These transformations can be used to reverse the rotation and cropping that :func:`~rotate_and_crop_map` did to
    the original map.

    :param map_raster_padded_dim: Size of the padded map raster image
    :param map_raster_bbox: The WGS84 bounding box of the padded map raster
    :param map_raster_rotation: The rotation that was applied to the map raster before matching in radians
    :param img_dim: Size of the image
    :return: Tuple containing 2D affinre transformations from 1. pixel coordinates to WGS84, 2. from original unrotated
    and uncropped map pixel coordinates to WGS84, 3. from rotated map coordinates to unrotated map coordinates, and 4.
    from cropped map coordinates to uncropped (but still rotated) map pixel coordinates.
    """
    map_dim_arr = np.array(map_raster_padded_dim)
    img_dim_arr = np.array(img_dim)
    crop_padding = map_dim_arr - img_dim_arr
    crop_translation = (crop_padding / 2)
    pix_to_uncropped = np.identity(3)
    # Invert order x<->y in translation vector since height comes first in Dim tuple (inputs should be Dims)
    assert_type(map_raster_padded_dim, Dim)
    assert_type(img_dim, Dim)
    pix_to_uncropped[0:2][:, 2] = crop_translation[::-1]

    rotation_center = map_dim_arr / 2
    rotation = cv2.getRotationMatrix2D(rotation_center, np.degrees(map_raster_rotation), 1.0)
    rotation_padding = np.array([[0, 0, 1]])
    uncropped_to_unrotated = np.vstack((rotation, rotation_padding))

    src_corners = create_src_corners(*map_raster_padded_dim)
    dst_corners = bbox_to_polygon(map_raster_bbox)
    unrotated_to_wgs84 = cv2.getPerspectiveTransform(np.float32(src_corners).squeeze(),
                                                     np.float32(dst_corners).squeeze())

    pix_to_wgs84_ = unrotated_to_wgs84 @ uncropped_to_unrotated @ pix_to_uncropped
    return pix_to_wgs84_, unrotated_to_wgs84, uncropped_to_unrotated, pix_to_uncropped


def bbox_to_polygon(bbox: BBox) -> np.ndarray:
    """Converts BBox to a np.ndarray polygon format

    :param bbox: BBox to convert
    :return: bbox corners in np.ndarray"""
    return np.array([[bbox.top, bbox.left],
                     [bbox.bottom, bbox.left],
                     [bbox.bottom, bbox.right],
                     [bbox.top, bbox.right]]).reshape(-1, 1, 2)


def rotate_and_crop_map(map_: np.ndarray, radians: float, dimensions: Dim, visualize: bool = False) -> np.ndarray:
    """Rotates map counter-clockwise and then crops a dimensions-sized part from the middle.

    Map needs padding so that a circle with diameter of the diagonal of the img_size rectangle is enclosed in map.

    :param map_: Map raster
    :param radians: Rotation in radians
    :param dimensions: Map dimensions
    :param visualize: Flag to indicate whether intermediate rasters should be visualized
    :return: Rotated and cropped map raster
    """
    # TODO: only tested on width>height images.
    cx, cy = tuple(np.array(map_.shape[0:2]) / 2)
    degrees = math.degrees(radians)
    r = cv2.getRotationMatrix2D((cx, cy), degrees, 1.0)
    map_rotated = cv2.warpAffine(map_, r, map_.shape[1::-1])
    map_cropped = crop_center(map_rotated, dimensions)
    if visualize:
        cv2.imshow('padded', map_)
        cv2.waitKey(1)
        cv2.imshow('rotated', map_rotated)
        cv2.waitKey(1)
        cv2.imshow('cropped', map_cropped)
        cv2.waitKey(1)
    assert map_cropped.shape[0:2] == dimensions, f'Cropped shape {map_cropped.shape} did not match dims {dimensions}.'
    return map_cropped


def inv_homography_from_k_and_e(k: np.ndarray, e: np.ndarray) -> Optional[np.ndarray]:
    """Returns inverted homography based on the intrinsic and extrinsic matrices of the camera

    Used to project image pixels to world plane (ground)

    :param k: Camera intrinsics
    :param e: Camera extrinsics
    :return: Homography matrix (assumes world Z coordinate as zero), or None if inversion cannot be done
    """
    e = np.delete(e, 2, 1)  # Remove z-column, making the matrix square
    h = k @ e
    try:
        h_inv = np.linalg.inv(h)
    except np.linalg.LinAlgError as _:
        return None
    return h_inv


def crop_center(img: np.ndarray, dimensions: Dim) -> np.ndarray:
    """Crops dimensions sized part from center.

    :param img: Image to crop
    :param dimensions: Dimensions of area to crop (not of image itself)
    :return: Cropped image
    """
    # TODO: only tested on width>height images.
    cx, cy = tuple(np.array(img.shape[0:2]) / 2)  # TODO: could be passed from rotate_and_crop_map instead of computing again
    img_cropped = img[math.floor(cy - dimensions.height / 2):math.floor(cy + dimensions.height / 2),
                      math.floor(cx - dimensions.width / 2):math.floor(cx + dimensions.width / 2)]   # TODO: use floor or smth else?
    assert (img_cropped.shape[0:2] == dimensions.height, dimensions.width), 'Something went wrong when cropping the ' \
                                                                            'map raster. '
    return img_cropped


def get_azimuth(x: float, y: float) -> float:
    """Get azimuth of position x and y coordinates.

    Note: in NED coordinates x is north, so here it would be y.

    :param x: Meters towards east
    :param y: Meters towards north
    :return: Azimuth in degrees
    """
    rads = math.atan2(y, x)
    rads = rads if rads > 0 else rads + 2*math.pi  # Counter-clockwise from east
    rads = -rads + math.pi/2  # Clockwise from north
    return math.degrees(rads)


def axes_ned_to_image(rpy: RPY, degrees: bool = True) -> RPY:
    """Converts roll-pitch-yaw euler angles from NED to image frame"""
    straight_angle = 180 if degrees else np.pi
    right_angle = 90 if degrees else np.pi / 2
    pitch = -rpy.pitch - right_angle
    if pitch < 0:
        # Gimbal pitch and yaw flip over when abs(gimbal_yaw) should go over 90, adjust accordingly
        pitch += straight_angle
    roll = pitch
    pitch = rpy.roll
    yaw = rpy.yaw
    rpy = RPY(roll, pitch, yaw)
    return rpy


def make_keypoint(pt: np.ndarray, sz: float = 1.0) -> cv2.KeyPoint:
    """Converts input numpy array to a cv2.KeyPoint.

    :param pt: Keypoint x and y coordinates
    :param sz: Keypoint size
    :return:
    """
    assert_type(pt, np.ndarray)
    assert_shape(pt, (2,))
    return cv2.KeyPoint(pt[0], pt[1], sz)
