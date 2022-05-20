"""Helper functions for transforming between image pixel, WGS84 and other coordinates"""
import math
from typing import Tuple, Optional
from shapely.geometry import box, Polygon

import cv2
import numpy as np

from python_px4_ros2_map_nav.assertions import assert_type, assert_shape, assert_len, assert_ndim

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


def relative_area_of_intersection(fov1: np.ndarray, fov2: np.ndarray) -> float:
    """Returns relative area of intersection of two polygons (fields of view)"""
    polygon_shape_1 = Polygon(fov1.squeeze())
    polygon_shape_2 = Polygon(fov2.squeeze())
    intersection_area_1 = polygon_shape_1.intersection(polygon_shape_2).area
    intersection_area_2 = polygon_shape_2.intersection(polygon_shape_1).area
    intersection_area = min(intersection_area_1, intersection_area_2)  # If other fov is fully contained by the other
    try:
        ratio = intersection_area/polygon_shape_2.area
    except Exception as e:
        print(f'{e} {fov1} {fov2}')
    #print(f'intersection area ratio {ratio} {polygon_shape_1} {polygon_shape_2}')
    return ratio


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


def make_keypoint(pt: np.ndarray, sz: float = 1.0) -> cv2.KeyPoint:
    """Converts input numpy array to a cv2.KeyPoint.

    :param pt: Keypoint x and y coordinates
    :param sz: Keypoint size
    :return:
    """
    assert_type(pt, np.ndarray)
    assert_shape(pt, (2,))
    return cv2.KeyPoint(pt[0], pt[1], sz)


def is_convex_isosceles_trapezoid(fov_pix: np.ndarray, diagonal_length_tolerance: float = 0.1) -> bool:
    """Returns True if provided quadrilateral is a convex isosceles trapezoid

    If the estimated field of view (FOV) is not a convex isosceles trapezoid, it is a sign that (1) the match was bad or
    (2) the gimbal the camera is mounted on has not had enough time to stabilize (camera has non-zero roll). Matches
    where the FOV is not a convex isosceles trapezoid should be rejected assuming we can't determine (1) from (2) and
    that it is better to wait for a good position estimate than to use a bad one.

    See also :func:`~create_src_corners` for the assumed order of the quadrilateral corners.

    :param fov_pix: Corners of the quadrilateral (upper left, lower left, lower right, upper right) in pixel coordinates
    :param diagonal_length_tolerance: Tolerance for relative length difference between trapezoid diagonals
    :return: True if the quadrilateral is a convex isosceles trapezoid
    """
    assert_len(fov_pix, 4)
    ul, ll, lr, ur = tuple(map(lambda pt: pt.squeeze().tolist(), fov_pix))

    # Check convexity (exclude self-crossing trapezoids)
    # Note: inverted y-axis, origin at upper left corner of image
    if not(ul[0] < ur[0] and ul[1] < ll[1] and lr[0] > ll[0] and lr[1] > ur[1]):
        return False

    # Check diagonals same length within tolerance
    ul_lr_length = math.sqrt((ul[0] - lr[0])**2 + (ul[1] - lr[1])**2)
    ll_ur_length = math.sqrt((ll[0] - ur[0])**2 + (ll[1] - ur[1])**2)
    if abs((ul_lr_length/ll_ur_length)-1) > diagonal_length_tolerance:
        return False

    return True
