"""Helper functions for transforming between image pixel, WGS84 and other coordinates"""
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

