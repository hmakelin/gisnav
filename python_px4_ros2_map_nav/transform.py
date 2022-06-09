"""Helper functions for transforming between image pixel, WGS84 and other coordinates"""
from typing import Tuple, Optional
from shapely.geometry import box, Polygon

import cv2
import numpy as np

from python_px4_ros2_map_nav.assertions import assert_type, assert_shape, assert_len, assert_ndim


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

