"""Immutable dataclasses to protect atomicity of related information

The classes here may return a :class:`.DataValueError` upon instantiation to
prevent invalid data structures from being instantiated. Example usage that
handles the exception:

.. code-block:: python

    try:
        position = Position(
            ...
        )
        return position
    except DataValueError as dve:
        self.get_logger().error('Error determining vehicle position.')
        return None
"""
from __future__ import annotations  # Python version 3.7+

import os
import warnings
from collections import namedtuple
from dataclasses import dataclass, field
from typing import Optional, Union
from xml.etree import ElementTree

import numpy as np
from scipy.spatial.transform import Rotation

from ._assertions import assert_len, assert_type

warnings.filterwarnings(
    action="ignore", category=UserWarning, message="Gimbal lock detected."
)

BBox = namedtuple("BBox", "left bottom right top")


# noinspection PyClassHasNoInit
@dataclass(frozen=True)
class Attitude:
    """Attitude (orientation) in 3D space, typically in FRD or NED frame
    depending on context"""

    # (x, y, z, w) SciPy format! :class:`px4_msgs.VehicleAttitude` q is (w, x, y, z)
    q: np.ndarray
    roll: float = field(init=False)
    pitch: float = field(init=False)
    yaw: float = field(init=False)
    r: float = field(init=False)
    extrinsic: bool = False

    def __post_init__(self):
        """Post-initialization validity checks"""
        assert_len(self.q, 4)
        rotation = Rotation.from_quat(self.q)
        roll, pitch, yaw = tuple(rotation.as_euler("xyz" if self.extrinsic else "XYZ"))
        object.__setattr__(self, "roll", roll)
        object.__setattr__(self, "pitch", pitch)
        object.__setattr__(self, "yaw", yaw)
        object.__setattr__(self, "r", rotation.as_matrix())

    def to_esd(self) -> Attitude:
        """Converts attitude from NED to solvePnP ESD world frame

        :return: Attitude in SED frame
        """
        nadir_pitch = np.array(
            [0, np.sin(np.pi / 4), 0, np.sin(np.pi / 4)]
        )  # Adjust origin to nadir facing camera
        r = Rotation.from_quat(self.q) * Rotation.from_quat(nadir_pitch)
        q = r.as_quat()
        q = np.array([q[1], -q[0], q[2], -q[3]])  # NED to ESD
        att = Attitude(q, self.extrinsic)
        return att

    def as_rotation(self) -> Rotation:
        """Attitude aa :class:`scipy.spatial.transform.Rotation` instance"""
        return Rotation.from_quat(self.q)


def create_src_corners(h: int, w: int) -> np.ndarray:
    """Helper function that returns image corner pixel coordinates in a numpy array.

    Returns: top-left, bottom-left, bottom-right, top-right

    :param h: Source image height
    :param w: Source image width
    :return: Source image corner pixel coordinates
    """
    assert_type(h, int)
    assert_type(w, int)
    assert (
        h > 0 and w > 0
    ), f"Height {h} and width {w} are both expected to be positive."
    return np.float32([[0, 0], [h - 1, 0], [h - 1, w - 1], [0, w - 1]]).reshape(
        -1, 1, 2
    )
