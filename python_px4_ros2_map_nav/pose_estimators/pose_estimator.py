"""Module that defines an abstract base class for all pose estimators"""
from __future__ import annotations  # Python version 3.7+

import numpy as np

from abc import ABC, abstractmethod
from typing import Optional, Tuple

from python_px4_ros2_map_nav.assertions import assert_type, assert_pose


class PoseEstimator(ABC):
    """Abstract base class for all pose estimators

    This class implements static initializer and worker methods that are intended to be used with
    :class:`multiprocessing.pool.Pool` or :class:`multiprocessing.pool.ThreadPool`. The abstract :meth:`.estimate_pose`
    must be implemented by extending classes.
    """

    POSE_ESTIMATOR_GLOBAL_VAR = '_pose_estimator'
    """A global pose estimator instance is stored under this name"""

    @staticmethod
    def initializer(class_: type, *args) -> None:
        """Creates a global instance of given :class:`.PoseEstimator` child class under
        :py:attr:`.POSE_ESTIMATOR_GLOBAL_VAR`.

        :param class_: The :class:`.PoseEstimator` child class to initialize
        :return:
        """
        if PoseEstimator.POSE_ESTIMATOR_GLOBAL_VAR not in globals():
            globals()[PoseEstimator.POSE_ESTIMATOR_GLOBAL_VAR]: PoseEstimator = class_(*args)

    @staticmethod
    def worker(query: np.ndarray, reference: np.ndarray, k: np.ndarray,
               guess: Optional[Tuple[np.ndarray, np.ndarray]]) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Returns pose between provided images, or None if pose cannot be estimated

        :param query: The first (query) image for pose estimation
        :param reference: The second (reference) image for pose estimation
        :param k: Camera intrinsics matrix of shape (3, 3)
        :param guess: Optional initial guess for camera pose
        :return: Pose tuple consisting of a rotation (3, 3) and translation (3, 1) numpy arrays
        """
        pose_estimator: PoseEstimator = globals()[PoseEstimator.POSE_ESTIMATOR_GLOBAL_VAR]

        if guess is not None:
            assert_pose(guess)

        pose = pose_estimator.estimate_pose(query, reference, k, guess)

        # Do independent check - child class might not check their output
        if pose is not None:
            assert_pose(pose)

        return pose

    @abstractmethod
    def estimate_pose(self, query: np.ndarray, reference: np.ndarray, k: np.ndarray,
                      guess: Optional[Tuple[np.ndarray, np.ndarray]]) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Returns pose between provided images, or None if pose cannot be estimated

        :param query: The first (query) image for pose estimation
        :param reference: The second (reference) image for pose estimation
        :param k: Camera intrinsics matrix of shape (3, 3)
        :param guess: Optional initial guess for camera pose
        :return: Pose tuple of rotation (3, 3) and translation (3, 1) numpy arrays, or None if could not estimate
        """
        pass
