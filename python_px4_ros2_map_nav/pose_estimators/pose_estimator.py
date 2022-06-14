"""Module that defines an abstract base class for all pose estimators"""
from __future__ import annotations  # Python version 3.7+

import numpy as np

from abc import ABC, abstractmethod
from typing import Optional

from python_px4_ros2_map_nav.assertions import assert_type


class PoseEstimator(ABC):
    """Abstract base class for all pose estimators

    This class implements static initializer and worker methods that are intended to be passed to a separate process or
    thread in e.g. a :class:`multiprocessing.pool.Pool` or :class:`multiprocessing.pool.ThreadPool`. An abstract
    :meth:`.estimate_pose` method must be implemented by extending classes.
    """

    _POSE_ESTIMATOR_GLOBAL_VAR = '_pose_estimator'
    """A global pose estimator instance is stored under this name"""

    @staticmethod
    def initializer(class_: type, *args) -> None:
        """Initializes a global pose estimator with the provided constructor

        The args are passed onto the class __init__ method.

        Intended to be used together with :class:`.worker` with :py:mod:`.multiprocessing`.

        :param class_: The implementing :class:`.PoseEstimator` class to initialize
        :return:
        """
        assert_type(class_, type)
        if not PoseEstimator._POSE_ESTIMATOR_GLOBAL_VAR in globals():
            pose_estimator = class_(*args)
            assert issubclass(type(pose_estimator), PoseEstimator)
            globals()[PoseEstimator._POSE_ESTIMATOR_GLOBAL_VAR] = pose_estimator

    @staticmethod
    def worker(query: np.ndarray, reference: np.ndarray, k: np.ndarray,
               guess: Optional[Tuple[np.ndarray, np.ndarray]]) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Returns Pose between provided images

        Intended to be used together with :class:`.initializer` with :py:mod:`.multiprocessing`

        :param query: The first (query) image for pose estimation
        :param reference: The second (reference) image for pose estimation
        :param k: Camera intrinsics matrix (3, 3)
        :param guess: Optional initial guess for camera pose
        :return: Pose tuple consisting of a rotation (3, 3) and translation (3, 1) np.ndarrays
        """
        assert PoseEstimator._POSE_ESTIMATOR_GLOBAL_VAR in globals()
        pose_estimator = globals()[PoseEstimator._POSE_ESTIMATOR_GLOBAL_VAR]
        assert issubclass(type(pose_estimator), PoseEstimator)

        # Ensure guess passed on to estimate_pose is valid
        guess = guess if PoseEstimator._check_pose(guess) else None

        # TODO: raise error if query, reference or k (or guess?) are not valid

        pose = pose_estimator.estimate_pose(query, reference, k, guess)

        return pose if PoseEstimator._check_pose(pose) else None

    # TODO: same logic in data.py Pose class, make this an assertion and move to assertions.py? Use common definitions
    #  the shapes (3, 3) and (3, 1), they would be encoded there in one place
    @staticmethod
    def _check_pose(pose: Optional[Tuple[np.ndarray, np.ndarray]]) -> bool:
        """Helper method to check validity of a pose tuple

        :param pose: Tuple consisting of a rotation (3, 3) and translation (3, 1) np.ndarrays with valid values
        :return: True if pose tuple is correctly formatted and valid
        """
        if pose is not None:
            r, t = pose
            if np.isnan(r).any() or np.isnan(t).any() or r.shape != (3, 3) or t.shape != (3, 1):
                return False
            else:
                # TODO: check rotation matrix orthogonality as well?
                return True
        else:
            return False

    @abstractmethod
    def estimate_pose(self, query: np.ndarray, reference: np.ndarray, k: np.ndarray,
                      guess: Optional[Tuple[np.ndarray, np.ndarray]]) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Returns pose between given query and reference images, or None if no pose could not be estimated

        :param query: The first (query) image for pose estimation
        :param reference: The second (reference) image for pose estimation
        :param k: Camera intrinsics matrix (3, 3)
        :param guess: Optional initial guess for camera pose
        :return: Pose tuple consisting of a rotation (3, 3) and translation (3, 1) np.ndarrays
        """