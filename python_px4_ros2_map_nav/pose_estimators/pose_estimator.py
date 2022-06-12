"""Module that defines an abstract base class for all pose estimators"""
from __future__ import annotations  # Python version 3.7+

import numpy as np

from abc import ABC, abstractmethod
from typing import Optional
from python_px4_ros2_map_nav.data import ImagePair, Pose
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
    def initializer(class_name: type, *args) -> None:
        """Initializes a global pose estimator with the provided constructor

        The args are passed onto the class __init__ method.

        :param class_name: The implementing class to initialize
        :return:
        """
        assert_type(class_name, type)
        if not PoseEstimator._POSE_ESTIMATOR_GLOBAL_VAR in globals():
            pose_estimator = class_name(*args)
            assert issubclass(type(pose_estimator), PoseEstimator)
            globals()[PoseEstimator._POSE_ESTIMATOR_GLOBAL_VAR] = pose_estimator

    @staticmethod
    def worker(image_pair: ImagePair, guess: Optional[Pose]) -> Optional[Pose]:
        """Returns Pose between provided images

        :param image_pair: Image pair for pose estimation
        :param guess: Optional initial guess for camera pose
        :return: Pose between images, or None if no pose could be estimated
        """
        assert PoseEstimator._POSE_ESTIMATOR_GLOBAL_VAR in globals()
        pose_estimator = globals()[PoseEstimator._POSE_ESTIMATOR_GLOBAL_VAR]
        assert issubclass(type(pose_estimator), PoseEstimator)
        return pose_estimator.estimate_pose(image_pair, guess)

    @abstractmethod
    def estimate_pose(self, image_pair: ImagePair, guess: Optional[Pose]) -> Optional[Pose]:
        """Returns camera pose between image pair, or None if no pose could not be estimated"""
        pass