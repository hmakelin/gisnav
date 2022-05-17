"""Interface implemented by all keypoint matchers"""
import numpy as np
import cv2

from abc import abstractmethod
from typing import Tuple, Optional

from python_px4_ros2_map_nav.matchers.matcher import Matcher
from python_px4_ros2_map_nav.assertions import assert_type
from python_px4_ros2_map_nav.data import Pose, InputData


# TODO: move keypoint visualization here
class KeypointMatcher(Matcher):
    # TODO: initialize with min_matches
    """Abstract base class for all keypoint based matchers

    This class defines a worker method that is intended to be passed to a separate process in a
    :class:`multiprocessing.pool.Pool`, along with a method for computing pose from keypoint mathces.
    """
    # Minimum matches for homography estimation, should be at least 4
    HOMOGRAPHY_MINIMUM_MATCHES = 4

    def __init__(self, min_matches: int):
        self.min_matches = max(self.HOMOGRAPHY_MINIMUM_MATCHES, min_matches or HOMOGRAPHY_MINIMUM_MATCHES)

    @abstractmethod
    def _match(self, img: np.ndarray, map_: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Uses the keypoint matcher to find matching keypoints between provided image and map

        :param img: The image captured from drone camera
        :param map_: The reference map raster of the same area (retrieved e.g. from a WMS endpoint) or previous frame
        :return: Tuple of two numpy arrays containing matched keypoint coordinates in img and map respectively
        """
        pass

    @staticmethod
    def initializer(class_name: str, *args) -> None:
        """Initializes a global matcher in a dedicated process

        Class name and args are intended to be read from a YAML file and passed to this initializer. The args are passed
        onto the class __init__ method.

        :param class_name: Name of the implementing class to initialize
        :return:
        """
        # noinspection PyGlobalUndefined
        global matcher
        matcher = class_name(*args)

    @staticmethod
    def worker(img: np.ndarray, map_: np.ndarray, input_data: InputData, guess: Optional[Pose]) -> Optional[Pose]:
        """Returns matching keypoints between provided image and map

        Requires a global :class:`~LoFTRMatcher` instance to work.

        :param img: The image captured from drone camera
        :param map_: The map raster of the same area (from e.g. WMS endpoint)
        :param input_data: The input data context
        :param guess: Optional initial guess for camera pose
        :return: Tuple of two lists containing matching keypoints in img and map respectively
        """
        try:
            assert_type(matcher, KeypointMatcher)  # see matcher.py for initialization of 'matcher'
            # noinspection PyProtectedMember
            mkp_img, mkp_map = matcher._match(img, map_)
            assert hasattr(input_data, 'k')
            return matcher._estimate_pose(mkp_img, mkp_map, input_data.k, guess)  # noqa (PyProtectedMember)
        except Exception as e:
            #raise e  # TODO: handle exception
            return None

    def _estimate_pose(self, mkp1: np.ndarray, mkp2: np.ndarray, k: np.ndarray, guess: Optional[Pose]) -> \
            Optional[Pose]:
        """Estimates pose (rotation and translation) based on found keypoint matches

        :param mkp1: Matching keypoints for image #1 (current frame)
        :param mkp2: Matching keypoints for image #2 (map or previous frame)
        :param k: Camera intrinsics matrix
        :param guess: Optional initial guess for solution
        :return: Pose estimate, or None if no estimate could be obtained
        """
        match_count = len(mkp1)
        if match_count < self.min_matches:
            return None
        assert len(mkp1) >= 4  # HOMOGRAPHY_MINIMUM_MATCHES == 4
        assert len(mkp2) == len(mkp1)
        padding = np.array([[0]]*len(mkp1))
        mkp2_3d = np.hstack((mkp2, padding))  # Set all world z-coordinates to zero
        dist_coeffs = np.zeros((4, 1))
        use_guess = guess is not None
        r, t = (guess.r, guess.t) if use_guess else (None, None)
        _, r, t, __ = cv2.solvePnPRansac(mkp2_3d, mkp1, k, dist_coeffs, r, t, useExtrinsicGuess=use_guess,
                                         iterationsCount=10)
        r, _ = cv2.Rodrigues(r)
        try:
            pose = Pose(k, r, t)
        except np.linalg.LinAlgError as _:  # e:
            # TODO: logging?
            return None

        return pose
