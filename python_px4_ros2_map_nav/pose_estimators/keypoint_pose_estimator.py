"""Module that defines and abstract base class for keypoint-based pose estimators"""
import numpy as np
import cv2

from abc import abstractmethod
from typing import Tuple, Optional
from dataclasses import dataclass, field

from python_px4_ros2_map_nav.pose_estimators.pose_estimator import PoseEstimator
from python_px4_ros2_map_nav.data import ImagePair, Pose
from python_px4_ros2_map_nav.assertions import assert_type, assert_len


class KeypointPoseEstimator(PoseEstimator):
    """Abstract base class for all keypoint-based pose estimators

    This class implements the :meth:`.estimate_pose` method that estimates a pose from matched keypoints. An abstract
    :meth:`.find_matching_keypoints` method must be implemented by extending classes.
    """
    _HOMOGRAPHY_MINIMUM_MATCHES = 4
    """Minimum matches for homography estimation, should be at least 4"""

    def __init__(self, min_matches: int):
        self._min_matches = max(self._HOMOGRAPHY_MINIMUM_MATCHES, min_matches or _HOMOGRAPHY_MINIMUM_MATCHES)

    # noinspection PyClassHasNoInit
    @dataclass(frozen=True)
    class KeypointMatches:
        """Holds matching keypoints for :class:`.ImagePair`"""
        query_keypoints: np.ndarray
        reference_keypoints: np.ndarray
        count: int = field(init=False)

        def __post_init__(self):
            """Post-initialization validity checks"""
            assert_len(self.query_keypoints, len(self.reference_keypoints))
            object.__setattr__(self, 'count', len(self.reference_keypoints))

    @abstractmethod
    def _find_matching_keypoints(self, image_pair: ImagePair) -> Optional[KeypointMatches]:
        """Returns matching keypoints between provided image pair

        Note that this method is called by :meth:`.estimate_pose` and should not be used outside the implementing
        class.

        :param image_pair: The image pair to find matching keypoints for
        :return: Matched keypoints, or None if none could be found
        """
        pass

    def estimate_pose(self, image_pair: ImagePair, guess: Optional[Pose]) -> Optional[Pose]:
        """Estimates pose from keypoints matched by :meth:`.find_matching_keypoints`

        :param image_pair: Image pair to estimate pose
        :param guess: Optional initial guess for solution
        :return: Pose estimate, or None if no estimate could be obtained
        """
        matched_keypoints = self._find_matching_keypoints(image_pair)
        if matched_keypoints is None or matched_keypoints.count < self._min_matches:
            return None  # Not enough matching keypoints found

        mkp1, mkp2 = matched_keypoints.query_keypoints, matched_keypoints.reference_keypoints
        padding = np.array([[0]] * len(mkp1))
        mkp2_3d = np.hstack((mkp2, padding))  # Set world z-coordinates to zero
        dist_coeffs = np.zeros((4, 1))
        use_guess = guess is not None
        r, t = (guess.r, guess.t) if use_guess else (None, None)
        _, r, t, __ = cv2.solvePnPRansac(mkp2_3d, mkp1, image_pair.qry.camera_data.k, dist_coeffs, r, t,
                                         useExtrinsicGuess=use_guess, iterationsCount=10)
        r, _ = cv2.Rodrigues(r)

        try:
            return Pose(r, t)
        except Pose.PoseValueError as _:
            return None  # The estimate is invalid
