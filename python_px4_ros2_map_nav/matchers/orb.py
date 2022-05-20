"""Module that contains an adapter for the ORBMatcher descriptor based matcher."""
import os
import sys
import cv2
import numpy as np

from typing import Tuple
from enum import Enum

from python_px4_ros2_map_nav.data import ImagePair
from python_px4_ros2_map_nav.matchers.keypoint_matcher import KeypointMatcher
from python_px4_ros2_map_nav.assertions import assert_type


class ORBMatcher(KeypointMatcher):
    """Adapter for ORBMatcher based keypoint matcher"""

    DEFAULT_CONFIDENCE_THRESHOLD = 0.7
    """Confidence threshold for filtering out bad matches"""

    MAX_KEYPOINTS = 40  # Should be 2x (?) larger than minimum required matches
    """Maximum number of keypoints and descriptors to return for matching"""

    def __init__(self, min_matches: int) -> None:
        """Initializes instance attributes

        This method is intended to be called inside :meth:`~initializer` together with a global variable declaration
        so that attributes initialized here are also available for :meth:`~worker`. This way we avoid having to declare
        a separate global variable for each attribute.

        :param min_matches: Minimum required keypoint matches (should be >= 4)
        """
        super(ORBMatcher, self).__init__(min_matches)
        self._orb = cv2.ORB_create()
        self._bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    @property
    def _orb(self) -> cv2.ORB:
        """ORBMatcher keypoint detector and descriptor"""
        return self.__orb

    @_orb.setter
    def _orb(self, value: cv2.ORB) -> None:
        assert_type(value, cv2.ORB)
        self.__orb = value

    @property
    def _bf(self) -> cv2.BFMatcher:
        """Brute force matcher for matching keypoints"""
        return self.__bf

    @_bf.setter
    def _bf(self, value: cv2.BFMatcher) -> None:
        assert_type(value, cv2.BFMatcher)
        self.__bf = value

    def _match(self, image_pair: ImagePair, conf_threshold: float = DEFAULT_CONFIDENCE_THRESHOLD) \
            -> Tuple[np.ndarray, np.ndarray]:
        """Uses ORBMatcher to find matching keypoints between provided image and map

        Note: map_ may be any image, not necessarily a map if this is used for visual odometry.

        :param image_pair: The image pair to match
        :param conf_threshold: Confidence threshold for filtering out bad matches
        :return: Tuple of two numpy arrays containing matched keypoint coordinates in img and map respectively
        """
        img_grayscale = cv2.cvtColor(image_pair.img.image.arr, cv2.COLOR_BGR2GRAY)
        map_grayscale = cv2.cvtColor(image_pair.ref.image.arr, cv2.COLOR_BGR2GRAY)

        kp_img, desc_img = self._orb.detectAndCompute(img_grayscale, None)
        kp_map, desc_map = self._orb.detectAndCompute(map_grayscale, None)

        matches = self._bf.match(desc_img, desc_map)
        matches = sorted(matches, key=lambda x: x.distance)
        matches = matches[:min(len(matches), self.MAX_KEYPOINTS)]

        mkp_img = np.float32([kp_img[m.queryIdx].pt for m in matches]).reshape(-1, 2)
        mkp_map = np.float32([kp_map[m.trainIdx].pt for m in matches]).reshape(-1, 2)

        return mkp_img, mkp_map
