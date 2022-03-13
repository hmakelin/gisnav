"""Module that contains an adapter for the ORB descriptor based matcher."""
import os
import sys
import cv2
import numpy as np

from typing import Tuple
from enum import Enum

from python_px4_ros2_map_nav.keypoint_matchers.keypoint_matcher import KeypointMatcher
from python_px4_ros2_map_nav.assertions import assert_type


class ORB(KeypointMatcher):
    """Adapter for ORB based keypoint matcher"""

    DEFAULT_CONFIDENCE_THRESHOLD = 0.7
    """Confidence threshold for filtering out bad matches"""

    def __init__(self, params: dict) -> None:
        """Initializes instance attributes

        This method is intended to be called inside :meth:`~initializer` together with a global variable declaration
        so that attributes initialized here are also available for :meth:`~worker`. This way we avoid having to declare
        a separate global variable for each attribute.

        :param params: ORB config to be passed to :class:`models.matching.Matching`
        """
        self._config = params  # TODO: this needed?
        self._orb = cv2.ORB_create()
        self._bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

    @property
    def _orb(self) -> cv2.ORB:
        """ORB keypoint detector and descriptor"""
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

    @staticmethod
    def initializer(params: dict) -> None:
        """Instantiates :class:`~ORB` as a global variable

        This makes the :class:`cv2.ORB` instance of :py:attr:`~_orb` available to be called by :meth:`~worker` as long
        as it is called in the same process.

        :param params: ORB config to be passed to :func:`cv2.ORB_create`
        :return:
        """
        # noinspection PyGlobalUndefined
        global orb
        orb = ORB(params)

    @staticmethod
    def worker(img: np.ndarray, map_: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Returns matching keypoints between provided image and map

        Requires a global :class:`~ORB` instance to work.

        :param img: The image captured from drone camera
        :param map_: The map raster of the same area (from e.g. WMS endpoint)
        :return: Tuple of two arrays containing matching keypoints in img and map respectively
        """
        try:
            assert_type(orb, ORB)
            return orb._match(img, map_)
        except Exception as e:
            # TODO: handle exception
            raise e

    def _match(self, img: np.ndarray, map_: np.ndarray, conf_threshold: float = DEFAULT_CONFIDENCE_THRESHOLD) \
            -> Tuple[np.ndarray, np.ndarray]:
        """Uses ORB to find matching keypoints between provided image and map

        Note: map_ may be any image, not necessarily a map if this is used for visual odometry.

        :param img: The image captured from drone camera
        :param map_: The map raster of the same area (retrieved e.g. from a WMS endpoint)
        :param conf_threshold: Confidence threshold for filtering out bad matches
        :return: Tuple of two numpy arrays containing matched keypoint coordinates in img and map respectively
        """
        img_grayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        map_grayscale = cv2.cvtColor(map_, cv2.COLOR_BGR2GRAY)

        kp_img, desc_img = self._orb.detectAndCompute(img_grayscale, None)
        kp_map, desc_map = self._orb.detectAndCompute(map_grayscale, None)

        matches = self._bf.match(desc_img, desc_map)
        #matches = sorted(matches, key=lambda x: x.distance)
        #valid = []
        #for m, n in matches:
        #    if m.distance < conf_threshold * n.distance:
        #        valid.append(m)

        mkp_img = np.float32([kp_img[m.queryIdx].pt for m in valid]).reshape(-1, 1, 2)
        mkp_map = np.float32([kp_map[m.trainIdx].pt for m in valid]).reshape(-1, 1, 2)

        return mkp_img, mkp_map
