"""Module that contains an adapter for the SuperGlue GNN model."""
import os
import sys
import torch
import cv2
import numpy as np

from typing import Tuple
from enum import Enum
from python_px4_ros2_map_nav.data import ImagePair
from python_px4_ros2_map_nav.assertions import assert_type
from python_px4_ros2_map_nav.matchers.keypoint_matcher import KeypointMatcher
from SuperGluePretrainedNetwork.models.matching import Matching
from SuperGluePretrainedNetwork.models.utils import frame2tensor


class SuperGlue(KeypointMatcher):
    """Adapter for Superglue, and Attentional Graph Neural Network based keypoint matcher"""

    DEFAULT_CONFIDENCE_THRESHOLD = 0.7
    """Confidence threshold for filtering out bad matches"""

    class TorchDevice(Enum):
        """Possible devices on which torch tensors are allocated."""
        CPU = 'cpu'
        CUDA = 'cuda'

    def __init__(self, min_matches: int, params: dict) -> None:
        """Initializes instance attributes

        This method is intended to be called inside :meth:`~initializer` together with a global variable declaration
        so that attributes initialized here are also available for :meth:`~worker`. This way we avoid having to declare
        a separate global variable for each attribute.

        :param min_matches: Minimum required keypoint matches (should be >= 4)
        :param params: SuperGlue config to be passed to :class:`models.matching.Matching`
        """
        super(SuperGlue, self).__init__(min_matches)
        self._device = SuperGlue.TorchDevice.CUDA.value if torch.cuda.is_available() else \
            SuperGlue.TorchDevice.CPU.value
        self._matching = Matching(params).eval().to(self._device)

    @property
    def _device(self) -> str:
        """Device on which torch tensors are allocated."""
        return self.__device

    @_device.setter
    def _device(self, value: str) -> None:
        assert_type(value, str)
        self.__device = value

    @property
    def _matching(self) -> Matching:
        """Holds a :class:`models.matching.Matching` instance for performing SuperGlue matches"""
        return self.__matching

    @_matching.setter
    def _matching(self, value: Matching) -> None:
        assert_type(value, Matching)
        self.__matching = value

    def _match(self, image_pair: ImagePair, conf_threshold: float = DEFAULT_CONFIDENCE_THRESHOLD) \
            -> Tuple[np.ndarray, np.ndarray]:
        """Uses SuperGlue to find matching keypoints between provided image and map

        :param image_pair: The image pair to match
        :param conf_threshold: Confidence threshold for filtering out bad matches
        :return: Tuple of two numpy arrays containing matched keypoint coordinates in img and map respectively
        """
        img_grayscale = cv2.cvtColor(image_pair.img.image.arr, cv2.COLOR_BGR2GRAY)
        map_grayscale = cv2.cvtColor(image_pair.ref.image.arr, cv2.COLOR_BGR2GRAY)
        img_tensor = frame2tensor(img_grayscale, self._device)
        map_tensor = frame2tensor(map_grayscale, self._device)

        pred = self._matching({'image0': img_tensor, 'image1': map_tensor})
        pred = {k: v[0].cpu().detach().numpy() for k, v in pred.items()}
        kp_img, kp_map = pred['keypoints0'], pred['keypoints1']
        matches, conf = pred['matches0'], pred['matching_scores0']

        # Valid matched keypoints ('mkp') that pass confidence threshold
        valid = np.logical_and(matches > -1, conf >= conf_threshold)
        mkp_img = kp_img[valid]
        mkp_map = kp_map[matches[valid]]

        return mkp_img, mkp_map
