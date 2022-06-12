"""Module that contains an adapter for the SuperGlueEstimator GNN model."""
import os
import sys
import torch
import cv2
import numpy as np

from typing import Optional, Tuple
from enum import Enum

from python_px4_ros2_map_nav.data import ImagePair
from python_px4_ros2_map_nav.assertions import assert_type
from python_px4_ros2_map_nav.pose_estimators.keypoint_pose_estimator import KeypointPoseEstimator

from SuperGluePretrainedNetwork.models.matching import Matching
from SuperGluePretrainedNetwork.models.utils import frame2tensor


class SuperGlueEstimator(KeypointPoseEstimator):
    """Adapter for Superglue, an Attentional Graph Neural Network based keypoint matcher"""

    DEFAULT_CONFIDENCE_THRESHOLD = 0.7
    """Confidence threshold for filtering out bad matches"""

    class TorchDevice(Enum):
        """Possible devices on which torch tensors are allocated."""
        CPU = 'cpu'
        CUDA = 'cuda'

    def __init__(self, min_matches: int, params: dict) -> None:
        """Class initializer

        This method is intended to be called inside :meth:`.initializer` together with a global variable declaration
        so that attributes initialized here are also available for :meth:`.worker`.

        :param min_matches: Minimum required keypoint matches (should be >= 4)
        :param params: SuperGlueEstimator config to be passed to :class:`models.matching.Matching`
        """
        super(SuperGlueEstimator, self).__init__(min_matches)
        self._device = SuperGlueEstimator.TorchDevice.CUDA.value if torch.cuda.is_available() else \
            SuperGlueEstimator.TorchDevice.CPU.value
        self._matching = Matching(params).eval().to(self._device)

    def _find_matching_keypoints(self, image_pair: ImagePair, conf_threshold: float = DEFAULT_CONFIDENCE_THRESHOLD) \
            -> Optional[KeypointPoseEstimator.KeypointMatches]:
        """Finds matching keypoints between provided image and map

        :param image_pair: The image pair to match
        :param conf_threshold: Confidence threshold for filtering out bad matches
        :return: Matched keypoints, or None if none could be found
        """
        img_grayscale = cv2.cvtColor(image_pair.qry.image.arr, cv2.COLOR_BGR2GRAY)
        map_grayscale = cv2.cvtColor(image_pair.ref.image.arr, cv2.COLOR_BGR2GRAY)
        img_tensor = frame2tensor(img_grayscale, self._device)
        map_tensor = frame2tensor(map_grayscale, self._device)

        pred = self._matching({'image0': img_tensor, 'image1': map_tensor})
        pred = {k: v[0].cpu().detach().numpy() for k, v in pred.items()}
        kp_img, kp_map = pred['keypoints0'], pred['keypoints1']
        matches, conf = pred['matches0'], pred['matching_scores0']

        valid = np.logical_and(matches > -1, conf >= conf_threshold)
        if len(valid) == 0:
            return None
        else:
            # Valid matched keypoints ('mkp') that pass confidence threshold
            mkp_img = kp_img[valid]
            mkp_map = kp_map[matches[valid]]
            return KeypointPoseEstimator.KeypointMatches(query_keypoints=mkp_img, reference_keypoints=mkp_map)
