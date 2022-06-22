"""Module that contains an adapter for the SuperGlueEstimator GNN model."""
import os
import sys
import torch
import cv2
import numpy as np

from typing import Optional, Tuple
from enum import Enum

from gisnav.assertions import assert_type
from gisnav.pose_estimators.keypoint_pose_estimator import KeypointPoseEstimator

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

    def _find_matching_keypoints(self, query: np.ndarray, reference: np.ndarray) \
            -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Returns matching keypoints between provided query and reference image

        :param query: The first (query) image for pose estimation
        :param reference: The second (reference) image for pose estimation
        :return: Tuple of matched keypoint arrays for the images, or None if none could be found
        """
        qry_grayscale = cv2.cvtColor(query, cv2.COLOR_BGR2GRAY)
        ref_grayscale = cv2.cvtColor(reference, cv2.COLOR_BGR2GRAY)
        qry_tensor = frame2tensor(qry_grayscale, self._device)
        ref_tensor = frame2tensor(ref_grayscale, self._device)

        pred = self._matching({'image0': qry_tensor, 'image1': ref_tensor})
        pred = {k: v[0].cpu().detach().numpy() for k, v in pred.items()}
        kp_qry, kp_ref = pred['keypoints0'], pred['keypoints1']
        matches, conf = pred['matches0'], pred['matching_scores0']

        valid = np.logical_and(matches > -1, conf >= conf_threshold)
        if len(valid) == 0:
            return None
        else:
            # Valid matched keypoints ('mkp') that pass confidence threshold
            mkp_qry = kp_qry[valid]
            mkp_ref = kp_ref[matches[valid]]
            return mkp_qry, mkp_ref
