"""Module that contains an adapter for the LoFTR model."""
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
from LoFTR.loftr import LoFTR, default_cfg

from ament_index_python.packages import get_package_share_directory


class LoFTREstimator(KeypointPoseEstimator):
    """Adapter for LoFTR keypoint matcher"""

    # TODO: redundant implementation in superglue.py
    class TorchDevice(Enum):
        """Possible devices on which torch tensors are allocated."""
        CPU = 'cpu'
        CUDA = 'cuda'

    WEIGHTS_PATH = 'LoFTR/weights/outdoor_ds.ckpt'
    """Path to model weights - for LoFTR these need to be downloaded separately (see LoFTR README.md)"""

    CONFIDENCE_THRESHOLD = 0.7
    """Confidence threshold for filtering out bad matches"""

    def __init__(self, min_matches: int) -> None:
        """Class initializer

        This method is intended to be called inside :meth:`.initializer` together with a global variable declaration
        so that attributes initialized here are also available for :meth:`.worker`.

        :param min_matches: Minimum required keypoint matches (should be >= 4)
        """
        super(LoFTREstimator, self).__init__(min_matches)
        self._device = LoFTREstimator.TorchDevice.CUDA.value if torch.cuda.is_available() else \
            LoFTREstimator.TorchDevice.CPU.value
        self._model = LoFTR(config=default_cfg)
        weights_path = os.path.join(get_package_share_directory('python_px4_ros2_map_nav'), self.WEIGHTS_PATH)  # TODO: provide as arg to constructor, do not hard-code path here
        self._model.load_state_dict(torch.load(weights_path)['state_dict'])
        self._model = self._model.eval().cuda()

    def _find_matching_keypoints(self, image_pair: ImagePair) -> Optional[KeypointPoseEstimator.KeypointMatches]:
        """Uses LoFTR to find matching keypoints between provided image pair

        :param image_pair: The image pair to match
        :return: Matched keypoints, or None if none could be found
        """
        img_grayscale = cv2.cvtColor(image_pair.qry.image.arr, cv2.COLOR_BGR2GRAY)
        map_grayscale = cv2.cvtColor(image_pair.ref.image.arr, cv2.COLOR_BGR2GRAY)
        img_tensor = torch.from_numpy(img_grayscale)[None][None].cuda() / 255.
        map_tensor = torch.from_numpy(map_grayscale)[None][None].cuda() / 255.

        batch = {'image0': img_tensor, 'image1': map_tensor}

        with torch.no_grad():
            self._model(batch)
            mkp_img = batch['mkpts0_f'].cpu().numpy()
            mkp_map = batch['mkpts1_f'].cpu().numpy()
            conf = batch['mconf'].cpu().numpy()

        valid = conf > self.CONFIDENCE_THRESHOLD
        if len(valid) == 0:
            return None
        else:
            mkp_img = mkp_img[valid, :]
            mkp_map = mkp_map[valid, :]
            return KeypointPoseEstimator.KeypointMatches(query_keypoints=mkp_img, reference_keypoints=mkp_map)
