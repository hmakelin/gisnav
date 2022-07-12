"""Module that contains an adapter for the LoFTR model."""
import os
import torch
import cv2
import numpy as np

from typing import Optional, Tuple
from enum import Enum
from gisnav.pose_estimators.keypoint_pose_estimator import KeypointPoseEstimator
from LoFTR.loftr import LoFTR, default_cfg

from ament_index_python.packages import get_package_share_directory


class LoFTRPoseEstimator(KeypointPoseEstimator):
    """Adapter for LoFTR keypoint matcher"""

    # TODO: redundant implementation in superglue_pose_estimator.py
    class TorchDevice(Enum):
        """Possible devices on which torch tensors are allocated."""
        CPU = 'cpu'
        CUDA = 'cuda'

    WEIGHTS_PATH = 'weights/outdoor_ds.ckpt'
    """Path to model weights - for LoFTR these need to be downloaded separately (see LoFTR README.md)"""

    CONFIDENCE_THRESHOLD = 0.7
    """Confidence threshold for filtering out bad matches"""

    def __init__(self, min_matches: int) -> None:
        """Class initializer

        This method is intended to be called inside :meth:`.initializer` together with a global variable declaration
        so that attributes initialized here are also available for :meth:`.worker`.

        :param min_matches: Minimum required keypoint matches (should be >= 4)
        """
        super(LoFTRPoseEstimator, self).__init__(min_matches)
        self._device = LoFTRPoseEstimator.TorchDevice.CUDA.value if torch.cuda.is_available() else \
            LoFTRPoseEstimator.TorchDevice.CPU.value
        self._model = LoFTR(config=default_cfg)
        weights_path = os.path.join(get_package_share_directory('gisnav'), self.WEIGHTS_PATH)  # TODO: provide as arg to constructor, do not hard-code path here
        self._model.load_state_dict(torch.load(weights_path)['state_dict'])
        self._model = self._model.eval().cuda()

    def _find_matching_keypoints(self, query: np.ndarray, reference: np.ndarray) \
            -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Returns matching keypoints between provided query and reference image

        :param query: The first (query) image for pose estimation
        :param reference: The second (reference) image for pose estimation
        :return: Tuple of matched keypoint arrays for the images, or None if none could be found
        """
        qry_grayscale = cv2.cvtColor(query, cv2.COLOR_BGR2GRAY)
        ref_grayscale = cv2.cvtColor(reference, cv2.COLOR_BGR2GRAY)
        qry_tensor = torch.from_numpy(qry_grayscale)[None][None].cuda() / 255.
        ref_tensor = torch.from_numpy(ref_grayscale)[None][None].cuda() / 255.

        batch = {'image0': qry_tensor, 'image1': ref_tensor}

        with torch.no_grad():
            self._model(batch)
            mkp_qry = batch['mkpts0_f'].cpu().numpy()
            mkp_ref = batch['mkpts1_f'].cpu().numpy()
            conf = batch['mconf'].cpu().numpy()

        valid = conf > self.CONFIDENCE_THRESHOLD
        if len(valid) == 0:
            return None
        else:
            mkp_qry = mkp_qry[valid, :]
            mkp_ref = mkp_ref[valid, :]
            return mkp_qry, mkp_ref
