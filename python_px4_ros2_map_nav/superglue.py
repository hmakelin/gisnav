"""Module that contains an adapter for the SuperGlue GNN model."""
import rclpy.impl.rcutils_logger
import torch
import cv2
import numpy as np

from typing import Tuple
# Assumes models has been added to path (see import statements in map_nav_node.py)
from models.matching import Matching
from models.utils import frame2tensor


class SuperGlue:
    """Matches img to map, see code in match_pairs.py for further examples."""

    DEFAULT_SUPERPOINT_NMS_RADIUS = 3
    DEFAULT_SUPERPOINT_KEYPOINT_THRESHOLD = 0.005
    DEFAULT_SUPERPOINT_MAX_KEYPOINTS = 2048
    DEFAULT_SUPERGLUE_WEIGHTS = 'outdoor'
    DEFAULT_SUPERGLUE_SINKHORN_ITERATIONS = 20
    DEFAULT_SUPERGLUE_MATCH_THRESHOLD = 0.2

    def __init__(self, config: dict, logger: rclpy.impl.rcutils_logger.RcutilsLogger = None):
        """Initializer

        :param config: Dict with SuperGlue params parameters.
        :param logger: Path to directory where to store output visualization.
        """
        self._config = config
        self._device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self._logger = logger
        if self._logger is not None:
            self._logger.debug('SuperGlue using device {}'.format(self._device))
            self._logger.debug('SuperGlue using params {}'.format(self._config))
        self._matching = Matching(self._config).eval().to(self._device)

    def match(self, img: np.ndarray, map_: np.ndarray, confidence: float = 0.3) -> Tuple[np.ndarray, np.ndarray]:
        """Matches image to map.

        :param img: The image array
        :param map_: The map array
        :param confidence: Confidence threshold for valid matched keypoints
        :return: Matched keypoints
        """
        img_grayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        map_grayscale = cv2.cvtColor(map_, cv2.COLOR_BGR2GRAY)
        img = frame2tensor(img_grayscale, self._device)
        map_ = frame2tensor(map_grayscale, self._device)

        pred = self._matching({'image0': img, 'image1': map_})  # TODO: check that img and map are formatted correctly

        pred = {k: v[0].cpu().detach().numpy() for k, v in pred.items()}
        kp_img, kp_map = pred['keypoints0'], pred['keypoints1']
        matches, conf = pred['matches0'], pred['matching_scores0']

        # Matching keypoints
        valid = np.logical_and(matches > -1, conf >= confidence)
        mkp_img = kp_img[valid]
        mkp_map = kp_map[matches[valid]]

        return mkp_img, mkp_map
