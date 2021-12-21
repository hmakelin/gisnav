"""This module adapts the SuperGlue match_pairs.py demo code for this app."""
import torch
import cv2
import numpy as np

# Assumes models has been added to path (see import statements in map_nav_node.py)
from models.matching import Matching
from models.utils import frame2tensor


class SuperGlue:
    """Matches img to map, adapts code from match_pairs.py so that do not have to write files to disk."""

    def __init__(self, config, logger=None):
        """Init the SuperGlue matcher.

        Args:
            config - Dict with SuperGlue config parameters.
            output_dir - Path to directory where to store output visualization.
            logger - ROS2 node logger for logging messages.
        """
        self._config = config
        self._device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self._logger = logger
        if self._logger is not None:
            self._logger.debug('SuperGlue using device {}'.format(self._device))

        if self._logger is not None:
            self._logger.debug('SuperGlue using config {}'.format(self._config))
        self._matching = Matching(self._config).eval().to(self._device)

    def match(self, img, map, confidence=0.7):
        """Match img to map.

        Arguments:
            img - The image frame.
            map - The map frame.
            confidence - Confidence threshold for filtering returned matches.
        """
        img_grayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        map_grayscale = cv2.cvtColor(map, cv2.COLOR_BGR2GRAY)
        img = frame2tensor(img_grayscale, self._device)
        map = frame2tensor(map_grayscale, self._device)

        pred = self._matching({'image0': img, 'image1': map})  # TODO: check that img and map are formatted correctly

        pred = {k: v[0].cpu().detach().numpy() for k, v in pred.items()}
        kp_img, kp_map = pred['keypoints0'], pred['keypoints1']
        matches, conf = pred['matches0'], pred['matching_scores0']

        # Matching keypoints
        valid = np.logical_and(matches > -1, conf >= confidence)
        mkp_img = kp_img[valid]
        mkp_map = kp_map[matches[valid]]

        return mkp_img, mkp_map
