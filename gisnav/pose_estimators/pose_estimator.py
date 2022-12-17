"""Module that defines an abstract base class for all pose estimators"""
from __future__ import annotations  # Python version 3.7+

import numpy as np

from abc import ABC, abstractmethod
from typing import Optional, Tuple


class PoseEstimator(ABC):
    """Abstract base class for all pose estimators"""

    @abstractmethod
    def estimate(self, query: np.ndarray, reference: np.ndarray, k: np.ndarray,
                 guess: Optional[Tuple[np.ndarray, np.ndarray]] = None,
                 elevation_reference: Optional[np.ndarray] = None) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Returns pose between provided images, or None if pose cannot be estimated

        :param query: The first (query) image for pose estimation
        :param reference: The second (reference) image for pose estimation
        :param k: Camera intrinsics matrix of shape (3, 3)
        :param guess: Optional initial guess for camera pose
        :param elevation_reference: Optional elevation raster (same size resolution as reference image, grayscale)
        :return: Pose tuple of rotation (3, 3) and translation (3, 1) numpy arrays, or None if could not estimate
        """
        pass
