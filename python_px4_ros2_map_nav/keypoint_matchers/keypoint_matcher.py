"""Interface implemented by all keypoint matchers"""
import numpy as np

from abc import ABC, abstractmethod
from typing import Tuple


class KeypointMatcher(ABC):
    """Abstract base class for all keypoint matchers

    This class defines static initializer and worker methods that are intended to be passed to a separate process in a
    :class:`multiprocessing.pool.Pool`.
    """

    @staticmethod
    @abstractmethod
    def initializer(*args) -> None:
        """Initializes the keypoint matcher in a dedicated process"""
        pass

    @staticmethod
    @abstractmethod
    def worker(image: np.ndarray, map_: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Returns matching keypoints for provided image and map raster

        :param image: Image captured from vehicle camera
        :param map_: Map raster retrieved from e.g. WMS endpoint
        :return: Tuple containing the numpy arrays of matched keypoints for image and map, respectively
        """
        pass
