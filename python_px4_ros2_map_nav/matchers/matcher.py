"""Interface implemented by all matchers (:=pose estimators)"""
import numpy as np

from abc import ABC, abstractmethod
from typing import Optional
from python_px4_ros2_map_nav.data import Pose, InputData


class Matcher(ABC):
    """Abstract base class for all matchers

    This class defines static initializer and worker methods that are intended to be passed to a separate process in a
    :class:`multiprocessing.pool.Pool`.
    """
    @staticmethod
    @abstractmethod
    def initializer(class_name: str, *args) -> None:
        """Initializes a global matcher in a dedicated process

        Class name and args are intended to be read from a YAML file and passed to this initializer. The args are passed
        onto the class __init__ method.

        :param class_name: Name of the implementing class to initialize
        :return:
        """
        pass

    @staticmethod
    @abstractmethod
    def worker(image: np.ndarray, reference_image: np.ndarray, input_data: InputData, guess: Optional[Pose]) -> \
            Optional[Pose]:
        """Returns camera pose between input images

        :param image: Image captured from vehicle camera
        :param reference_image: Map raster retrieved from e.g. WMS endpoint, or previous frame from vehicle camera
        :param input_data: The input data context for the image pair that may be needed in pose estimation
        :param guess: Optional guess for the solution
        :return: Camera pose between the images, or None if estimate could not be obtained
        """
        pass
