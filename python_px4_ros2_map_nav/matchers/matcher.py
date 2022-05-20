"""Interface implemented by all matchers (:=_pose estimators)"""
import numpy as np

from abc import ABC, abstractmethod
from typing import Optional
from python_px4_ros2_map_nav.data import Pose, InputData, ImagePair


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
    def worker(image_pair: ImagePair, guess: Optional[Pose]) -> Optional[Pose]:
        """Returns camera _pose between input images

        :param image_pair: Image pair to match
        :param guess: Optional guess for the solution
        :return: Camera _pose between the images, or None if estimate could not be obtained
        """
        pass
