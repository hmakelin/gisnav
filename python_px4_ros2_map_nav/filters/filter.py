"""Module that contains the abstract base class for filters"""
import numpy as np

from abc import ABC, abstractmethod
from typing import Optional, Tuple


class Filter(ABC):
    """Abstract base class for Kalman and Particle filters"""

    @abstractmethod
    def update(self, position: np.ndarray) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Returns a filtered position with estimated standard deviations

        :param position: A new position observation (measurement)
        :return: Tuple of position (3, 1) and standard deviation (3, 1) numpy arrays
        """
        pass
