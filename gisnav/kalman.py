"""Kalman filter for producing smoothed position estimates"""
import numpy as np
from typing import Optional, Tuple
from pykalman import KalmanFilter

from gisnav.assertions import assert_type, assert_shape


class SimpleFilter:
    """Simple Kalman filter implementation

    Implements 3D model with position and velocity amounting to a total of 6 state variables. Assumes only position is
    observed.
    """
    _MIN_MEASUREMENTS = 20
    """Default minimum measurements before outputting an estimate"""

    def __init__(self, window_length=_MIN_MEASUREMENTS):
        """Class initializer

        :param window_length: Minimum number of measurements before providing a state estimate
        """
        assert window_length > 0
        self._measurements = None
        self._initial_state_mean = None
        self._transition_matrix = np.array([
            [1, 1, 0, 0, 0, 0],  # x(t) = x(t-1) + x_vel(t-1)*dt
            [0, 1, 0, 0, 0, 0],  # x_vel(t) = x_vel(t-1)
            [0, 0, 1, 1, 0, 0],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 1],
            [0, 0, 0, 0, 0, 1]
        ])
        self._observation_matrix = np.array([
            [1, 0, 0, 0, 0, 0],  # x
            [0, 0, 1, 0, 0, 0],  # y
            [0, 0, 0, 0, 1, 0],  # z
        ])
        self._kf = None  # Initialized when enough measurements are available
        self._window_length = window_length
        self._previous_mean = None
        self._previous_covariance = None

    def _init_initial_state(self, measurement: np.ndarray) -> None:
        """Initializes initial state once measurements are available

        :param measurements: First available measurements in numpy array (oldest first)
        """
        self._measurements = measurement
        self._initial_state_mean = np.array([
            self._measurements[0][0],   # x
            0,                          # x_vel
            self._measurements[0][1],   # y
            0,                          # y_vel
            self._measurements[0][2],   # z
            0                           # z_vel
        ])

    def update(self, measurement: np.ndarray) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Returns position and standard deviation estimates, or None if not enough data is available yet

        :param measurement: A new measurement (position observation)
        :return: Tuple of filtered measurement means and standard deviations, or None if output not yet available
        """
        if self._measurements is None:
            # First measurement, save initial state
            self._init_initial_state(measurement)
            return None
        elif len(self._measurements) < self._window_length:
            # Store new measurement, not enough measurements yet
            self._measurements = np.vstack((self._measurements, measurement))
            return None
        else:
            # No need to maintain self._measurements, use online filtering (filter_update)
            #self._measurements = self._measurements[len(self._measurements) - self._MIN_MEASUREMENTS:]
            #self._measurements = np.vstack((self._measurements, measurement))
            if self._kf is None:
                # Initialize KF
                self._kf = KalmanFilter(
                    transition_matrices=self._transition_matrix,
                    observation_matrices=self._observation_matrix,
                    initial_state_mean=self._initial_state_mean
                )
                self._kf = self._kf.em(self._measurements, n_iter=20)

                # First pass
                means, covariances = self._kf.filter(self._measurements)
                mean, covariance = means[-1], covariances[-1]
            else:
                assert self._previous_mean is not None
                assert self._previous_covariance is not None
                mean, covariance = self._kf.filter_update(self._previous_mean, self._previous_covariance,
                                                          measurement.squeeze())

            assert mean is not None
            assert covariance is not None
            self._previous_mean = mean
            self._previous_covariance = covariance

            xyz_mean = mean[0::2]  # x, y, z - skip velocities
            xyz_sd = np.sqrt(np.diagonal(covariance)[0::2])  # x, y, z

            return xyz_mean, xyz_sd
