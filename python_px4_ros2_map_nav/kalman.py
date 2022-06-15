"""Kalman filter for producing a smooth position and variance estimates"""
import numpy as np
from typing import Optional, Tuple
from pykalman import KalmanFilter

from python_px4_ros2_map_nav.assertions import assert_type, assert_shape


# TODO: how to handle uneven timesteps?
class SimpleFilter(Filter):
    """Simple Kalman filter implementation

    3-dimensional location model with velocity taken into account with total of 6 (3x2) state variables
    """
    # TODO: min number of measurements before KF will output an estimate, current name is confusing, rename it?
    QUEUE_MAX_HEIGHT = 50
    """Default max height for measurement queue"""

    def __init__(self, window_length=QUEUE_MAX_HEIGHT):
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
            self._measurements[0][0],  # x
            0,  # x_vel
            self._measurements[0][1],  # y
            0,  # y_vel
            self._measurements[0][2],  # z
            0  # z_vel
        ])

    def update(self, measurement: np.ndarray) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Returns a filtered position with estimated standard deviations

        Pushes the measurement into the queue and removes the oldest one if queue is full.

        :param position: A new position observation (measurement)
        :return: Filtered position, AMSL altitude also gets filtered if provided, epx, epy and epz values provided of None if output not yet available
        """

        if self._measurements is None:
            self._init_initial_state(measurement)
            return None
        elif len(self._measurements) < self._window_length:
            # Store new measurement, not enough measurements yet
            self._measurements = np.vstack((self._measurements, measurement))
            return None
        else:
            # No need to maintain self._measurements, use online filtering (filter_update) instead
            self._measurements = self._measurements[len(self._measurements) - self.QUEUE_MAX_HEIGHT:]
            self._measurements = np.vstack((self._measurements, measurement))

            assert len(self._measurements) > 0  # If window length for some reason not sensible
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
                # Online update
                assert self._previous_mean is not None
                assert self._previous_covariance is not None
                self._kf = self._kf.em(self._measurements, n_iter=5)  # TODO: this needed?
                mean, covariance = self._kf.filter_update(self._previous_mean, self._previous_covariance,
                                                          measurement.squeeze())

            # Store latest mean & covariance for online filtering in the future
            assert mean is not None
            assert covariance is not None
            self._previous_mean = mean
            self._previous_covariance = covariance

            # TODO: fix unintuitive reverse order!
            # Create a new position instance with filtered values converted back to the original CRS
            xyz_mean = mean[0::2]  # x, y, z - skip velocities
            xyz_sd = np.sqrt(np.diagonal(covariance)[0::2])  # x, y, z

            assert_shape(xyz_mean, (3,))  # TODO: make shape adaptive, same shape as input, not hard-coded
            assert_shape(xyz_sd, (3,))
            return xyz_mean, xyz_sd



