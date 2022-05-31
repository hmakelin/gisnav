"""Kalman filter for producing a smooth position and variance estimates"""
import numpy as np
from pykalman import KalmanFilter


# TODO: how to handle uneven timesteps?
class SimpleFilter:
    """Simple Kalman filter implementation

    3-dimensional location model with velocity taken into account with total of 6 (3x2) state variables
    """

    OBS_DIM = 6
    """Observation dimensions := 6 (x, x_vel, y, y_vel, z, z_vel)"""

    # TODO: min number of measurements before KF will output an estimate, current name is confusing, rename it?
    QUEUE_MAX_HEIGHT = 50
    """Default max height for measurement queue"""

    def __init__(self, window_length=QUEUE_MAX_HEIGHT):
        self._measurements = None
        self._initial_state_mean = np.array([
            measurements[0, 0],  # x
            0,                   # x_vel
            measurements[0, 1],  # y
            0,                   # y_vel
            measurements[0, 1],  # z
            0                    # z_vel
        ])
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

    def filter(self, measurement: np.ndarray) -> Optional[Tuple[2*(np.ndarray,)]]:
        """Returns the filtered state means and covariances if enough measurements provided

        Pushes the measurement into the queue and removes the oldest one if queue is full.

        :param measurement: A new measurement
        :return: Tuple of filtered state means and covariances, or None if not yet available
        """
        # TODO: assert measurements dimensions is 6
        if self._measurements is None:
            self._measurements = measurement
        elif len(self.measurements) < self._window_length:
            # Store new measurement, not enough measurements yet
            self._measurements = np.vstack(self._measurements, measurement)
            return None
        else:
            # No need to maintain self._measurements, use online filtering (filter_update) instead
            # self._measurements = self._measurements[len(self._measurements) - QUEUE_MAX_HEIGHT:]
            # self._measurements = np.vstack((self._measurements, measurement))

            assert len(self._measurements) > 0  # If window length for some reason not sensible
            if self._kf is None:
                # Initialize KF
                self._kf = KalmanFilter(
                    transition_matrices=self._transition_matrix,
                    observation_matrices=self._observation_matrix,
                    initial_state_mean=self._initial_state_mean
                )
                self._kf = self._kf.em(self._measurements, n_iter=5)

                # First pass
                means, covariances = self._kf.filter(self._measurements)
                mean, covariance = means[-1], covariances[-1]
            else:
                # Online update
                assert self._previous_mean is not None
                assert self._previous_covariance is not None
                mean, covariance = self._kf.filter_update(self._previous_mean, self._previous_covariance,
                                                          self.measurement)

            # Store latest mean & covariance for online filtering in the future
            self._previous_mean = mean
            self._previous_covariance = covariance
            return self._previous_mean, self._previous_covariance




