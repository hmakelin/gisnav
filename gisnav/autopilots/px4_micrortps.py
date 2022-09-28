"""Adapter for PX4-ROS 2 bridge"""
import rclpy
import time
import numpy as np
from typing import Optional, Tuple, Callable, get_args

from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, VehicleGlobalPosition, GimbalDeviceAttitudeStatus, \
    GimbalDeviceSetAttitude

from gisnav.assertions import assert_type
from gisnav.data import TimePair, Attitude
from gisnav.geo import GeoPoint

from gisnav.autopilots.autopilot import Autopilot


class PX4microRTPS(Autopilot):
    """PX4-ROS 2 microRTPS bridge adapter"""

    ROS_TOPICS = {
        '/fmu/vehicle_attitude/out': {
            Autopilot._TOPICS_MSG_KEY: VehicleAttitude,
            Autopilot._TOPICS_QOS_KEY: rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        },
        '/fmu/vehicle_local_position/out': {
            Autopilot._TOPICS_MSG_KEY: VehicleLocalPosition,
            Autopilot._TOPICS_QOS_KEY: rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        },
        '/fmu/vehicle_global_position/out': {
            Autopilot._TOPICS_MSG_KEY: VehicleGlobalPosition,
            Autopilot._TOPICS_QOS_KEY: rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        },
        '/fmu/gimbal_device_set_attitude/out': {
            Autopilot._TOPICS_MSG_KEY: GimbalDeviceSetAttitude,
            Autopilot._TOPICS_QOS_KEY: rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        }
    }
    """ROS topics to subscribe"""

    def __init__(self, node: rclpy.node.Node, camera_info_topic: str, image_topic: str,
                 image_callback: Callable[[Image], None]) -> None:
        """Initialize class

        :param node: Parent node that handles the ROS subscriptions
        :param camera_info_topic: ROS camera info topic to subscribe to
        :param image_topic: ROS camera image (raw) topic to subscribe to
        :param image_callback: Callback function for camera image
        """
        super().__init__(node, camera_info_topic, image_topic, image_callback)

        self._setup_subscribers()

        self._vehicle_local_position = None
        self._vehicle_global_position = None
        self._vehicle_attitude = None
        self._gimbal_device_set_attitude = None
        self._gimbal_attitude = None
        self._time_sync = None

    # region Properties
    @property
    def _vehicle_local_position(self) -> Optional[VehicleLocalPosition]:
        """VehicleLocalPosition received via the PX4-ROS 2 bridge."""
        return self.__vehicle_local_position

    @_vehicle_local_position.setter
    def _vehicle_local_position(self, value: Optional[VehicleLocalPosition]) -> None:
        assert_type(value, get_args(Optional[VehicleLocalPosition]))
        self.__vehicle_local_position = value

    @property
    def _vehicle_attitude(self) -> Optional[VehicleAttitude]:
        """VehicleAttitude received via the PX4-ROS 2 bridge."""
        return self.__vehicle_attitude

    @_vehicle_attitude.setter
    def _vehicle_attitude(self, value: Optional[VehicleAttitude]) -> None:
        assert_type(value, get_args(Optional[VehicleAttitude]))
        self.__vehicle_attitude = value

    @property
    def _vehicle_global_position(self) -> Optional[VehicleGlobalPosition]:
        """VehicleGlobalPosition received via the PX4-ROS 2 bridge."""
        return self.__vehicle_global_position

    @_vehicle_global_position.setter
    def _vehicle_global_position(self, value: Optional[VehicleGlobalPosition]) -> None:
        assert_type(value, get_args(Optional[VehicleGlobalPosition]))
        self.__vehicle_global_position = value

    @property
    def _gimbal_device_set_attitude(self) -> Optional[GimbalDeviceSetAttitude]:
        """GimbalDeviceSetAttitude received via the PX4-ROS 2 bridge."""
        return self.__gimbal_device_set_attitude

    @_gimbal_device_set_attitude.setter
    def _gimbal_device_set_attitude(self, value: Optional[GimbalDeviceSetAttitude]) -> None:
        assert_type(value, get_args(Optional[GimbalDeviceSetAttitude]))
        self.__gimbal_device_set_attitude = value

    @property
    def _time_sync(self) -> Optional[TimePair]:
        """A :class:`gisnav.data.TimePair` with local and foreign (EKF2) timestamps in microseconds

        The pair will contain the local system time and the foreign EKF2 time received from autopilot via ROS. The pair
        can then at any time be used to locally estimate the EKF2 system time.
        """
        return self.__time_sync

    @_time_sync.setter
    def _time_sync(self, value: Optional[TimePair]) -> None:
        assert_type(value, get_args(Optional[TimePair]))
        self.__time_sync = value
    # endregion

    # region Private
    def _sync_timestamps(self, ekf2_timestamp_usec: int) -> None:
        """Synchronizes local timestamp with PX4 EKF2's reference time

        This synchronization is triggered in the :meth:`._vehicle_local_position_callback` and therefore expected to be
        done at high frequency.

        .. seealso:
            :py:attr:`._time_sync`

        :param ekf2_timestamp_usec: Time since PX4 EKF2 system start in microseconds
        """
        assert_type(ekf2_timestamp_usec, int)
        now_usec = time.time() * 1e6
        self._time_sync = TimePair(now_usec, ekf2_timestamp_usec)

    def _setup_subscribers(self) -> None:
        """Creates and stores subscribers for microRTPS bridge topics"""
        for topic_name, d in self.ROS_TOPICS.items():
            class_ = d.get(Autopilot._TOPICS_MSG_KEY, None)
            assert class_ is not None, f'Message definition not provided for {topic_name}.'
            qos = d.get(Autopilot._TOPICS_QOS_KEY, rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value)
            callback_name = f'_{topic_name.split("/")[2].lower()}_callback'  # TODO make topic name parsing less brittle
            callback = getattr(self, callback_name, None)
            assert callback is not None, f'Missing callback implementation for {callback_name}.'
            self._subscribe(topic_name, class_, callback, qos)

    def _vehicle_local_position_callback(self, msg: VehicleLocalPosition) -> None:
        """Handles latest :class:`px4_msgs.msg.VehicleLocalPosition` message

        Uses the EKF2 system time in the message to synchronize local system time.

        :param msg: :class:`px4_msgs.msg.VehicleLocalPosition` message from the PX4-ROS 2 bridge
        :return:
        """
        assert_type(msg.timestamp, int)
        self._vehicle_local_position = msg
        self._sync_timestamps(self._vehicle_local_position.timestamp)

    def _vehicle_attitude_callback(self, msg: VehicleAttitude) -> None:
        """Handles latest :class:`px4_msgs.msg.VehicleAttitude` message

        :param msg: :class:`px4_msgs.msg.VehicleAttitude` message from the PX4-ROS 2 bridge
        :return:
        """
        self._vehicle_attitude = msg

    def _vehicle_global_position_callback(self, msg: VehicleGlobalPosition) -> None:
        """Handles latest :class:`px4_msgs.msg.VehicleGlobalPosition` message

        :param msg: :class:`px4_msgs.msg.VehicleGlobalPosition` message from the PX4-ROS 2 bridge
        :return:
        """
        self._vehicle_global_position = msg

    def _gimbal_device_set_attitude_callback(self, msg: GimbalDeviceSetAttitude) -> None:
        """Handles latest :class:`px4_msgs.msg.GimbalDeviceSetAttitude` message

        :param msg: :class:`px4_msgs.msg.GimbalDeviceSetAttitude` message from the PX4-ROS 2 bridge
        :return:
        """
        self._gimbal_device_set_attitude = msg

    def _is_expired(self, message) -> bool:
        """Returns True if the given message is older than :py:attr:`._TELEMETRY_EXPIRATION_LIMIT`

        .. note::
            Assumes message is expired if its age cannot be determined

        :return: True if the message is too old
        """
        if not hasattr(message, 'timestamp') or self.synchronized_time is None or \
                self.synchronized_time > message.timestamp + self.TELEMETRY_EXPIRATION_LIMIT:
            return True
        else:
            return False
    # endregion

    # region Public
    @property
    def synchronized_time(self) -> Optional[int]:
        """Estimated autopilot EKF2 system reference time in microseconds or None if not available

        .. seealso:
            :py:attr:`._time_sync` and :meth:`._sync_timestamps`

        :return: Estimated autpilot timestamp in usec or None if not available
        """
        if self._time_sync is None:
            return None
        else:
            now_usec = time.time() * 1e6
            assert now_usec > self._time_sync.local,\
                f'Current timestamp {now_usec} was unexpectedly smaller than timestamp stored earlier for ' \
                f'synchronization {self._time_sync.local}.'
            ekf2_timestamp_usec = int(self._time_sync.foreign + (now_usec - self._time_sync.local))
            return ekf2_timestamp_usec

    @property
    def attitude(self) -> Optional[Attitude]:
        """Vehicle attitude in NED frame as an :class:`.Attitude` instance"""
        if self._vehicle_attitude is not None and hasattr(self.__vehicle_attitude, 'q'):
            return Attitude(np.append(self._vehicle_attitude.q[1:4], self._vehicle_attitude.q[0]), extrinsic=True)
        else:
            return None

    @property
    def gimbal_attitude(self) -> Optional[np.ndarray]:
        """Gimbal attitude as an :class:`.Attitude` instance or None if not available"""
        return None

    @property
    def gimbal_set_attitude(self) -> Optional[np.ndarray]:
        """Gimbal set attitude in NED frame as an :class:`.Attitude` instance or None if not available

        .. note::
            * Assumes 2-axis (roll & pitch/tilt stabilized) gimbal
            * Gimbal actual attitude may not match the set attitude if gimbal has not stabilized itself
        """
        if self._vehicle_attitude is None or self._gimbal_device_set_attitude is None:
            return None

        yaw_mask = np.array([1, 0, 0, 1])  # Gimbal roll & pitch/tilt is assumed stabilized so only need yaw/pan
        vehicle_yaw = self._vehicle_attitude.q * yaw_mask

        gimbal_set_attitude_frd = self._gimbal_device_set_attitude.q

        # SciPy expects (x, y, z, w) while PX4 messages are (w, x, y, z)
        vehicle_yaw = Rotation.from_quat(np.append(vehicle_yaw[1:], vehicle_yaw[0]))
        gimbal_set_attitude_frd = Rotation.from_quat(np.append(gimbal_set_attitude_frd[1:], gimbal_set_attitude_frd[0]))

        gimbal_set_attitude_ned = vehicle_yaw * gimbal_set_attitude_frd

        return Attitude(gimbal_set_attitude_ned.as_quat(), extrinsic=True)

    @property
    def altitude_agl(self) -> Optional[float]:
        """Vehicle altitude in meters above ground level (AGL) or None if not available

        Tries to get distance to ground from :class:`.VehicleLocalPosition` ``dist_bottom`` and ``z`` attributes, and
        :class:`.VehicleGlobalPosition` ``terrain_alt`` attribute.
        """
        if self._vehicle_local_position is not None:
            if self._vehicle_local_position.dist_bottom_valid:
                return abs(self._vehicle_local_position.dist_bottom)
            elif self._vehicle_local_position.z_valid:
                return abs(self._vehicle_local_position.z)

        if self._vehicle_global_position is not None \
                and hasattr(self._vehicle_global_position, 'terrain_alt') \
                and hasattr(self._vehicle_global_position, 'terrain_alt_valid') \
                and self._vehicle_global_position.terrain_alt_valid:
            return self._vehicle_global_position.terrain_alt

        return None

    @property
    def altitude_amsl(self) -> Optional[float]:
        """Vehicle altitude in meters above mean sea level (AMSL) or None if not available

        .. seealso::
            :py:attr:`.altitude_amsl`
        """
        if self._vehicle_global_position is not None and hasattr(self._vehicle_global_position, 'alt'):
            return self._vehicle_global_position.alt
        else:
            return None

    @property
    def ground_elevation_amsl(self) -> Optional[float]:
        """Ground elevation in meters above mean sea level (AMSL) or None if information is not available

        .. note::
            Ground elevation assumed to be same as :class:`px4_msgs.VehicleLocalPosition` reference altitude.
            TODO: use DEM raster

        .. seealso::
            :py:attr:`.altitude_agl`
        """
        if self._vehicle_local_position is not None and hasattr(self._vehicle_local_position, 'ref_alt'):
            return self._vehicle_local_position.ref_alt
        else:
            return None

    @property
    def global_position(self) -> Optional[GeoPoint]:
        """Returns vehicle global position as a :class:`.GeoPoint`"""
        if self._vehicle_global_position is not None and hasattr(self._vehicle_global_position, 'lat') \
                and hasattr(self._vehicle_global_position, 'lon'):
            return GeoPoint(self._vehicle_global_position.lon, self._vehicle_global_position.lat, crs='epsg:4326')
        else:
            return None
