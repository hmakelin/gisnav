"""Adapter for ArduPilot MAVROS bridge"""
import rclpy
import time
import numpy as np
from typing import Optional, Callable, get_args

from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import MountControl

from gisnav.assertions import assert_type
from gisnav.data import TimePair, Attitude
from gisnav.geo import GeoPoint

from gisnav.autopilots.autopilot import Autopilot


class ArduPilotMAVROS(Autopilot):
    """ArduPilot MAVROS bridge adapter

    .. seealso::
        `MAVROS topic name to message definition mapping <https://wiki.ros.org/mavros>`_
    """

    ROS_TOPICS = {
        '/mavros/local_position/pose': {
            Autopilot._TOPICS_MSG_KEY: PoseStamped,
            Autopilot._TOPICS_QOS_KEY: rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        },
        '/mavros/global_position/global': {
            Autopilot._TOPICS_MSG_KEY: NavSatFix,
            Autopilot._TOPICS_QOS_KEY: rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        },
        '/mavros/mount_control/command': {
            Autopilot._TOPICS_MSG_KEY: MountControl,
            Autopilot._TOPICS_QOS_KEY: rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value
        },
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

        self._local_position_pose = None
        self._global_position_global = None
        self._mount_control_command = None

        self._time_sync = None

        self._setup_subscribers()

    # region Properties
    @property
    def _local_position_pose(self) -> Optional[PoseStamped]:
        """Local position pose received via MAVROS."""
        return self.__local_position_pose

    @_local_position_pose.setter
    def _local_position_pose(self, value: Optional[PoseStamped]) -> None:
        assert_type(value, get_args(Optional[PoseStamped]))
        self.__local_position_pose = value

    @property
    def _global_position_global(self) -> Optional[NavSatFix]:
        """Global positiion received via MAVROS."""
        return self.__global_position_global

    @_global_position_global.setter
    def _global_position_global(self, value: Optional[NavSatFix]) -> None:
        assert_type(value, get_args(Optional[NavSatFix]))
        self.__global_position_global = value

    @property
    def _mount_control_command(self) -> Optional[MountControl]:
        """Mount control command received via MAVROS."""
        return self.__mount_control_command

    @_mount_control_command.setter
    def _mount_control_command(self, value: Optional[MountControl]) -> None:
        assert_type(value, get_args(Optional[MountControl]))
        self.__mount_control_command = value

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

        This synchronization is triggered in the :meth:`._local_position_pose` and therefore expected to be done at
        high frequency.

        .. seealso:
            :py:attr:`._time_sync`

        :param ekf2_timestamp_usec: Time since foreign system start in microseconds
        """
        assert_type(ekf2_timestamp_usec, int)
        now_usec = time.time() * 1e6
        self._time_sync = TimePair(now_usec, ekf2_timestamp_usec)

    def _setup_subscribers(self) -> None:
        """Creates and stores subscribers ROS topics"""
        for topic_name, d in self.ROS_TOPICS.items():
            class_ = d.get(Autopilot._TOPICS_MSG_KEY, None)
            assert class_ is not None, f'Message definition not provided for {topic_name}.'
            qos = d.get(Autopilot._TOPICS_QOS_KEY, rclpy.qos.QoSPresetProfiles.SYSTEM_DEFAULT.value)
            callback_name = f'_{"_".join(topic_name.split("/")[-2:]).lower()}_callback'  # TODO make topic name parsing less brittle
            callback = getattr(self, callback_name, None)
            assert callback is not None, f'Missing callback implementation for {callback_name}.'
            self._subscribe(topic_name, class_, callback, qos)

    def _local_position_pose_callback(self, msg: PoseStamped) -> None:
        """Handles latest :class:`geometry_msgs.msg.PoseStamped` message

        Uses the EKF2 system time in the message to synchronize local system time.

        :param msg: :class:`class:`geometry_msgs.msg.PoseStamped` message via MAVROS
        :return:
        """
        assert hasattr(msg, 'header') and hasattr(msg.header, 'stamp') and hasattr(msg.header.stamp, 'sec') and \
            hasattr(msg.header.stamp, 'nanosec')
        self._local_position_pose = msg
        usec = msg.header.stamp.sec + int(msg.header.stamp.nanosec / 1e3)
        self._sync_timestamps(usec)

    def _global_position_global_callback(self, msg: NavSatFix) -> None:
        """Handles latest :class:`sensor_msgs.msg.NavSatFix` message

        :param msg: :class:`sensor_msgs.msg.NavSatFix` message via MAVROS
        :return:
        """
        self._global_position_global = msg

    def _mount_control_command_callback(self, msg: MountControl) -> None:
        """Handles latest :class:`mavros_msgs.msg.MountControl` message

        :param msg: :class:`mavros_msgs.msg.MountControl` via MAVROS
        :return:
        """
        self._mount_control_command = msg

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
        if self._local_position_pose is not None \
                and hasattr(self._local_position_pose, 'pose') \
                and hasattr(self._local_position_pose.pose, 'orientation') \
                and hasattr(self._local_position_pose.pose.orientation, 'x') \
                and hasattr(self._local_position_pose.pose.orientation, 'y') \
                and hasattr(self._local_position_pose.pose.orientation, 'z') \
                and hasattr(self._local_position_pose.pose.orientation, 'w'):
            return Attitude(np.array([self._local_position_pose.pose.orientation.x,
                                      self._local_position_pose.pose.orientation.y,
                                      self._local_position_pose.pose.orientation.z,
                                      self._local_position_pose.pose.orientation.w]), extrinsic=True)
        else:
            return None

    @property
    def gimbal_attitude(self) -> Optional[np.ndarray]:
        """Gimbal attitude as an :class:`.Attitude` instance or None if not available"""
        # TODO
        return None

    @property
    def gimbal_set_attitude(self) -> Optional[np.ndarray]:
        """Gimbal set attitude in NED frame as an :class:`.Attitude` instance or None if not available

        .. note::
            * Assumes 2-axis (roll & pitch/tilt stabilized) gimbal
            * Gimbal actual attitude may not match the set attitude if gimbal has not stabilized itself
        """
        return None

    @property
    def altitude_agl(self) -> Optional[float]:
        """Vehicle altitude in meters above ground level (AGL) or None if not available

        .. note::
            Assumes local position reference altitude is ground level
        """
        if self._local_position_pose is not None and hasattr(self._local_position_pose, 'pose') and \
                hasattr(self._local_position_pose.pose, 'position') and \
                hasattr(self._local_position_pose.pose.position, 'z'):
            return self._local_position_pose.pose.position.z

        return None

    @property
    def altitude_amsl(self) -> Optional[float]:
        """Vehicle altitude in meters above mean sea level (AMSL) or None if not available

        .. note::
            Assumes MAVROS global position altitude above ellipsoid :~ altitude AMSL

        .. seealso::
            :py:attr:`.altitude_amsl`
        """
        if self._global_position_global is not None and hasattr(self._global_position_global, 'altitude'):
            return self._global_position_global.altitude
        else:
            return None

    @property
    def ground_elevation_amsl(self) -> Optional[float]:
        """Ground elevation in meters above mean sea level (AMSL) or None if information is not available

        .. note::
            Ground elevation assumed to be same as local position reference altitude.
            TODO: use DEM raster

        .. seealso::
            :py:attr:`.altitude_agl`
        """
        if self.altitude_amsl is not None and self.altitude_agl is not None:
            return self.altitude_amsl - self.altitude_agl
        else:
            return None

    @property
    def global_position(self) -> Optional[GeoPoint]:
        """Returns vehicle global position as a :class:`.GeoPoint`"""
        if self._global_position_global is not None and hasattr(self._global_position_global, 'latitude') \
                and hasattr(self._global_position_global, 'longitude'):
            return GeoPoint(self._global_position_global.longitude, self._global_position_global.latitude,
                            crs='epsg:4326')
        else:
            return None
