"""Module with abstract base class for autopilot (PX4, Ardupilot etc.) adapters"""
import numpy as np
import rclpy

from abc import ABC, abstractmethod
from typing import Optional, Callable, get_args

from sensor_msgs.msg import Image, CameraInfo

from gisnav.data import Attitude, CameraData, ImageData, Dim
from gisnav.geo import GeoPoint
from gisnav.assertions import assert_type


class Autopilot(ABC):
    """Abstract base class definining autopilot bridge interface"""

    # Keys for topics dictionary that map microRTPS bridge topics to subscribers and message definitions
    _TOPICS_MSG_KEY = 'message'
    _TOPICS_SUBSCRIBER_KEY = 'subscriber'
    _TOPICS_QOS_KEY = 'qos'

    # TODO: use this for all properties (make wrapper)
    TELEMETRY_EXPIRATION_LIMIT = 1e6
    """Expiration period in usec for vehicle state (if telemetry data is older than this it should not be used)"""

    CAMERA_INFO_TOPIC = 'camera/camera_info'
    """ROS camera info (:class:`.sensor_msgs.msg.CameraInfo`) topic to subscribe to"""

    IMAGE_RAW_TOPIC = 'camera/image_raw'
    """ROS image raw (:class:`.sensor_msgs.msg.Image`) topic to subscribe to"""

    def __init__(self, node: rclpy.node.Node, on_image_callback: Callable[[ImageData], None]) -> None:
        """Initializes autopilot adapter

        :param node: Parent node that handles the ROS subscriptions
        :param on_image_callback: Callback function for camera image
        """
        self._node = node
        self._topics = {}

        # Subscribe to camera data topic
        self.camera_data = None
        self._subscribe(self.CAMERA_INFO_TOPIC, CameraInfo, self._camera_info_callback,
                        rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)

        # Image topic callback gets access to base node through provided on_image_callback
        self._subscribe(self.IMAGE_RAW_TOPIC, Image, on_image_callback, rclpy.qos.QoSPresetProfiles.SENSOR_DATA.value)

    @property
    def _node(self) -> rclpy.node.Node:
        """ROS node that handles the subscriptions (and holds a handle to this adapter)"""
        return self.__node

    @_node.setter
    def _node(self, value: rclpy.node.Node) -> None:
        assert_type(value, rclpy.node.Node)
        self.__node = value

    @property
    def _topics(self) -> dict:
        """Dictionary that stores all rclpy subscribers."""
        return self.__topics

    @_topics.setter
    def _topics(self, value: dict) -> None:
        assert_type(value, dict)
        self.__topics = value

    @property
    def camera_data(self) -> Optional[CameraData]:
        """CameraInfo received via the PX4-ROS 2 bridge as a :class:`.CameraData` instance."""
        return self.__camera_data

    @camera_data.setter
    def camera_data(self, value: Optional[CameraData]) -> None:
        assert_type(value, get_args(Optional[CameraData]))
        self.__camera_data = value

    def _subscribe(self, topic_name: str, class_: type, callback: Callable, qos: rclpy.qos.QoSProfile) -> None:
        """Subscribes to ROS topic

        :param topic_name: Name of the microRTPS topic
        :param class_: Message definition class type (e.g. px4_msgs.msg.VehicleLocalPosition)
        :param callback: Callback function for the topic
        :param qos: Subscription quality of service profile
        :return: The subscriber instance
        """
        self._topics.update({topic_name: {self._TOPICS_SUBSCRIBER_KEY:
                                              self._node.create_subscription(class_, topic_name, callback, qos)}})

    def _unsubscribe(self, topic_name: str):
        """Unsubscribes from ROS topic

        :param topic_name: Name of ROS topic to unsubscribe
        """
        for topic_name, v in self._topics.items():
            subscriber = v.get(topic_name, {}).get(self._TOPICS_SUBSCRIBER_KEY, None)
            if subscriber is not None:
                subscriber.destroy()

    def unsubscribe_all(self):
        """Destroys all subscribers"""
        for k, _ in self._topics.items():
            self._unsubscribe(k)

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        """Handles latest :class:`px4_msgs.msg.CameraInfo` message

        .. note::
            Checks that intrisic matrix and height/width information is received and then destroys subscription (
            assumes camera info is static)

        :param msg: :class:`px4_msgs.msg.CameraInfo` message from the PX4-ROS 2 bridge
        """
        if not all(hasattr(msg, attr) for attr in ['k', 'height', 'width']):
            # TODO: check that k and height/width match
            return None
        else:
            self.camera_data = CameraData(k=msg.k.reshape((3, 3)), dim=Dim(msg.height, msg.width))
            camera_info_sub = self._topics.get(self.CAMERA_INFO_TOPIC, {}).get(self._TOPICS_SUBSCRIBER_KEY, None)
            if camera_info_sub is not None:
                # Assume camera info is static, destroy subscription
                camera_info_sub.destroy()

    @property
    @abstractmethod
    def synchronized_time(self) -> Optional[None]:
        """Estimated foreign (autopilot) timestamp in microseconds or None if not available"""
        pass

    @property
    @abstractmethod
    def attitude(self) -> Optional[Attitude]:
        """Vehicle attitude as an :class:`.Attitude` instance or None if not available"""
        pass

    @property
    @abstractmethod
    def altitude_agl(self) -> Optional[float]:
        """Vehicle altitude in meters above ground level (AGL) or None if not available

        .. seealso::
            :py:attr:`.altitude_amsl`, :py:attr:`.altitude_ellipsoid`
        """
        pass

    @property
    @abstractmethod
    def altitude_amsl(self) -> Optional[float]:
        """Vehicle altitude in meters above mean sea level (AMSL) or None if not available

        .. seealso::
            :py:attr:`.altitude_agl`, :py:attr:`.altitude_ellipsoid`
        """
        pass

    @property
    @abstractmethod
    def altitude_ellipsoid(self) -> Optional[float]:
        """Vehicle altitude in meters above WGS 84 ellipsoid or None if not available

        .. seealso::
            :py:attr:`.altitude_amsl`, :py:attr:`.altitude_agl`
        """
        pass

    @property
    @abstractmethod
    def ground_elevation_amsl(self) -> Optional[float]:
        """Ground elevation in meters above mean sea level (AMSL) or None if information is not available"""
        pass

    @property
    @abstractmethod
    def ground_elevation_ellipsoid(self) -> Optional[float]:
        """Ground elevation in meters above WGS 84 ellipsoid or None if information is not available"""
        pass

    @property
    @abstractmethod
    def gimbal_attitude(self) -> Optional[np.ndarray]:
        """Gimbal attitude quaternion in (x, y, z, w) format in NED frame or None if not available

        .. note::
            This is the same format that for example SciPy uses, while e.g. the PX4-ROS 2 bridge uses (w, x, y, z)
        """
        pass

    @property
    @abstractmethod
    def gimbal_set_attitude(self) -> Optional[np.ndarray]:
        """Gimbal set attitude quaternion in (x, y, z, w) format in NED frame or None if not available

        .. note::
            * This is the same format that for example SciPy uses, while e.g. the PX4-ROS 2 bridge uses (w, x, y, z)
            * Gimbal actual attitude may not match the set attitude if e.g. gimbal has not yet stabilized itself
            """
        pass

    @property
    @abstractmethod
    def global_position(self) -> Optional[GeoPoint]:
        """Vehicle global position as a :class:`.GeoPoint`"""
        pass

