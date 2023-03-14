"""Module that contains the AutopilotNode abstract base class ROS 2 node."""
from abc import ABC, abstractmethod
from typing import Optional

from geographic_msgs.msg import GeoPointStamped, GeoPoseStamped
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import Altitude
from rclpy.qos import QoSPresetProfiles
from std_msgs.msg import Float32

from gisnav.nodes import messaging
from gisnav.nodes.base.base_node import BaseNode


class AutopilotNode(BaseNode, ABC):
    """A ROS 2 abstract base class for nodes that provide a stable internal
    interface to autopilot telemetry

    .. note::
        This abstract base class is intended for package internal use only.
        It should be extended by nodes that adapt it to the ROS topics provided
        by any given autopilot platform, for example PX4 or ArduPilot.
    """

    def __init__(self, name: str) -> None:
        """Initializes the ROS 2 node.

        :param name: Name of the node
        """
        super().__init__(name)

        # Publishers
        # Use name mangling to protect these from being overwritten by extending
        # classes
        self.__vehicle_geopose_pub = self.create_publisher(
            GeoPoseStamped,
            messaging.ROS_TOPIC_VEHICLE_GEOPOSE,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self.__vehicle_altitude_pub = self.create_publisher(
            Altitude,
            messaging.ROS_TOPIC_VEHICLE_ALTITUDE,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self.__gimbal_quaternion_pub = self.create_publisher(
            Quaternion,
            messaging.ROS_TOPIC_GIMBAL_QUATERNION,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self.__home_geopoint_pub = self.create_publisher(
            GeoPointStamped,
            messaging.ROS_TOPIC_HOME_GEOPOINT,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

        # Subscribers
        # terrain_altitude and egm96_height properties intended to be used by
        # extending classes -> no name mangling
        self.terrain_altitude = None
        self.__terrain_altitude_sub = self.create_subscription(
            Altitude,
            messaging.ROS_TOPIC_TERRAIN_ALTITUDE,
            self.__terrain_altitude_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

        self.egm96_height = None
        self.__egm96_height_sub = self.create_subscription(
            Float32,
            messaging.ROS_TOPIC_EGM96_HEIGHT,
            self.__egm96_height_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

    # region public properties
    @property
    def terrain_altitude(self) -> Optional[Altitude]:
        """Terrain altitude

        Needed by implementing classes to generate vehicle
        :class:`geographic_msgs.msg.GeoPoseStamped` message
        """
        return self.__terrain_altitude

    @terrain_altitude.setter
    def terrain_altitude(self, value: Optional[Altitude]) -> None:
        self.__terrain_altitude = value

    @property
    def egm96_height(self) -> Optional[Float32]:
        """EGM96 geoid height

        Needed by implementing classes to generate vehicle
        :class:`geographic_msgs.msg.GeoPoseStamped` and
        :class:`mavros_msgs.msg.Altitude` messages
        """
        return self.__egm96_height

    @egm96_height.setter
    def egm96_height(self, value: Optional[Float32]) -> None:
        self.__egm96_height = value

    # endregion public properties

    # region ROS subscriber callbacks
    def __terrain_altitude_callback(self, msg: Altitude) -> None:
        """Handles terrain altitude message"""
        self.__terrain_altitude = msg

    def __egm96_height_callback(self, msg: Float32) -> None:
        """Handles ellipsoid height message"""
        self.__egm96_height = msg

    # endregion ROS subscriber callbacks

    # region publish hooks
    @property
    @abstractmethod
    def vehicle_geopose(self) -> Optional[GeoPoseStamped]:
        """Vehicle pose as :class:`geographic_msgs.msg.GeoPoseStamped` message
        or None if not available"""

    def publish_vehicle_geopose(self) -> None:
        """Publishes vehicle :class:`geographic_msgs.msg.GeoPoseStamped`"""
        if self.vehicle_geopose is not None:
            self.__vehicle_geopose_pub.publish(self.vehicle_geopose)
        else:
            self.get_logger().warn("Vehicle geopose unknown, skipping publishing.")

    @property
    @abstractmethod
    def vehicle_altitude(self) -> Optional[Altitude]:
        """Vehicle altitude as :class:`mavros_msgs.msg.Altitude` message or
        None if not available"""

    def publish_vehicle_altitude(self) -> None:
        """Publishes vehicle :class:`mavros_msgs.msg.Altitude`"""
        if self.vehicle_altitude is not None:
            self.__vehicle_altitude_pub.publish(self.vehicle_altitude)
        else:
            self.get_logger().warn("Vehicle altitude unknown, skipping publishing.")

    @property
    @abstractmethod
    def gimbal_quaternion(self) -> Optional[Quaternion]:
        """Gimbal orientation as :class:`geometry_msgs.msg.Quaternion` message
        or None if not available"""

    def publish_gimbal_quaternion(self) -> None:
        """Publishes gimbal :class:`geometry_msgs.msg.Quaternion` orientation"""
        # TODO: NED or ENU? ROS convention is ENU but current implementation is NED?
        if self.gimbal_quaternion is not None:
            self.__gimbal_quaternion_pub.publish(self.gimbal_quaternion)
        else:
            self.get_logger().warn("Gimbal quaternion unknown, skipping publishing.")

    @property
    @abstractmethod
    def home_geopoint(self) -> Optional[GeoPointStamped]:
        """Home position as :class:`geographic_msgs.msg.GeoPointStamped` message
        or None if not available"""

    def publish_home_geopoint(self) -> None:
        """Publishes home :class:`.geographic_msgs.msg.GeoPointStamped`"""
        if self.home_geopoint is not None:
            self.__home_geopoint_pub.publish(self.home_geopoint)
        else:
            self.get_logger().warn("Home geopoint unknown, skipping publishing.")

    # endregion publish hooks
