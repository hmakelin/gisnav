"""Module that contains the autopilot middleware (MAVROS) adapter ROS 2 node."""
from typing import Optional

import numpy as np
from geographic_msgs.msg import GeoPoint, GeoPointStamped, GeoPose, GeoPoseStamped
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import Altitude, HomePosition, MountControl
from rclpy.qos import QoSPresetProfiles
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32

from . import messaging
from .base.base_node import BaseNode


class AutopilotNode(BaseNode):
    """ROS 2 node that acts as an adapter for MAVROS"""

    ROS_PARAM_DEFAULTS = []
    """List containing ROS parameter name, default value and read_only flag tuples"""

    def __init__(self, name: str) -> None:
        """Initializes the ROS 2 node

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

        self._vehicle_nav_sat_fix = None
        self._vehicle_nav_sat_fix_sub = self.create_subscription(
            NavSatFix,
            "/mavros/global_position/global",
            self._vehicle_nav_sat_fix_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self._vehicle_pose_stamped = None
        self._vehicle_pose_stamped_sub = self.create_subscription(
            PoseStamped,
            "/mavros/local_position/pose",
            self._vehicle_pose_stamped_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self._home_position = None
        self._home_position_sub = self.create_subscription(
            HomePosition,
            "/mavros/home_position/home",
            self._home_position_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self._mount_control = None
        self._mount_control_sub = self.create_subscription(
            MountControl,
            "/mavros/mount_control/command",
            self._mount_control_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

    # region ROS subscriber callbacks
    def _vehicle_nav_sat_fix_callback(self, msg: NavSatFix) -> None:
        """Handles latest :class:`mavros_msgs.msg.NavSatFix` message

        Calls :meth:`.publish_vehicle_geopose` and :meth:`.publish_vehicle_altitude`
        because the contents of those messages are affected by an updated
        :class:`mavros_msgs.msg.NavSatFix` message.

        :param msg: :class:`mavros_msgs.msg.NavSatFix` message from MAVROS
        """
        self._vehicle_nav_sat_fix = msg
        self.publish_vehicle_geopose()
        self.publish_vehicle_altitude()
        # TODO: temporarily assuming static camera so publishing gimbal quat here
        self.publish_gimbal_quaternion()

    def _vehicle_pose_stamped_callback(self, msg: PoseStamped) -> None:
        """Handles latest :class:`mavros_msgs.msg.PoseStamped` message

        Calls :meth:`.publish_vehicle_geopose` because the content of that
        message is affected by an updated :class:`mavros_msgs.msg.PoseStamped` message.

        :param msg: :class:`mavros_msgs.msg.PoseStamped` message from MAVROS
        """
        self._vehicle_pose_stamped = msg
        self.publish_vehicle_geopose()
        # self.publish_vehicle_altitude()  # Needed? This is mainly about vehicle pose

    def _home_position_callback(self, msg: HomePosition) -> None:
        """Handles latest :class:`mavros_msgs.msg.HomePosition` message

        Calls :meth:`.publish_home_geopoint` because the content of that message is
        affected by an updated :class:`mavros_msgs.msg.HomePosition` message.

        :param msg: :class:`mavros_msgs.msg.HomePosition` message from MAVROS
        """
        self._home_position = msg
        self.publish_home_geopoint()

    def _mount_control_callback(self, msg: MountControl) -> None:
        """Handles latest :class:`mavros_msgs.msg.MountControl` message

        Calls :meth:`.publish_gimbal_quaternion` because the content of that
        message is affected by an updated :class:`mavros_msgs.msg.MountControl` message.

        :param msg: :class:`mavros_msgs.msg.MountControl` message from MAVROS
        """
        self._mount_control = msg
        self.publish_gimbal_quaternion()

    # endregion ROS subscriber callbacks

    # region computed attributes
    @property
    def vehicle_geopose(self) -> Optional[GeoPoseStamped]:
        """Vehicle pose as :class:`geographic_msgs.msg.GeoPoseStamped` message
        or None if not available"""
        if (
            self._vehicle_nav_sat_fix is not None
            and self._vehicle_pose_stamped is not None
        ):
            # Position
            latitude, longitude = (
                self._vehicle_nav_sat_fix.latitude,
                self._vehicle_nav_sat_fix.longitude,
            )
            altitude = self._vehicle_nav_sat_fix.altitude

            # Convert ENU->NED + re-center yaw
            enu_to_ned = Rotation.from_euler("XYZ", np.array([np.pi, 0, np.pi / 2]))
            attitude_ned = (
                Rotation.from_quat(
                    messaging.as_np_quaternion(
                        self._vehicle_pose_stamped.pose.orientation
                    )
                )
                * enu_to_ned.inv()
            )
            rpy = attitude_ned.as_euler("XYZ", degrees=True)
            rpy[0] = (rpy[0] + 180) % 360
            attitude_ned = Rotation.from_euler("XYZ", rpy, degrees=True)
            attitude_ned = attitude_ned.as_quat()
            orientation = messaging.as_ros_quaternion(attitude_ned)

            return GeoPoseStamped(
                header=messaging.create_header("base_link"),
                pose=GeoPose(
                    position=GeoPoint(
                        latitude=latitude, longitude=longitude, altitude=altitude
                    ),
                    orientation=orientation,
                ),
            )
        else:
            self.get_logger().warn(
                "NavSatFix and/or PoseStamped message not yet received, cannot "
                "determine vehicle geopose."
            )
            return None

    @property
    def _vehicle_altitude_local(self) -> Optional[float]:
        """Returns z coordinate from :class:`sensor_msgs.msg.PoseStamped` message
        or None if not available"""
        if self._vehicle_pose_stamped is not None:
            return self._vehicle_pose_stamped.pose.position.z
        else:
            self.get_logger().warn(
                "VehicleLocalPosition message not yet received, cannot determine "
                "vehicle local altitude."
            )
            return None

    @property
    def vehicle_altitude(self) -> Optional[Altitude]:
        """Vehicle altitude as :class:`mavros_msgs.msg.Altitude` message or
        None if not available"""
        if (
            self._vehicle_nav_sat_fix is not None
            and self.egm96_height is not None
            and self.terrain_altitude is not None
        ):
            vehicle_altitude_amsl = (
                self._vehicle_nav_sat_fix.altitude - self.egm96_height.data
            )
            vehicle_altitude_terrain = (
                vehicle_altitude_amsl - self.terrain_altitude.amsl
            )
            local = (
                self._vehicle_altitude_local
                if self._vehicle_altitude_local is not None
                else np.nan
            )
            altitude = Altitude(
                header=messaging.create_header("base_link"),
                amsl=vehicle_altitude_amsl,
                local=local,  # TODO: home altitude ok?
                relative=-local,
                terrain=vehicle_altitude_terrain,
                bottom_clearance=np.nan,
            )
            return altitude
        else:
            self.get_logger().warn(
                f"NavSatFix {self._vehicle_nav_sat_fix} and/or terrain Altitude "
                f"{self.terrain_altitude} and/or EGM 96 height {self.egm96_height } "
                f"message not yet received, cannot determine vehicle altitude."
            )
            return None

    @property
    def gimbal_quaternion(self) -> Optional[Quaternion]:
        """Gimbal orientation as :class:`geometry_msgs.msg.Quaternion` message
        or None if not available

        .. note::
            Current implementation assumes camera is facing directly down from
            vehicle body
        """
        # TODO: assumes static nadir facing camera, do proper implementation
        if self.vehicle_geopose is not None:
            vehicle_attitude = Rotation.from_quat(
                messaging.as_np_quaternion(self.vehicle_geopose.pose.orientation)
            )
            gimbal_rpy = vehicle_attitude.as_euler("xyz", degrees=True)
            gimbal_rpy[1] -= 90
            gimbal_attitude = Rotation.from_euler("xyz", gimbal_rpy, degrees=True)
            return messaging.as_ros_quaternion(gimbal_attitude.as_quat())
        else:
            self.get_logger().warn(
                "Vehicle GeoPose unknown, cannot determine gimbal quaternion "
                "for static camera."
            )
            return None

    @property
    def home_geopoint(self) -> Optional[GeoPointStamped]:
        """Home position as :class:`geographic_msgs.msg.GeoPointStamped` message
        or None if not available"""
        if self._home_position is not None:
            return GeoPointStamped(
                header=messaging.create_header("base_link"),
                position=GeoPoint(
                    latitude=self._home_position.geo.latitude,
                    longitude=self._home_position.geo.longitude,
                    altitude=self._home_position.geo.altitude,
                ),
            )
        else:
            self.get_logger().warn(
                "HomePosition message not yet received, cannot determine home geopoint."
            )
            return None

    # endregion computed attributes

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
    def publish_vehicle_geopose(self) -> None:
        """Publishes vehicle :class:`geographic_msgs.msg.GeoPoseStamped`"""
        if self.vehicle_geopose is not None:
            self.__vehicle_geopose_pub.publish(self.vehicle_geopose)
        else:
            self.get_logger().warn("Vehicle geopose unknown, skipping publishing.")

    def publish_vehicle_altitude(self) -> None:
        """Publishes vehicle :class:`mavros_msgs.msg.Altitude`"""
        if self.vehicle_altitude is not None:
            self.__vehicle_altitude_pub.publish(self.vehicle_altitude)
        else:
            self.get_logger().warn("Vehicle altitude unknown, skipping publishing.")

    def publish_gimbal_quaternion(self) -> None:
        """Publishes gimbal :class:`geometry_msgs.msg.Quaternion` orientation"""
        # TODO: NED or ENU? ROS convention is ENU but current implementation is NED?
        if self.gimbal_quaternion is not None:
            self.__gimbal_quaternion_pub.publish(self.gimbal_quaternion)
        else:
            self.get_logger().warn("Gimbal quaternion unknown, skipping publishing.")

    def publish_home_geopoint(self) -> None:
        """Publishes home :class:`.geographic_msgs.msg.GeoPointStamped`"""
        if self.home_geopoint is not None:
            self.__home_geopoint_pub.publish(self.home_geopoint)
        else:
            self.get_logger().warn("Home geopoint unknown, skipping publishing.")

    # endregion publish hooks
