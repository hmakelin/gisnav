"""Module that contains the autopilot middleware (MAVROS) adapter ROS 2 node."""
import math
from typing import Optional

import numpy as np
from geographic_msgs.msg import GeoPoint, GeoPointStamped, GeoPose, GeoPoseStamped
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import Altitude, GimbalDeviceAttitudeStatus, HomePosition
from rclpy.qos import QoSPresetProfiles
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32

from gisnav.assertions import enforce_types

from . import messaging
from .base.rviz_publisher_node import RVizPublisherNode


class AutopilotNode(RVizPublisherNode):
    """ROS 2 node that acts as an adapter for MAVROS"""

    ROS_PARAM_DEFAULTS: list = []
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
        self._gimbal_device_attitude_status = None
        self._gimbal_device_attitude_status_sub = self.create_subscription(
            GimbalDeviceAttitudeStatus,
            "/mavros/gimbal_control/device/attitude_status",
            self._gimbal_device_attitude_status_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

    @staticmethod
    def _navsatfix_to_geoposestamped(msg: NavSatFix) -> GeoPoseStamped:
        # Publish to rviz2 for debugging
        geopose_stamped = GeoPoseStamped()
        geopose_stamped.header.stamp = msg.header.stamp
        geopose_stamped.header.frame_id = "map"

        geopose_stamped.pose.position.longitude = msg.longitude
        geopose_stamped.pose.position.latitude = msg.latitude
        geopose_stamped.pose.position.altitude = msg.altitude

        # No orientation information in NavSatFix
        geopose_stamped.pose.orientation.w = 1.0

        return geopose_stamped

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

        if (
            self.vehicle_altitude is not None
            and self.vehicle_altitude.terrain is not np.nan
        ):
            # publish to RViz for debugging and visualization
            geopose_stamped = self._navsatfix_to_geoposestamped(msg)
            self.publish_rviz(geopose_stamped, self.vehicle_altitude.terrain)

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

    def _gimbal_device_attitude_status_callback(
        self, msg: GimbalDeviceAttitudeStatus
    ) -> None:
        """Handles latest :class:`mavros_msgs.msg.GimbalDeviceAttitudeStatus` message

        Calls :meth:`.publish_gimbal_quaternion` because the content of that
        message is affected by an updated :class:`mavros_msgs.msg.MountControl` message.

        :param msg: :class:`mavros_msgs.msg.MountControl` message from MAVROS
        """
        self._gimbal_device_attitude_status = msg
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
                    orientation=orientation,  # TODO: is this NED or ENU?
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

        @enforce_types(self.get_logger().warn, "Cannot determine vehicle altitude")
        def _vehicle_altitude(
            navsatfix: NavSatFix,
            egm96_height: Float32,
            terrain_altitude: Altitude,
            vehicle_altitude_local: Optional[Altitude],
        ):
            vehicle_altitude_amsl = navsatfix.altitude - egm96_height.data
            vehicle_altitude_terrain = vehicle_altitude_amsl - terrain_altitude.amsl
            local = (
                vehicle_altitude_local if vehicle_altitude_local is not None else np.nan
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

        return _vehicle_altitude(
            self._vehicle_nav_sat_fix,
            self.egm96_height,
            self.terrain_altitude,
            self._vehicle_altitude_local,
        )

        # else:
        #    self.get_logger().warn(
        #        "NavSatFix and/or terrain Altitude and/or EGM 96 height "
        #        "message not yet received, cannot determine vehicle altitude."
        #    )
        #    self.get_logger().debug(
        #        f"NavSatFix {self._vehicle_nav_sat_fix} and/or terrain Altitude "
        #        f"{self.terrain_altitude} and/or EGM 96 height {self.egm96_height } "
        #        f"message not yet received, cannot determine vehicle altitude."
        #    )
        #    return None

    @staticmethod
    def _euler_from_quaternion(q):
        # Convert quaternion to euler angles
        t0 = 2.0 * (q.w * q.x + q.y * q.z)
        t1 = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(t0, t1)

        t2 = 2.0 * (q.w * q.y - q.z * q.x)
        t2 = 1.0 if t2 > 1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    @staticmethod
    def yaw_from_quaternion(q):
        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw_rad = math.atan2(t3, t4)

        # Convert to degrees and normalize to [0, 360)
        yaw_deg = math.degrees(yaw_rad) % 360

        return yaw_deg

    @staticmethod
    def quaternion_multiply(q1, q2):
        w1, x1, y1, z1 = q1.w, q1.x, q1.y, q1.z
        w2, x2, y2, z2 = q2.w, q2.x, q2.y, q2.z

        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

        return Quaternion(w=w, x=x, y=y, z=z)

    def apply_vehicle_yaw(self, vehicle_q, gimbal_q):
        # Extract yaw from vehicle quaternion
        t3 = 2.0 * (vehicle_q.w * vehicle_q.z + vehicle_q.x * vehicle_q.y)
        t4 = 1.0 - 2.0 * (vehicle_q.y * vehicle_q.y + vehicle_q.z * vehicle_q.z)
        yaw_rad = math.atan2(t3, t4)

        # Create a new quaternion with only yaw rotation
        yaw_q = Quaternion(
            w=math.cos(yaw_rad / 2), x=0.0, y=0.0, z=math.sin(yaw_rad / 2)
        )

        # Apply the vehicle yaw rotation to the gimbal quaternion
        gimbal_yaw_q = self.quaternion_multiply(yaw_q, gimbal_q)

        return gimbal_yaw_q

    @property
    def gimbal_quaternion(self) -> Optional[Quaternion]:
        """Gimbal orientation as :class:`geometry_msgs.msg.Quaternion` message
        or None if not available

        .. note::
            Current implementation assumes camera is facing directly down from
            vehicle body
        """
        # TODO check frame (e.g. base_link_frd/vehicle body in PX4 SITL simulation)
        if self.vehicle_geopose is None:
            self.get_logger().debug(
                "Vehicle geopose unknown. Cannot determine gimbal attitude."
            )
            return None

        if self._gimbal_device_attitude_status is None:
            # Assume nadir-facing (roll and yaw are 0, pitch is -90 degrees)
            roll = 0
            pitch = -85  # do not make it -90 to avoid gimbal lock
            yaw = 0
            nadir_facing_rotation = Rotation.from_euler(
                "xyz", [roll, pitch, yaw], degrees=True
            )
            nadir_facing_quaternion = nadir_facing_rotation.as_quat()
            nadir_facing_quaternion = Quaternion(
                x=nadir_facing_quaternion[0],
                y=nadir_facing_quaternion[1],
                z=nadir_facing_quaternion[2],
                w=nadir_facing_quaternion[3],
            )
            gimbal_device_attitude_status = GimbalDeviceAttitudeStatus()
            gimbal_device_attitude_status.q = nadir_facing_quaternion
        else:
            gimbal_device_attitude_status = self._gimbal_device_attitude_status

        assert gimbal_device_attitude_status is not None
        assert self.vehicle_geopose is not None
        roll, pitch, yaw = self._euler_from_quaternion(gimbal_device_attitude_status.q)
        self.get_logger().debug(
            "Gimbal device set attitude FRD Euler angles "
            "(roll, pitch, yaw): (%.2f, %.2f, %.2f)"
            % (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))
        )

        compound_q = self.apply_vehicle_yaw(
            self.vehicle_geopose.pose.orientation, gimbal_device_attitude_status.q
        )
        roll, pitch, yaw = self._euler_from_quaternion(compound_q)
        self.get_logger().debug(
            "Compound attitude (NED?) Euler angles "
            "(roll, pitch, yaw): (%.2f, %.2f, %.2f)"
            % (math.degrees(roll), math.degrees(pitch), math.degrees(yaw))
        )

        return compound_q

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
