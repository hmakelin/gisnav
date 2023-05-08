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

from gisnav.assertions import ROS, enforce_types

from . import messaging
from .base.rviz_publisher_node import RVizPublisherNode


class AutopilotNode(RVizPublisherNode):
    """ROS 2 node that acts as an adapter for MAVROS"""

    # TODO: remove once all nodes have @ROS.parameters decoration
    ROS_PARAM_DEFAULTS: list = []
    """List containing ROS parameter name, default value and read_only flag tuples"""

    #: Max delay for messages where updates are not needed nor expected often,
    #   e.g. home position
    _DELAY_SLOW_MS = 10000

    #: Max delay for things like global position
    _DELAY_NORMAL_MS = 2000

    #: Max delay for messages with fast dynamics that go "stale" quickly, e.g.
    #   local position and attitude. The delay can be a bit higher than is
    #   intuitive because the vehicle EKF should be able to fuse things with
    #   fast dynamics with higher lags as long as the timestamps are accurate.
    _DELAY_FAST_MS = 500

    def __init__(self, name: str) -> None:
        """Initializes the ROS 2 node

        :param name: Name of the node
        """
        super().__init__(name)

        # Calling these decorated properties the first time will setup
        # subscriptions to the appropriate ROS topics
        self.terrain_altitude
        self.egm96_height
        self.nav_sat_fix
        self.pose_stamped
        self.home_position

    @property
    @ROS.max_delay_ms(_DELAY_NORMAL_MS)
    @ROS.subscribe(
        messaging.ROS_TOPIC_TERRAIN_ALTITUDE, QoSPresetProfiles.SENSOR_DATA.value
    )
    def terrain_altitude(self) -> Optional[Altitude]:
        """Altitude of terrain directly under vehicle, or None if unknown or too old"""

    @property
    # @ROS.max_delay_ms(_DELAY_NORMAL_MS)  # Float32 does not have header
    @ROS.subscribe(
        messaging.ROS_TOPIC_EGM96_HEIGHT, QoSPresetProfiles.SENSOR_DATA.value
    )
    def egm96_height(self) -> Optional[Float32]:
        """
        Height in meters of EGM96 geoid at vehicle location, or None if unknown
        or too old
        """

    def nav_sat_fix_cb(self, msg: NavSatFix) -> None:
        """Callback for :class:`mavros_msgs.msg.NavSatFix` message

        Publishes vehicle :class:`.geographic_msgs.msg.GeoPoseStamped` and
        :class:`mavros_msgs.msg.Altitude` because the contents of those messages
        are affected by this update.

        :param msg: :class:`mavros_msgs.msg.NavSatFix` message from MAVROS
        """
        # TODO: temporarily assuming static camera so publishing gimbal quat here
        self.gimbal_quaternion

        @enforce_types(self.get_logger().warn, "Cannot publish to RViz")
        def _publish_rviz(geopose_stamped: GeoPoseStamped, altitude: Altitude):
            self.publish_rviz(geopose_stamped, altitude.terrain)

        _publish_rviz(self.geopose_stamped, self.altitude)

    @property
    @ROS.max_delay_ms(_DELAY_NORMAL_MS)
    @ROS.subscribe(
        "/mavros/global_position/global",
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=nav_sat_fix_cb,
    )
    def nav_sat_fix(self) -> Optional[NavSatFix]:
        """Vehicle GPS fix, or None if unknown or too old"""

    def pose_stamped_cb(self, msg: PoseStamped) -> None:
        """Callback for :class:`mavros_msgs.msg.PoseStamped` message

        Publishes :class:`.geographic_msgs.msg.GeoPoseStamped` because the
        content of that message is affected by this update.

        :param msg: :class:`mavros_msgs.msg.PoseStamped` message from MAVROS
        """
        self.geopose_stamped
        # self.publish_vehicle_altitude()  # Needed? This is mainly about vehicle pose

    @property
    @ROS.max_delay_ms(_DELAY_FAST_MS)
    @ROS.subscribe(
        "/mavros/local_position/pose",
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=pose_stamped_cb,
    )
    def pose_stamped(self) -> Optional[PoseStamped]:
        """Vehicle GPS fix, or None if unknown or too old"""

    def home_position_cb(self, msg: HomePosition) -> None:
        """Callback for :class:`mavros_msgs.msg.HomePosition` message

        Publishes home :class:`.geographic_msgs.msg.GeoPointStamped` because
        the content of that message is affected by this update.

        :param msg: :class:`mavros_msgs.msg.HomePosition` message from MAVROS
        """
        self.home_geopoint

    @property
    @ROS.max_delay_ms(_DELAY_SLOW_MS)
    @ROS.subscribe(
        "/mavros/home_position/home",
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=home_position_cb,
    )
    def home_position(self) -> Optional[HomePosition]:
        """Home position, or None if unknown or too old"""

    def gimbal_device_attitude_status_cb(self, msg: GimbalDeviceAttitudeStatus) -> None:
        """Callback for :class:`mavros_msgs.msg.GimbalDeviceAttitudeStatus` message

        Publishes gimbal :class:`.geometry_msgs.msg.Quaternion` because the
        content of that message is affected by this update.

        :param msg: :class:`mavros_msgs.msg.GimbalDeviceAttitudeStatus` message
            from MAVROS
        """
        self.gimbal_quaternion

    @property
    @ROS.max_delay_ms(_DELAY_FAST_MS)
    @ROS.subscribe(
        "/mavros/gimbal_control/device/attitude_status",
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=gimbal_device_attitude_status_cb,
    )
    def gimbal_device_attitude_status(self) -> Optional[GimbalDeviceAttitudeStatus]:
        """Gimbal attitude, or None if unknown or too old"""

    @property
    @ROS.publish(
        messaging.ROS_TOPIC_VEHICLE_ALTITUDE, QoSPresetProfiles.SENSOR_DATA.value
    )
    def altitude(self) -> Optional[Altitude]:
        """Altitude of vehicle, or None if unknown or too old"""

        @enforce_types(self.get_logger().warn, "Cannot determine vehicle altitude")
        def _altitude(
            nav_sat_fix: NavSatFix,
            egm96_height: Float32,
            terrain_altitude: Altitude,
            altitude_local: Optional[float],
        ):
            altitude_amsl = nav_sat_fix.altitude - egm96_height.data
            altitude_terrain = altitude_amsl - terrain_altitude.amsl
            local = altitude_local if altitude_local is not None else np.nan
            altitude = Altitude(
                header=messaging.create_header("base_link"),
                amsl=altitude_amsl,
                local=local,  # TODO: home altitude ok?
                relative=-local,  # TODO: check sign
                terrain=altitude_terrain,
                bottom_clearance=np.nan,
            )
            return altitude

        return _altitude(
            self.nav_sat_fix,
            self.egm96_height,
            self.terrain_altitude,
            self._altitude_local,
        )

    @property
    @ROS.publish(
        messaging.ROS_TOPIC_VEHICLE_GEOPOSE, QoSPresetProfiles.SENSOR_DATA.value
    )
    def geopose_stamped(self) -> Optional[GeoPoseStamped]:
        """Vehicle pose as :class:`geographic_msgs.msg.GeoPoseStamped` message
        or None if not available"""

        @enforce_types(self.get_logger().warn, "Cannot determine vehicle GeoPose")
        def _geopose_stamped(nav_sat_fix: NavSatFix, pose_stamped: PoseStamped):
            # Position
            latitude, longitude = (
                nav_sat_fix.latitude,
                nav_sat_fix.longitude,
            )
            altitude = nav_sat_fix.altitude

            # Convert ENU->NED + re-center yaw
            enu_to_ned = Rotation.from_euler("XYZ", np.array([np.pi, 0, np.pi / 2]))
            attitude_ned = (
                Rotation.from_quat(
                    messaging.as_np_quaternion(pose_stamped.pose.orientation)
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

        return _geopose_stamped(self.nav_sat_fix, self.pose_stamped)

    @property
    @ROS.publish(
        messaging.ROS_TOPIC_GIMBAL_QUATERNION, QoSPresetProfiles.SENSOR_DATA.value
    )
    def gimbal_quaternion(self) -> Optional[Quaternion]:
        """Gimbal orientation as :class:`geometry_msgs.msg.Quaternion` message
        or None if not available

        .. note::
            Current implementation assumes camera is facing directly down from
            vehicle body if GimbalDeviceAttitudeStatus (MAVLink gimbal protocol v2)
            is not available.
        """

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

        def _quaternion_multiply(q1, q2):
            w1, x1, y1, z1 = q1.w, q1.x, q1.y, q1.z
            w2, x2, y2, z2 = q2.w, q2.x, q2.y, q2.z

            w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
            x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
            y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
            z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

            return Quaternion(w=w, x=x, y=y, z=z)

        def _apply_vehicle_yaw(vehicle_q, gimbal_q):
            # Extract yaw from vehicle quaternion
            t3 = 2.0 * (vehicle_q.w * vehicle_q.z + vehicle_q.x * vehicle_q.y)
            t4 = 1.0 - 2.0 * (vehicle_q.y * vehicle_q.y + vehicle_q.z * vehicle_q.z)
            yaw_rad = math.atan2(t3, t4)

            # Create a new quaternion with only yaw rotation
            yaw_q = Quaternion(
                w=math.cos(yaw_rad / 2), x=0.0, y=0.0, z=math.sin(yaw_rad / 2)
            )

            # Apply the vehicle yaw rotation to the gimbal quaternion
            gimbal_yaw_q = _quaternion_multiply(yaw_q, gimbal_q)

            return gimbal_yaw_q

        # TODO check frame (e.g. base_link_frd/vehicle body in PX4 SITL simulation)
        @enforce_types(self.get_logger().warn, "Cannot determine gimbal Quaternion")
        def _gimbal_quaternion(
            geopose_stamped: GeoPoseStamped,
            gimbal_device_attitude_status: Optional[GimbalDeviceAttitudeStatus],
        ):
            if gimbal_device_attitude_status is None:
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

            assert gimbal_device_attitude_status is not None

            compound_q = _apply_vehicle_yaw(
                geopose_stamped.pose.orientation, gimbal_device_attitude_status.q
            )
            roll, pitch, yaw = _euler_from_quaternion(compound_q)

            return compound_q

        return _gimbal_quaternion(
            self.geopose_stamped, self.gimbal_device_attitude_status
        )

    @property
    @ROS.publish(messaging.ROS_TOPIC_HOME_GEOPOINT, QoSPresetProfiles.SENSOR_DATA.value)
    def home_geopoint(self) -> Optional[GeoPointStamped]:
        """Home position as :class:`geographic_msgs.msg.GeoPointStamped` message
        or None if not available"""

        @enforce_types(self.get_logger().warn, "Cannot determine home GeoPoint")
        def _home_geopoint(home_position: HomePosition):
            return GeoPointStamped(
                header=messaging.create_header("base_link"),
                position=GeoPoint(
                    latitude=home_position.geo.latitude,
                    longitude=home_position.geo.longitude,
                    altitude=home_position.geo.altitude,
                ),
            )

        return _home_geopoint(self.home_position)

    @property
    def _altitude_local(self) -> Optional[float]:
        """Returns z coordinate from :class:`sensor_msgs.msg.PoseStamped` message
        or None if not available"""

        @enforce_types(
            self.get_logger().warn, "Cannot determine vehicle local altitude"
        )
        def _altitude_local(pose_stamped: PoseStamped):
            return pose_stamped.pose.position.z

        return _altitude_local(self.pose_stamped)
