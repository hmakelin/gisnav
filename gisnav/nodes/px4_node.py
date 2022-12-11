"""Module that contains the PX4Node ROS 2 node."""
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.qos import QoSPresetProfiles
from geometry_msgs.msg import Quaternion
from geographic_msgs.msg import GeoPoint, GeoPointStamped, GeoPose, GeoPoseStamped
from mavros_msgs.msg import Altitude
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, VehicleGlobalPosition, GimbalDeviceSetAttitude

from . import messaging
from .base.autopilot_node import _AutopilotNode
from ..assertions import assert_shape


class PX4Node(_AutopilotNode):
    """ROS 2 node that acts as an adapter for PX4's microRTPS bridge

    .. note::
        Current implementation uses :class:`px4_msgs.msg.GimbalDeviceSetAttitude` instead of
        :class:`px4_msgs.msg.GimbalDeviceAttitudeStatus` for :py:attr:`.gimbal_quaternion` because the SITL simulation
        does not publish the actual attitude. The set attitude does not match actual attitude in situations where
        gimbal has not yet stabilized.
    """

    # Should not need anything but these PX4 messages to implement parent :class:`._AutopilotNode` computed attributes
    __slots__ = 'vehicle_global_position', 'vehicle_local_position', 'vehicle_attitude', 'gimbal_device_set_attitude'

    def __init__(self, name: str) -> None:
        """Initializes the ROS 2 node

        :param name: Name of the node
        """
        super().__init__(name)

        self._vehicle_global_position = None
        self._vehicle_global_position_sub = self.create_subscription(VehicleGlobalPosition,
                                                                     '/fmu/vehicle_global_position/out',
                                                                     self._vehicle_global_position_callback,
                                                                     QoSPresetProfiles.SENSOR_DATA.value)
        self._vehicle_local_position = None
        self._vehicle_local_position_sub = self.create_subscription(VehicleLocalPosition,
                                                                    '/fmu/vehicle_local_position/out',
                                                                    self._vehicle_local_position_callback,
                                                                    QoSPresetProfiles.SENSOR_DATA.value)
        self._vehicle_attitude = None
        self._vehicle_attitude_sub = self.create_subscription(VehicleAttitude,
                                                              '/fmu/vehicle_attitude/out',
                                                              self._vehicle_attitude_callback,
                                                              QoSPresetProfiles.SENSOR_DATA.value)
        self._gimbal_device_set_attitude = None
        self._gimbal_device_set_attitude_sub = self.create_subscription(GimbalDeviceSetAttitude,
                                                                        '/fmu/gimbal_device_set_attitude/out',
                                                                        self._gimbal_device_set_attitude_callback,
                                                                        QoSPresetProfiles.SENSOR_DATA.value)

    # region ROS subscriber callbacks
    def _vehicle_global_position_callback(self, msg: VehicleGlobalPosition) -> None:
        """Handles latest :class:`px4_msgs.msg.VehicleGlobalPosition` message

        Calls :meth:`.publish_vehicle_geopose` and :meth:`.publish_vehicle_altitude` because the contents of those
        messages are affected by an updated :class:`px4_msgs.msg.VehicleGlobalPosition` message.

        :param msg: :class:`px4_msgs.msg.VehicleGlobalPosition` message from the PX4-ROS 2 bridge
        """
        self._vehicle_global_position = msg
        self.publish_vehicle_geopose()
        self.publish_vehicle_altitude()

    def _vehicle_local_position_callback(self, msg: VehicleLocalPosition) -> None:
        """Handles latest :class:`px4_msgs.msg.VehicleLocalPosition` message

        Calls :meth:`.publish_home_geopoint` because the content of that message is affected by an updated
        :class:`px4_msgs.msg.VehicleLocalPosition` message.

        :param msg: :class:`px4_msgs.msg.VehicleLocalPosition` message from the PX4-ROS 2 bridge
        """
        self._vehicle_local_position = msg
        self.publish_home_geopoint()

    def _vehicle_attitude_callback(self, msg: VehicleAttitude) -> None:
        """Handles latest :class:`px4_msgs.msg.VehicleAttitude` message

        Calls :meth:`.publish_vehicle_geopose` because the content of that message is affected by an updated
        :class:`px4_msgs.msg.VehicleAttitude` message.

        :param msg: :class:`px4_msgs.msg.VehicleAttitude` message from the PX4-ROS 2 bridge
        """
        self._vehicle_attitude = msg
        self.publish_vehicle_geopose()

    def _gimbal_device_set_attitude_callback(self, msg: GimbalDeviceSetAttitude) -> None:
        """Handles latest :class:`px4_msgs.msg.GimbalDeviceSetAttitude` message

        Calls :meth:`.publish_gimbal_quaternion` because the content of that message is affected by an updated
        :class:`px4_msgs.msg.GimbalDeviceSetAttitude` message.

        :param msg: :class:`px4_msgs.msg.GimbalDeviceSetAttitude` message from the PX4-ROS 2 bridge
        """
        self._gimbal_device_set_attitude = msg
        self.publish_gimbal_quaternion()
    # endregion ROS subscriber callbacks

    # region computed attributes
    @property
    def vehicle_geopose(self) -> Optional[GeoPoseStamped]:
        """Vehicle pose as :class:`geographic_msgs.msg.GeoPoseStamped` message or None if not available"""
        if self._vehicle_global_position is not None and self._vehicle_attitude is not None:
            latitude, longitude = self._vehicle_global_position.lat, self._vehicle_global_position.lon
            orientation = messaging.as_ros_quaternion(messaging.wxyz_to_xyzw_q(self._vehicle_attitude.q))

            if self.egm96_height is not None:
                # compute ellipsoid altitude
                altitude = self._vehicle_global_position.alt + self.egm96_height.data
            else:
                altitude = np.nan

            return GeoPoseStamped(header=messaging.create_header('base_link'),
                                  pose=GeoPose(
                                      position=GeoPoint(latitude=latitude, longitude=longitude, altitude=altitude),
                                      orientation=orientation)
                                  )
        else:
            # TODO: could publish a GeoPoint message even if VehicleAttitude is not available
            self.get_logger().warn('VehicleGlobalPosition and/or VehicleAttitude message not yet received, cannot '
                                   'determine vehicle geopose.')
            return None

    @property
    def _vehicle_altitude_local(self) -> Optional[float]:
        """Returns z coordinate from :class:`px4_msgs.msg.VehicleLocalPosition` message or None if not available"""
        if self._vehicle_local_position is not None:
            if self._vehicle_local_position.z_valid:
                return self._vehicle_local_position.z
            else:
                self.get_logger().warn('VehicleLocalPosition message z is not valid, cannot determine vehicle local '
                                       'altitude')
                return None
        else:
            self.get_logger().warn('VehicleLocalPosition message not yet received, cannot determine vehicle local '
                                   'altitude.')
            return None

    @property
    def vehicle_altitude(self) -> Optional[Altitude]:
        """Vehicle altitude as :class:`mavros_msgs.msg.Altitude` message or None if not available"""
        if self._vehicle_global_position is not None and self.terrain_altitude is not None:
            amsl = self._vehicle_global_position.alt
            terrain = self._vehicle_global_position.alt - self.terrain_altitude.amsl
            local = self._vehicle_altitude_local if self._vehicle_altitude_local is not None else np.nan
            altitude = Altitude(
                header=messaging.create_header('base_link'),
                amsl=amsl,
                local=local,  # TODO: home altitude ok? see https://mavlink.io/en/messages/common.html#ALTITUDE
                relative=-local,
                terrain=terrain,
                bottom_clearance=np.nan
            )
            return altitude
        else:
            self.get_logger().warn(f'VehicleGlobalPosition {self._vehicle_global_position} and/or terrain Altitude'
                                   f' {self.terrain_altitude} message not yet received, cannot determine vehicle '
                                   f'altitude.')
            return None

    @property
    def gimbal_quaternion(self) -> Optional[Quaternion]:
        """Gimbal orientation as :class:`geometry_msgs.msg.Quaternion` message or None if not available"""
        if self._vehicle_attitude is None or self._gimbal_device_set_attitude is None:
            return None

        # Gimbal roll & pitch/tilt is assumed stabilized so only need yaw/pan
        yaw_mask = np.array([1, 0, 0, 1])  # TODO: remove assumption
        assert_shape(self._vehicle_attitude.q.squeeze(), (4,))
        vehicle_yaw = self._vehicle_attitude.q * yaw_mask
        # geometry_msgs Quaternion expects (x, y, z, w) while px4_msgs VehicleAttitude has (w, x, y, z)
        vehicle_yaw = Rotation.from_quat(np.append(vehicle_yaw[1:], vehicle_yaw[0]))

        assert_shape(self._gimbal_device_set_attitude.q.squeeze(), (4,))
        gimbal_quaternion_frd = self._gimbal_device_set_attitude.q
        gimbal_quaternion_frd = Rotation.from_quat(np.append(gimbal_quaternion_frd[1:], gimbal_quaternion_frd[0]))
        gimbal_quaternion_ned = vehicle_yaw * gimbal_quaternion_frd  # TODO: ENU instead of NED? ROS convention?
        gimbal_quaternion_ned = messaging.as_ros_quaternion(gimbal_quaternion_ned.as_quat())

        return gimbal_quaternion_ned

    @property
    def home_geopoint(self) -> Optional[GeoPointStamped]:
        """Home position as :class:`geographic_msgs.msg.GeoPointStamped` message or None if not available"""
        if self._vehicle_local_position is not None:
            latitude, longitude = self._vehicle_local_position.ref_lat, self._vehicle_local_position.ref_lon

            if self.egm96_height is not None:
                # compute ellipsoid altitude
                altitude = self._vehicle_local_position.ref_alt + self.egm96_height.data
            else:
                altitude = np.nan

            return GeoPointStamped(header=messaging.create_header('base_link'),
                                   position=GeoPoint(latitude=latitude, longitude=longitude, altitude=altitude))
        else:
            self.get_logger().warn('VehicleLocalPosition message not yet received, cannot determine home geopoint.')
            return None
    # endregion computed attributes
