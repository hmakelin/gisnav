"""Publishes mock GPS (GNSS) messages"""
from typing import Optional

import numpy as np
from geographic_msgs.msg import GeoPoseStamped
from mavros_msgs.msg import Altitude, HilGPS
from rclpy.qos import QoSPresetProfiles

from gisnav.data import Attitude
from gisnav.nodes.base.base_node import BaseNode

from . import messaging


class MockGPSNode(BaseNode):
    """A node that publishes a mock GPS message over the microRTPS bridge"""

    ROS_PARAM_DEFAULTS = []
    """List containing ROS parameter name, default value and read_only flag tuples"""

    def __init__(self, name: str):
        """Class initializer

        :param name: Node name
        """
        super().__init__(name)

        self._mock_gps_pub = self.create_publisher(
            HilGPS, messaging.ROS_TOPIC_HIL_GPS, QoSPresetProfiles.SENSOR_DATA.value
        )
        self._vehicle_geopose_estimate_sub = self.create_subscription(
            GeoPoseStamped,
            messaging.ROS_TOPIC_VEHICLE_GEOPOSE_ESTIMATE,
            self._vehicle_geopose_estimate_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self._vehicle_altitude_estimate_sub = self.create_subscription(
            Altitude,
            messaging.ROS_TOPIC_VEHICLE_ALTITUDE_ESTIMATE,
            self._vehicle_altitude_estimate_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self._geopose_estimate = None
        self._altitude_estimate = None

    def _vehicle_geopose_estimate_callback(self, msg: GeoPoseStamped) -> None:
        """Handles latest geopose estimate

        :param msg: Latest :class:`geographic_msgs.msg.GeoPose` message
        """
        self._geopose_estimate = msg
        if self._altitude_estimate is not None:
            self._publish()
        else:
            self.get_logger().warn(
                "Altitude estimate not yet received, skipping publishing mock "
                "GPS message."
            )

    def _vehicle_altitude_estimate_callback(self, msg: Altitude) -> None:
        """Handles latest altitude message
        :param msg: Latest :class:`mavros_msgs.msg.Altitude` message
        """
        self._altitude_estimate = msg

    def _publish(self) -> None:
        """Publishes drone position as a :class:`px4_msgs.msg.SensorGps` message"""
        msg = self._generate_hil_gps()
        if msg is not None:
            self._mock_gps_pub.publish(msg)
        else:
            self.get_logger().info("Could not create GPS message, skipping publishing.")

    def _generate_hil_gps(self) -> Optional[HilGPS]:
        """Generates a :class:`.HilGPS` message to send over MAVROS

        :return: MAVLink HilGPS message
        """
        if self._geopose_estimate is not None and self._attitude_estimate is not None:
            geopose = self._geopose_estimate.pose
            header = self._geopose_estimate.header
            alt_amsl = self._altitude_estimate.amsl
        else:
            return None
        # alt_ellipsoid = msg.pose.position.altitude  # TODO: make this right

        # TODO check yaw sign (NED or ENU?)
        q = messaging.as_np_quaternion(geopose.orientation)
        yaw = Attitude(q=q).yaw
        yaw = int(np.degrees(yaw % (2 * np.pi)) * 100)
        yaw = 36000 if yaw == 0 else yaw  # MAVLink definition 0 := not available

        msg = HilGPS(
            header=header,
            fix_type=3,  # 3D position
            geo=geopose.position,
            eph=10,  # position.eph
            epv=3,  # position.epv
            vel=np.iinfo(np.uint16).max,
            vn=0,
            ve=0,
            vd=0,
            cog=np.iinfo(np.uint16).max,
            satellites_visible=np.iinfo(np.uint8).max,
        )
        # id = 0,  # TODO: make id configurable
        # yaw = yaw

        # this should be wgs 84 ellipsoid altitude
        msg.geo.altitude = alt_amsl  # todo: fix incoming geopose altitude
        msg.geo.latitude *= 1e7
        msg.geo.longitude *= 1e7  # expected by PX4 but not in documentation?

        return msg
