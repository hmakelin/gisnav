"""Publishes mock GPS (GNSS) messages"""
from datetime import datetime

import numpy as np
from geographic_msgs.msg import GeoPoseStamped
from gps_time import GPSTime
from mavros_msgs.msg import GPSINPUT, Altitude, HilGPS
from rclpy.qos import QoSPresetProfiles

from gisnav.data import Attitude
from gisnav.nodes.base.base_node import BaseNode

from . import messaging


class MockGPSNode(BaseNode):
    """A node that publishes a mock GPS message over the microRTPS bridge"""

    ROS_D_USE_HIL_GPS = True
    """Set to False to use GPSINPUT message for ArduPilot MAVROS,
    HIL_GPS otherwise (PX4)"""

    ROS_PARAM_DEFAULTS = [
        ("hil_gps", ROS_D_USE_HIL_GPS, True),
    ]
    """List containing ROS parameter name, default value and read_only flag tuples"""

    def __init__(self, name: str):
        """Class initializer

        :param name: Node name
        """
        super().__init__(name)
        self._hil_gps = self.get_parameter("hil_gps").get_parameter_value().bool_value

        if self._hil_gps:
            # Must match QoS settings with mavros hil plugin
            self._mock_gps_pub = self.create_publisher(
                HilGPS, messaging.ROS_TOPIC_HIL_GPS, 10
            )
        else:
            # match QoS settings with mavros gps_input plugin
            self._mock_gps_pub = self.create_publisher(
                GPSINPUT, messaging.ROS_TOPIC_GPS_INPUT, 1
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
            self._send_mavlink_message()
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

    def _send_mavlink_message(self) -> None:
        """Sends a MAVLink HIL_GPS or GPSINPUT message over UDP socket

        .. note::
            Used for now because for some reason the MAVROS hil gps callback
            is not triggering for the outgoing messages.

        :param msg: MAVLink HilGPS message
        """
        if self._geopose_estimate is None or self._altitude_estimate is None:
            return None

        if self._hil_gps:
            geopose = self._geopose_estimate.pose
            header = self._geopose_estimate.header
            alt_amsl = self._altitude_estimate.amsl

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
                vel=0,  # np.iinfo(np.uint16).max,
                vn=0,
                ve=0,
                vd=0,
                cog=np.iinfo(np.uint16).max,
                satellites_visible=np.iinfo(np.uint8).max,
            )
            # id = 0,  # TODO: make id configurable
            # yaw = yaw
            msg.geo.altitude = alt_amsl

            self._mock_gps_pub.publish(msg)
        else:
            lat, lon = (
                self._geopose_estimate.pose.position.latitude,
                self._geopose_estimate.pose.position.longitude,
            )
            altitude_amsl = self._altitude_estimate.amsl
            self._geopose_estimate.pose.position.altitude
            q = messaging.as_np_quaternion(self._geopose_estimate.pose.orientation)
            heading = Attitude(q=q).yaw
            timestamp = messaging.usec_from_header(self._geopose_estimate.header)
            gps_time = GPSTime.from_datetime(datetime.utcfromtimestamp(timestamp / 1e6))
            yaw = int(np.degrees(heading % (2 * np.pi)) * 100)
            yaw = 36000 if yaw == 0 else yaw  # MAVLink definition 0 := not available

            msg = GPSINPUT(
                header=self._geopose_estimate.header,
                fix_type=3,
                gps_id=0,
                ignore_flags=56,  # vel_horiz + vel_vert + speed_accuracy
                time_week_ms=int(gps_time.time_of_week * 1e3),  # accurate up to 1 sec
                time_week=gps_time.week_number,
                lat=int(lat * 1e7),
                lon=int(lon * 1e7),
                alt=altitude_amsl,
                horiz_accuracy=10.0,
                vert_accuracy=3.0,
                speed_accuracy=np.nan,
                hdop=0.0,
                vdop=0.0,
                vn=np.nan,
                ve=np.nan,
                vd=np.nan,
                satellites_visible=np.iinfo(np.uint8).max,
                yaw=yaw,
            )

            self._mock_gps_pub.publish(msg)
