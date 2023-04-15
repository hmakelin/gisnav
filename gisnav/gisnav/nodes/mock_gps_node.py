"""Publishes mock GPS (GNSS) messages"""
from typing import Optional
#import socket
import json

import numpy as np
from geographic_msgs.msg import GeoPoseStamped
from mavros_msgs.msg import Altitude, HilGPS
from rclpy.qos import QoSPresetProfiles
from pymavlink import mavutil

from gisnav.data import Attitude
from gisnav.nodes.base.base_node import BaseNode

from . import messaging


class MockGPSNode(BaseNode):
    """A node that publishes a mock GPS message over the microRTPS bridge"""

    ROS_D_UDP_HOST = "127.0.0.1"
    """MAVProxy GPSInput plugin default host"""

    ROS_D_UDP_PORT = 14550  #25100
    """MAVLink HilGPS (PX4) or GPSInput (ArduPilot) plugin default port"""

    """MAVProxy GPSInput plugin default port"""
    ROS_D_USE_HIL_GPS = True
    """Set to False to use GPSINPUT message for ArduPilot MAVROS, 
    HIL_GPS otherwise (PX4)"""

    ROS_PARAM_DEFAULTS = [
        ("udp_host", ROS_D_UDP_HOST, True),
        ("udp_port", ROS_D_UDP_PORT, True),
        ("hil_gps", ROS_D_USE_HIL_GPS, True),
    ]
    """List containing ROS parameter name, default value and read_only flag tuples"""

    def __init__(self, name: str):
        """Class initializer

        :param name: Node name
        """
        super().__init__(name)
        self._udp_host = self.get_parameter("udp_host").get_parameter_value().string_value
        self._udp_port = (
            self.get_parameter("udp_port").get_parameter_value().integer_value
        )
        #self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._hil_gps = self.get_parameter("hil_gps").get_parameter_value().bool_value
        self._mav = mavutil.mavlink_connection(
            f"udpout:{self._udp_host}:{self._udp_port}",
            source_system=1,
            source_component=mavutil.mavlink.MAV_COMP_ID_GPS,
        )

        #self._mock_gps_pub = self.create_publisher(
        #    HilGPS, messaging.ROS_TOPIC_HIL_GPS, QoSPresetProfiles.SENSOR_DATA.value
        #)
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

    #@staticmethod
    #def _hil_gps_msg_to_mavlink_dict(hil_gps_msg: HilGPS) -> dict:
    #    """Converts a MAVROS HilGPS message to a Python dictionary representing
    #    the underlying MAVLink HIL_GPS message.
    #
    #    .. note::
    #        Used for now because for some reason the MAVROS hil gps callback
    #        is not triggering for the outgoing messages.
    #
    #    :param hil_gps_msg: MAVROS HilGPS message
    #    :return: Python dictionary representing the MAVLink HIL_GPS message
    #    """
    #    mavlink_hil_gps_dict = {
    #        "time_usec": hil_gps_msg.header.stamp.nanosec // 1000,
    #        "fix_type": hil_gps_msg.fix_type,
    #        "lat": int(hil_gps_msg.geo.latitude * 1e7),
    #        "lon": int(hil_gps_msg.geo.longitude * 1e7),
    #        "alt": int(hil_gps_msg.geo.altitude * 1e3),
    #        "eph": hil_gps_msg.eph,
    #        "epv": hil_gps_msg.epv,
    #        "vel": hil_gps_msg.vel,
    #        "vn": hil_gps_msg.vn,
    #        "ve": hil_gps_msg.ve,
    #        "vd": hil_gps_msg.vd,
    #        "cog": hil_gps_msg.cog,
    #        "satellites_visible": hil_gps_msg.satellites_visible,
    #    }
    #    return mavlink_hil_gps_dict

    def _send_mavlink_message(self) -> None:
        """Sends a MAVLink HIL_GPS or GPSINPUT message over UDP socket

        .. note::
            Used for now because for some reason the MAVROS hil gps callback
            is not triggering for the outgoing messages.

        :param msg: MAVLink HilGPS message
        """
        if self._hil_gps:
            if self._geopose_estimate is not None and self._altitude_estimate is not None:
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
                vel=0,  # np.iinfo(np.uint16).max,
                vn=0,
                ve=0,
                vd=0,
                cog=np.iinfo(np.uint16).max,
                satellites_visible=8  # TODO np.iinfo(np.uint8).max,
            )
            # id = 0,  # TODO: make id configurable
            # yaw = yaw
            msg.geo.altitude = alt_amsl

            self._mav.mav.hil_gps_send(
                msg.header.stamp.nanosec,
                msg.fix_type,
                int(msg.geo.latitude * 1e7),
                int(msg.geo.longitude * 1e7),
                int(msg.geo.altitude * 1e3),
                msg.eph,
                msg.epv,
                msg.vel,
                msg.vn,
                msg.ve,
                msg.vd,
                np.iinfo(np.uint16).max,
                msg.satellites_visible,
            )
        else:
            msg = {}

            # Adjust UTC epoch timestamp for estimation delay
            # int(time.time_ns() / 1e3) - (self._bridge.synchronized_time - timestamp)
            msg["usec"] = timestamp
            msg["gps_id"] = 0
            msg["ignore_flags"] = 56  # vel_horiz + vel_vert + speed_accuracy

            gps_time = GPSTime.from_datetime(datetime.utcfromtimestamp(msg["usec"] / 1e6))
            msg["time_week"] = gps_time.week_number
            msg["time_week_ms"] = int(
                gps_time.time_of_week * 1e3
            )  # TODO this implementation accurate only up to 1 second
            msg["fix_type"] = 3  # 3D position
            msg["lat"] = int(lat * 1e7)
            msg["lon"] = int(lon * 1e7)
            msg["alt"] = altitude_amsl  # ArduPilot Gazebo SITL expects AMSL
            msg["horiz_accuracy"] = 10.0
            msg["vert_accuracy"] = 3.0
            msg["speed_accuracy"] = np.nan  # should be in ignore_flags
            msg["hdop"] = 0.0
            msg["vdop"] = 0.0
            msg["vn"] = np.nan  # should be in ignore_flags
            msg["ve"] = np.nan  # should be in ignore_flags
            msg["vd"] = np.nan  # should be in ignore_flags
            msg["satellites_visible"] = np.iinfo(np.uint8).max

            # TODO check yaw sign (NED or ENU?)
            yaw = int(np.degrees(heading % (2 * np.pi)) * 100)
            yaw = 36000 if yaw == 0 else yaw  # MAVLink definition 0 := not available
            msg["yaw"] = yaw

            # todo: send gpsinput