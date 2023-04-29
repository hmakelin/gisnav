"""Publishes mock GPS (GNSS) messages"""
import json
import socket
from datetime import datetime

import numpy as np
from geographic_msgs.msg import GeoPose, GeoPoseStamped
from geometry_msgs.msg import Pose, PoseStamped
from gps_time import GPSTime
from mavros_msgs.msg import Altitude
from px4_msgs.msg import SensorGps
from rclpy.qos import QoSPresetProfiles

from gisnav.data import Attitude
from gisnav.nodes.base.rviz_publisher_node import RVizPublisherNode

from . import messaging


class MockGPSNode(RVizPublisherNode):
    """A node that publishes a mock GPS message over the microRTPS bridge"""

    ROS_D_USE_SENSOR_GPS = True
    """Set to False to use GPSINPUT message for ArduPilot MAVROS,
    SensorGps otherwise (PX4)"""

    ROS_D_UDP_HOST = "127.0.0.1"
    """MAVProxy GPSInput plugin default host"""

    ROS_D_UDP_PORT = 25100
    """MAVProxy GPSInput plugin default port"""

    ROS_PARAM_DEFAULTS = [
        ("udp_host", ROS_D_UDP_HOST, True),
        ("udp_port", ROS_D_UDP_PORT, True),
        ("sensor_gps", ROS_D_USE_SENSOR_GPS, True),
    ]
    """List containing ROS parameter name, default value and read_only flag tuples"""

    def __init__(self, name: str):
        """
        Class initializer

        :param name: Node name
        """
        super().__init__(name)
        self._sensor_gps = (
            self.get_parameter("sensor_gps").get_parameter_value().bool_value
        )
        self._udp_host = (
            self.get_parameter("udp_host").get_parameter_value().string_value
        )
        self._udp_port = (
            self.get_parameter("udp_port").get_parameter_value().integer_value
        )

        if self._sensor_gps:
            self._mock_gps_pub = self.create_publisher(
                SensorGps,
                messaging.ROS_TOPIC_SENSOR_GPS,
                QoSPresetProfiles.SENSOR_DATA.value,
            )
            self._socket = None
        else:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._mock_gps_pub = None

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
        """
        Handles latest geopose estimate

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
        """
        Handles latest altitude message

        :param msg: Latest :class:`mavros_msgs.msg.Altitude` message
        """
        self._altitude_estimate = msg

    @property
    def _device_id(self) -> int:
        """Generates a device ID for the outgoing `px4_msgs.SensorGps` message"""
        # For reference, see:
        # https://docs.px4.io/main/en/middleware/drivers.html and
        # https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/drv_sensor.h
        # https://docs.px4.io/main/en/gps_compass/

        # DRV_GPS_DEVTYPE_SIM (0xAF) + dev 1 + bus 1 + DeviceBusType_UNKNOWN
        # = 10101111 00000001 00001 000
        # = 11469064
        return 11469064

    @staticmethod
    def _geopose_to_pose(msg: GeoPose):
        """
        Converts :class:`geographic_msgs.msg.GeoPose` to
        :class:`geometry_msgs.msg.Pose`
        """
        pose = Pose()

        pose.position.x = msg.position.latitude
        pose.position.y = msg.position.longitude
        pose.position.z = msg.position.altitude

        pose.orientation.x = msg.orientation.x
        pose.orientation.y = msg.orientation.y
        pose.orientation.z = msg.orientation.z
        pose.orientation.w = msg.orientation.w

        return pose

    @classmethod
    def _geoposestamped_to_posestamped(
        cls, geopose_stamped: GeoPoseStamped
    ) -> PoseStamped:
        """
        Converts :class:`geographic_msgs.msg.GeoPoseStamped` to
        :class:`geographic_msgs.msg.PoseStamped`
        """
        pose_stamped = PoseStamped()
        pose_stamped.header = geopose_stamped.header
        pose_stamped.pose = cls._geopose_to_pose(geopose_stamped.pose)
        return pose_stamped

    def _publish(self) -> None:
        """Sends a HIL_GPS message over MAVROS or a GPSINPUT message over UDP socket"""
        # TODO: handle velocity & position variance estimation better
        if (
            self._geopose_estimate is None
            or self._altitude_estimate is None
            or self._altitude_estimate.amsl is np.nan
        ):
            return None

        alt_amsl = self._altitude_estimate.amsl

        # TODO: check yaw sign (NED or ENU?)
        q = messaging.as_np_quaternion(self._geopose_estimate.pose.orientation)
        yaw = Attitude(q=q).yaw
        yaw = int(np.degrees(yaw % (2 * np.pi)))
        yaw = 360 if yaw == 0 else yaw  # MAVLink definition 0 := not available

        satellites_visible = np.iinfo(np.uint8).max
        timestamp = messaging.usec_from_header(self._geopose_estimate.header)

        eph = 10.0
        epv = 1.0

        if self._sensor_gps:
            yaw_rad = np.radians(yaw)

            msg = SensorGps()
            msg.timestamp = int(timestamp)
            msg.timestamp_sample = int(timestamp)
            msg.device_id = self._device_id
            # msg.device_id = 0
            msg.fix_type = 3
            msg.s_variance_m_s = 5.0  # not estimated, use default cruise speed
            msg.c_variance_rad = np.nan
            msg.lat = int(self._geopose_estimate.pose.position.latitude * 1e7)
            msg.lon = int(self._geopose_estimate.pose.position.longitude * 1e7)
            msg.alt = int(alt_amsl * 1e3)
            msg.alt_ellipsoid = int(self._geopose_estimate.pose.position.altitude * 1e3)
            msg.eph = eph
            msg.epv = epv
            msg.hdop = 0.0
            msg.vdop = 0.0
            msg.noise_per_ms = 0
            msg.automatic_gain_control = 0
            msg.jamming_state = 0
            msg.jamming_indicator = 0
            msg.vel_m_s = 0.0
            msg.vel_n_m_s = 0.0
            msg.vel_e_m_s = 0.0
            msg.vel_d_m_s = 0.0
            msg.cog_rad = np.nan
            msg.vel_ned_valid = True
            msg.timestamp_time_relative = 0
            msg.satellites_used = satellites_visible
            msg.time_utc_usec = msg.timestamp
            msg.heading = float(yaw_rad)
            msg.heading_offset = 0.0
            msg.heading_accuracy = 0.0

            self._mock_gps_pub.publish(msg)
        else:
            gps_time = GPSTime.from_datetime(datetime.utcfromtimestamp(timestamp / 1e6))

            msg = {}
            msg["usec"] = timestamp
            msg["gps_id"] = 0
            msg["ignore_flags"] = 0
            msg["time_week"] = gps_time.week_number
            msg["time_week_ms"] = int(gps_time.time_of_week * 1e3)
            msg["fix_type"] = 3
            msg["lat"] = int(self._geopose_estimate.pose.position.latitude * 1e7)
            msg["lon"] = int(self._geopose_estimate.pose.position.longitude * 1e7)
            msg["alt"] = alt_amsl
            msg["horiz_accuracy"] = eph
            msg["vert_accuracy"] = epv
            msg["speed_accuracy"] = 5.0  # not estimated, use default cruise speed
            msg["hdop"] = 0.0
            msg["vdop"] = 0.0
            msg["vn"] = 0.0
            msg["ve"] = 0.0
            msg["vd"] = 0.0
            msg["satellites_visible"] = satellites_visible
            msg["yaw"] = yaw * 100

            self._socket.sendto(
                f"{json.dumps(msg)}".encode("utf-8"), (self._udp_ip, self._udp_port)
            )

        # Publish pose stamped and path for rviz2, debugging etc.
        pose_stamped = self._geoposestamped_to_posestamped(self._geopose_estimate)
        self.publish_rviz(pose_stamped)
