"""Publishes mock GPS (GNSS) messages

.. mermaid::
    :caption: WIP: Mock GPS node data flow graph

    graph LR
        subgraph CVNode
            geopose_estimate[gisnav/vehicle/geopose/estimate]
            altitude_estimate[gisnav/vehicle/altitude/estimate]
        end

        subgraph MockGPSNode
            sensor_gps[fmu/in/sensor_gps]
            gps_input
        end

        geopose_estimate -->|geographic_msgs/GeoPose| MockGPSNode
        altitude_estimate -->|mavros_msgs/Altitude| MockGPSNode
        sensor_gps -->|px4_msgs.msg.SensorGps| micro-ros-agent:::hidden
        gps_input -->|GPSINPUT over UDP| MAVLink:::hidden
"""
import json
import socket
from datetime import datetime
from typing import Optional

import numpy as np
from geographic_msgs.msg import GeoPoseStamped
from gps_time import GPSTime
from mavros_msgs.msg import Altitude
from px4_msgs.msg import SensorGps
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from .. import messaging
from .._assertions import ROS
from .._data import Attitude
from ..static_configuration import (
    CV_NODE_NAME,
    ROS_NAMESPACE,
    ROS_TOPIC_RELATIVE_VEHICLE_ESTIMATED_ALTITUDE,
    ROS_TOPIC_RELATIVE_VEHICLE_ESTIMATED_GEOPOSE,
)


class MockGPSNode(Node):
    """A node that publishes a mock GPS message over the microRTPS bridge"""

    ROS_D_USE_SENSOR_GPS = True
    """Set to False to use GPSINPUT message for ArduPilot MAVROS,
    SensorGps otherwise (PX4)"""

    ROS_D_UDP_HOST = "127.0.0.1"
    """MAVProxy GPSInput plugin default host"""

    ROS_D_UDP_PORT = 25100
    """MAVProxy GPSInput plugin default port"""

    def __init__(self, *args, **kwargs):
        """Class initializer

        :param args: Positional arguments to parent :class:`.Node` constructor
        :param kwargs: Keyword arguments to parent :class:`.Node` constructor
        """
        super().__init__(*args, **kwargs)

        if self.sensor_gps:
            self._mock_gps_pub = self.create_publisher(
                SensorGps,
                messaging.ROS_TOPIC_SENSOR_GPS,
                QoSPresetProfiles.SENSOR_DATA.value,
            )
            self._socket = None
        else:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._mock_gps_pub = None

    @property
    @ROS.parameter(ROS_D_USE_SENSOR_GPS)
    def sensor_gps(self) -> Optional[bool]:
        """:term:`ROS` parameter flag indicating outgoing mock :term:`GPS` message
        should be published as :class:`px4_msgs.msg.SensorGps` for :term:`PX4`
        instead of as :class:`mavros_msgs.msg.GPSINPUT` for :term:`ArduPilot`.
        """

    @property
    @ROS.parameter(ROS_D_UDP_HOST)
    def udp_host(self) -> Optional[str]:
        """:term:`ROS` parameter MAVProxy GPSInput plugin host name or IP address"""

    @property
    @ROS.parameter(ROS_D_UDP_PORT)
    def udp_port(self) -> Optional[int]:
        """:term:`ROS` parameter MAVProxy GPSInput plugin port"""

    def _vehicle_estimated_geopose_cb(self, msg: GeoPoseStamped) -> None:
        """
        Handles latest geopose estimate

        :param msg: Latest :class:`geographic_msgs.msg.GeoPose` message
        """
        if self.vehicle_estimated_altitude is not None:
            self._publish()
        else:
            self.get_logger().warn(
                "Altitude estimate not yet received, skipping publishing mock "
                "GPS message."
            )

    @property
    @ROS.max_delay_ms(messaging.DELAY_DEFAULT_MS)
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_VEHICLE_ESTIMATED_GEOPOSE.replace("~", CV_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=_vehicle_estimated_geopose_cb,
    )
    def vehicle_estimated_geopose(self) -> Optional[GeoPoseStamped]:
        """Subscribed :term:`Vehicle` estimated :term:`geopose`, or None if not
        available
        """

    @property
    @ROS.max_delay_ms(messaging.DELAY_DEFAULT_MS)
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_VEHICLE_ESTIMATED_ALTITUDE.replace("~", CV_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def vehicle_estimated_altitude(self) -> Optional[Altitude]:
        """Subscribed :term:`Vehicle` estimated :term:`altitude`,or None if not
        available
        """

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

        if self.sensor_gps:
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

            # TODO: handle None host or port
            self._socket.sendto(
                f"{json.dumps(msg)}".encode("utf-8"), (self.udp_host, self.udp_port)
            )
