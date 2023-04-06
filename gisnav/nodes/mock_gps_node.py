"""Publishes mock GPS (GNSS) messages"""
from datetime import datetime

import numpy as np
from geographic_msgs.msg import GeoPoseStamped
from gps_time import GPSTime
from mavros_msgs.msg import GPSINPUT, Altitude
from rclpy.qos import QoSPresetProfiles

from gisnav.data import Attitude
from gisnav.nodes.base.base_node import BaseNode

from . import messaging


class MockGPSNode(BaseNode):
    """A node that publishes a mock GPS message over the microRTPS bridge"""

    def __init__(self, name: str):
        """Class initializer

        :param name: Node name
        """
        super().__init__(name)

        self._mock_gps_pub = self.create_publisher(
            GPSINPUT, messaging.ROS_TOPIC_GPS_INPUT, QoSPresetProfiles.SENSOR_DATA.value
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
            lat, lon = msg.pose.position.latitude, msg.pose.position.longitude
            alt_amsl = self._altitude_estimate.amsl
            alt_ellipsoid = msg.pose.position.altitude
            q = messaging.as_np_quaternion(msg.pose.orientation)
            yaw = Attitude(q=q).yaw
            timestamp = messaging.usec_from_header(msg.header)
            self._publish(lat, lon, alt_amsl, alt_ellipsoid, yaw, timestamp)
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

    def _publish(
        self, lat, lon, altitude_amsl, altitude_ellipsoid, heading, timestamp
    ) -> None:
        """Publishes drone position as a :class:`px4_msgs.msg.SensorGps` message

        :param lat: Vehicle latitude
        :param lon: Vehicle longitude
        :param altitude_amsl: Vehicle altitude AMSL
        :param altitude_ellipsoid: Vehicle altitude above WGS 84 ellipsoid
        :param heading: Vehicle heading in radians
        :param timestamp: Timestamp for outgoing mock GPS message (e.g. system time)

        """
        msg = self._generate_gps_input(lat, lon, altitude_amsl, heading, timestamp)

        if msg is not None:
            self._mock_gps_pub.publish(msg)
        else:
            self.get_logger().info("Could not create GPS message, skipping publishing.")

    def _generate_gps_input(
        self, lat, lon, altitude_amsl, heading, timestamp
    ) -> GPSINPUT:
        """Generates a :class:`.GPSINPUT` message to send over MAVROS

        .. seealso:
            `GPS_INPUT_IGNORE_FLAGS <https://mavlink.io/en/messages/common.html#GPS_INPUT_IGNORE_FLAGS>`_  # noqa: E501

        :param lat: Vehicle latitude
        :param lon: Vehicle longitude
        :param altitude_amsl: Vehicle altitude in meters AMSL
        :param heading: Vehicle heading in radians
        :param timestamp: System time in microseconds
        :return: MAVLink GPS_INPUT message as Python dict
        """
        header = messaging.create_header("base_link")
        header.stamp.sec = int(timestamp / 1e6)
        header.stamp.nanosec = int(((timestamp / 1e6) % 1) * 1e9)
        gps_time = GPSTime.from_datetime(datetime.utcfromtimestamp(header.stamp.sec))

        # TODO check yaw sign (NED or ENU?)
        yaw = int(np.degrees(heading % (2 * np.pi)) * 100)
        yaw = 36000 if yaw == 0 else yaw  # MAVLink definition 0 := not available

        msg = GPSINPUT(
            header=header,
            gps_id=0,
            ignore_flags=56,  # vel_horiz + vel_vert + speed_accuracy
            time_week=gps_time.week_number,
            time_week_ms=int(gps_time.time_of_week * 1e3),
            fix_type=3,  # 3D position
            lat=int(lat * 1e7),
            lon=int(lon * 1e7),
            alt=altitude_amsl,
            horiz_accuracy=10.0,  # position.eph
            vert_accuracy=3.0,  # position.epv
            speed_accuracy=np.nan,  # should be in ignore_flags
            hdop=0.0,
            vdop=0.0,
            vn=np.nan,  # should be in ignore_flags
            ve=np.nan,  # should be in ignore_flags
            vd=np.nan,  # should be in ignore_flags
            satellites_visible=np.iinfo(np.uint8).max,
            yaw=yaw,
        )

        return msg
