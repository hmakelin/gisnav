"""GISNav :term:`extension` :term:`node` that publishes mock GPS (GNSS) messages

.. mermaid::
    :caption: Mock GPS node data flow graph

    graph LR
        subgraph PnPNode
            pose[gisnav/vehicle/estimated/pose]
        end

        subgraph MockGPSNode
            sensor_gps[fmu/in/sensor_gps]
            gps_input
        end

        pose -->|geographic_msgs/PoseStamped| MockGPSNode
        sensor_gps -->|px4_msgs.msg.SensorGps| micro-ros-agent:::hidden
        gps_input -->|GPSINPUT over UDP| MAVLink:::hidden
"""
import json
import socket
from datetime import datetime
from typing import Final, Optional

import numpy as np
from geometry_msgs.msg import PoseStamped
from gps_time import GPSTime
from px4_msgs.msg import SensorGps
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from .. import messaging
from .._data import Attitude
from .._decorators import ROS, narrow_types
from ..static_configuration import (
    PNP_NODE_NAME,
    ROS_NAMESPACE,
    ROS_TOPIC_RELATIVE_CAMERA_ESTIMATED_POSE,
)

_ROS_PARAM_DESCRIPTOR_READ_ONLY: Final = ParameterDescriptor(read_only=True)
"""A read only ROS parameter descriptor"""


class MockGPSNode(Node):
    """A node that publishes a mock GPS message over the microRTPS bridge"""

    ROS_D_USE_SENSOR_GPS: Final = True
    """Set to ``False`` to use :class:`mavros_msgs.msg.GPSINPUT` message for
    :term:`ArduPilot`, :class:`px4_msgs.msg.SensorGps` for :term:`PX4` otherwise.
    """

    ROS_D_UDP_HOST: Final = "127.0.0.1"
    """MAVProxy GPSInput plugin default host

    .. note::
        Only used if :attr:`use_sensor_gps` is ``False``
    """

    ROS_D_UDP_PORT: Final = 25100
    """MAVProxy GPSInput plugin default port

    .. note::
        Only used if :attr:`use_sensor_gps` is ``False``
    """

    ROS_TOPIC_SENSOR_GPS: Final = "/fmu/in/sensor_gps"
    """Absolute :term:`topic` into which this :term:`node` publishes
    :attr:`.sensor_gps`
    """

    def __init__(self, *args, **kwargs):
        """Class initializer

        :param args: Positional arguments to parent :class:`.Node` constructor
        :param kwargs: Keyword arguments to parent :class:`.Node` constructor
        """
        super().__init__(*args, **kwargs)

        # Call the decorated properties to setup subscriptions
        self.vehicle_estimated_geopose
        self.vehicle_estimated_altitude

        if self.use_sensor_gps:
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
    @ROS.parameter(ROS_D_USE_SENSOR_GPS, descriptor=_ROS_PARAM_DESCRIPTOR_READ_ONLY)
    def use_sensor_gps(self) -> Optional[bool]:
        """:term:`ROS` parameter flag indicating outgoing mock :term:`GPS` message
        should be published as :class:`px4_msgs.msg.SensorGps` for :term:`PX4`
        instead of as :class:`mavros_msgs.msg.GPSINPUT` for :term:`ArduPilot`.
        """

    @property
    @ROS.parameter(ROS_D_UDP_HOST, descriptor=_ROS_PARAM_DESCRIPTOR_READ_ONLY)
    def udp_host(self) -> Optional[str]:
        """:term:`ROS` parameter MAVProxy GPSInput plugin host name or IP address"""

    @property
    @ROS.parameter(ROS_D_UDP_PORT, descriptor=_ROS_PARAM_DESCRIPTOR_READ_ONLY)
    def udp_port(self) -> Optional[int]:
        """:term:`ROS` parameter MAVProxy GPSInput plugin port"""

    def _camera_estimated_pose_cb(self, msg: PoseStamped) -> None:
        """
        Handles latest :term:`camera` pose estimate

        :param msg: Latest :class:`geographic_msgs.msg.PoseStamped` message
        """
        self.sensor_gps if self.use_sensor_gps else self.gps_input

    @property
    @ROS.max_delay_ms(messaging.DELAY_DEFAULT_MS)
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_CAMERA_ESTIMATED_POSE.replace("~", PNP_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=_camera_estimated_pose_cb,
    )
    def camera_estimated_pose(self) -> Optional[PoseStamped]:
        """Subscribed :term:`camera` estimated :term:`pose`, or None if not
        available
        """

    @property
    @ROS.publish(
        ROS_TOPIC_SENSOR_GPS,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def sensor_gps(self) -> Optional[SensorGps]:
        """Outgoing mock :term:`GNSS` :term:`message` when :attr:`use_sensor_gps`
        is ``True``
        """

        @narrow_types(self)
        def _sensor_gps(
            vehicle_estimated_pose: PoseStamped,
            device_id: int,
        ) -> Optional[SensorGps]:
            # TODO: check yaw sign (NED or ENU?)
            q = messaging.as_np_quaternion(vehicle_estimated_pose.orientation)
            yaw = Attitude(q=q).yaw
            yaw = int(np.degrees(yaw % (2 * np.pi)))
            yaw = 360 if yaw == 0 else yaw  # MAVLink definition 0 := not available

            satellites_visible = np.iinfo(np.uint8).max
            timestamp = messaging.usec_from_header(vehicle_estimated_pose.header)

            eph = 10.0
            epv = 1.0

            yaw_rad = np.radians(yaw)

            msg = SensorGps()
            msg.timestamp = int(timestamp)
            msg.timestamp_sample = int(timestamp)
            msg.device_id = device_id
            # msg.device_id = 0
            msg.fix_type = 3
            msg.s_variance_m_s = 5.0  # not estimated, use default cruise speed
            msg.c_variance_rad = np.nan

            # TODO lat lon
            # msg.lat = int(vehicle_estimated_pose.position.latitude * 1e7)
            # msg.lon = int(vehicle_estimated_pose.position.longitude * 1e7)

            # TODO altitude
            # msg.alt = int(vehicle_estimated_altitude.amsl * 1e3)
            # msg.alt_ellipsoid = int(
            #    vehicle_estimated_geopose.pose.position.altitude * 1e3
            # )

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

            return msg

        # TODO vheicle pose
        vehicle_estimated_pose = self.camera_estimated_pose

        return _sensor_gps(
            vehicle_estimated_pose,
            self._device_id,
        )

    @property
    # @ROS.publish(
    #    ROS_TOPIC_GPS_INPUT,
    #    QoSPresetProfiles.SENSOR_DATA.value,
    # )
    def gps_input(self) -> Optional[dict]:  # Optional[GPSINPUT]
        """Outgoing mock :term:`GNSS` :term:`message` when :attr:`use_sensor_gps`
        is ``False``

        .. note::
            Does not use :class:`mavros_msgs.msg.GPSINPUT` message over MAVROS,
            sends a :term:`MAVLink` GPS_INPUT message directly to :term:`ArduPilot`.
        """

        @narrow_types
        def _gps_input(
            vehicle_estimated_pose: PoseStamped,
        ) -> Optional[dict]:
            # TODO: check yaw sign (NED or ENU?)
            q = messaging.as_np_quaternion(vehicle_estimated_pose.orientation)
            yaw = Attitude(q=q).yaw
            yaw = int(np.degrees(yaw % (2 * np.pi)))
            yaw = 360 if yaw == 0 else yaw  # MAVLink definition 0 := not available

            satellites_visible = np.iinfo(np.uint8).max
            timestamp = messaging.usec_from_header(vehicle_estimated_pose.header)

            eph = 10.0
            epv = 1.0

            gps_time = GPSTime.from_datetime(datetime.utcfromtimestamp(timestamp / 1e6))

            # TODO: lat, lon, alt
            msg = dict(
                usec=timestamp,
                gps_id=0,
                ignore_flags=0,
                time_week=gps_time.week_number,
                time_week_ms=int(gps_time.time_of_week * 1e3),
                fix_type=3,
                # lat=int(vehicle_estimated_geopose.pose.position.latitude * 1e7),
                # lon=int(vehicle_estimated_geopose.pose.position.longitude * 1e7),
                # alt=vehicle_estimated_altitude.amsl,
                horiz_accuracy=eph,
                vert_accuracy=epv,
                speed_accuracy=5.0,
                hdop=0.0,
                vdop=0.0,
                vn=0.0,
                ve=0.0,
                vd=0.0,
                satellites_visible=satellites_visible,
                yaw=yaw * 100,
            )

            # TODO: handle None host or port
            self._socket.sendto(
                f"{json.dumps(msg)}".encode("utf-8"), (self.udp_host, self.udp_port)
            )

            return msg

        # TODO
        return None
        # return _gps_input(
        #    self.vehicle_estimated_geopose, self.vehicle_estimated_altitude
        # )

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
