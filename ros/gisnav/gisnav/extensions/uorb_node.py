"""This module contains :class:`.UORBNode`, an extension ROS node that publishes PX4
uORB :class:`.SensorGps` (GNSS) messages to the uXRCE-DDS middleware
"""
from typing import Final, Optional

import numpy as np
from px4_msgs.msg import SensorGps
from rcl_interfaces.msg import ParameterDescriptor

from .._decorators import ROS, narrow_types
from ..constants import ROS_TOPIC_SENSOR_GPS
from ._mock_gps_node import MockGPSNode

_ROS_PARAM_DESCRIPTOR_READ_ONLY: Final = ParameterDescriptor(read_only=True)
"""A read only ROS parameter descriptor"""


class UORBNode(MockGPSNode):
    """A node that publishes PX4 uORB :class:`.SensorGps` (GNSS) messages to the
    uXRCE-DDS middleware"""

    def __init__(self, *args, **kwargs):
        """Class initializer

        :param args: Positional arguments to parent :class:`.Node` constructor
        :param kwargs: Keyword arguments to parent :class:`.Node` constructor
        """
        super().__init__(*args, **kwargs)

    def _publish(self, mock_gps_dict: MockGPSNode.MockGPSDict) -> None:
        self.sensor_gps(**mock_gps_dict)

    @narrow_types
    @ROS.publish(ROS_TOPIC_SENSOR_GPS, 10)  # QoSPresetProfiles.SENSOR_DATA.value,
    def sensor_gps(
        self,
        lat: int,  # todo update to new message definition with degrees, not 1e7 degrees
        lon: int,  # todo update to new message definition with degrees, not 1e7 degrees
        altitude_ellipsoid: float,
        altitude_amsl: float,
        yaw_degrees: int,
        h_variance_rad: float,
        vel_n_m_s: float,
        vel_e_m_s: float,
        vel_d_m_s: float,
        cog: float,
        cog_variance_rad: float,
        s_variance_m_s: float,
        timestamp: int,
        eph: float,
        epv: float,
        satellites_visible: int,
    ) -> Optional[SensorGps]:
        """Outgoing mock GPS message, or None if cannot be computed

        > [!IMPORTANT] px4_msgs release/1.14
        > Uses the release/1.14 tag version of :class:`px4_msgs.msg.SensorGps`
        """
        yaw_rad = np.radians(yaw_degrees)
        msg = SensorGps()

        try:
            msg.timestamp = 0
            msg.timestamp_sample = int(timestamp)
            msg.device_id = 0
            msg.fix_type = 3
            msg.s_variance_m_s = s_variance_m_s
            msg.c_variance_rad = cog_variance_rad
            msg.lat = lat
            msg.lon = lon
            msg.alt_ellipsoid = int(altitude_ellipsoid * 1e3)
            msg.alt = int(altitude_amsl * 1e3)
            msg.eph = eph
            msg.epv = epv
            msg.hdop = 0.0
            msg.vdop = 0.0
            msg.noise_per_ms = 0
            msg.automatic_gain_control = 0
            msg.jamming_state = 0  # 1 := OK, 0 := UNKNOWN
            msg.jamming_indicator = 0
            msg.spoofing_state = 0  # 1 := OK, 0 := UNKNOWN
            msg.vel_m_s = np.sqrt(vel_n_m_s**2 + vel_e_m_s**2 + vel_d_m_s**2)
            msg.vel_n_m_s = vel_n_m_s
            msg.vel_e_m_s = vel_e_m_s
            msg.vel_d_m_s = vel_d_m_s
            msg.cog_rad = cog
            msg.vel_ned_valid = True
            msg.timestamp_time_relative = 0
            msg.satellites_used = satellites_visible
            msg.time_utc_usec = msg.timestamp_sample
            msg.heading = float(yaw_rad)
            msg.heading_offset = 0.0  # assume map frame is an ENU frame
            msg.heading_accuracy = h_variance_rad
        except AssertionError as e:
            self.get_logger().warning(
                f"Could not create mock GPS message due to exception: {e}"
            )
            msg = None

        return msg

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
