"""This module contains :class:`.UORBNode`, an extension ROS node that publishes PX4
uORB :class:`.SensorGps` (GNSS) messages to the uXRCE-DDS middleware
"""
import time
from typing import Final, Optional

import numpy as np
from rcl_interfaces.msg import ParameterDescriptor
from ublox_msgs.msg import NavPVT

from .._decorators import ROS, narrow_types
from ..constants import ROS_TOPIC_RELATIVE_NAV_PVT
from ._mock_gps_node import MockGPSNode

_ROS_PARAM_DESCRIPTOR_READ_ONLY: Final = ParameterDescriptor(read_only=True)
"""A read only ROS parameter descriptor"""


class UBXNode(MockGPSNode):
    """A node that publishes UBX messages to FCU via serial port"""

    ROS_D_PORT = "/dev/ttyS1"
    """Default for :attr:`.port`"""

    ROS_D_BAUDRATE = 9600
    """Default for :attr:`.baudrate`"""

    # EPSG code for WGS 84 and a common mean sea level datum (e.g., EGM96)
    _EPSG_WGS84 = 4326
    _EPSG_MSL = 5773  # Example: EGM96

    def __init__(self, *args, **kwargs):
        """Class initializer

        :param args: Positional arguments to parent :class:`.Node` constructor
        :param kwargs: Keyword arguments to parent :class:`.Node` constructor
        """
        super().__init__(*args, **kwargs)

    @property
    @ROS.parameter(ROS_D_PORT, descriptor=_ROS_PARAM_DESCRIPTOR_READ_ONLY)
    def port(self) -> Optional[str]:
        """Serial port for outgoing u-blox messages"""

    @property
    @ROS.parameter(ROS_D_BAUDRATE, descriptor=_ROS_PARAM_DESCRIPTOR_READ_ONLY)
    def baudrate(self) -> Optional[int]:
        """Baudrate for outgoing u-blox messages"""

    def _publish(self, mock_gps_dict: MockGPSNode.MockGPSDict) -> None:
        self.nav_pvt(**mock_gps_dict)

    @narrow_types
    @ROS.publish(ROS_TOPIC_RELATIVE_NAV_PVT, 10)  # QoSPresetProfiles.SENSOR_DATA.value,
    def nav_pvt(
        self,
        lat: int,
        lon: int,
        altitude_ellipsoid: float,
        altitude_amsl: float,
        yaw_degrees: int,
        h_variance_rad: float,
        vel_n_m_s: float,
        vel_e_m_s: float,
        vel_d_m_s: float,
        cog: float,
        s_variance_m_s: float,
        timestamp: int,
        eph: float,
        epv: float,
        satellites_visible: int,
        **kwargs,  # extra keyword arguments from MockGPSDict - not used here
    ) -> Optional[NavPVT]:
        """Retusn UBX mock GPS message, or None if cannot be computed"""
        msg = NavPVT()

        try:
            # Convert timestamp to GPS time of week
            gps_week, time_of_week = self._unix_to_gps_time(
                timestamp / 1e6
            )  # Assuming timestamp is in microseconds

            msg.i_tow = int(time_of_week * 1000)  # GPS time of week in ms
            (
                msg.year,
                msg.month,
                msg.day,
                msg.hour,
                msg.min,
                msg.sec,
            ) = self._get_utc_time(timestamp / 1e6)

            msg.valid = (
                0x01 | 0x02 | 0x04
            )  # Assuming valid date, time, and fully resolved
            msg.t_acc = 50000000  # Time accuracy estimate in ns (50ms)
            msg.nano = 0  # Fraction of second, range -1e9 .. 1e9 (UTC)

            msg.fix_type = 3  # 3D-Fix
            msg.flags = 0x01  # gnssFixOK
            msg.flags2 = 0
            msg.num_sv = satellites_visible

            msg.lon = lon
            msg.lat = lat
            msg.height = int(
                altitude_ellipsoid * int(1e3)
            )  # Height above ellipsoid in mm
            msg.h_msl = int(
                altitude_amsl * int(1e3)
            )  # Height above mean sea level in mm
            msg.h_acc = int(eph * int(1e3))  # Horizontal accuracy estimate in mm
            msg.v_acc = int(epv * int(1e3))  # Vertical accuracy estimate in mm

            msg.vel_n = int(vel_n_m_s * int(1e3))  # NED north velocity in mm/s
            msg.vel_e = int(vel_e_m_s * int(1e3))  # NED east velocity in mm/s
            msg.vel_d = int(vel_d_m_s * int(1e3))  # NED down velocity in mm/s
            msg.g_speed = int(
                np.sqrt(vel_n_m_s**2 + vel_e_m_s**2) * int(1e3)
            )  # Ground Speed (2-D) in mm/s
            msg.heading = int(
                np.degrees(cog) * int(1e5)
            )  # Heading of motion (2-D) in degrees * 1e-5

            msg.s_acc = int(
                s_variance_m_s * int(1e3)
            )  # Speed accuracy estimate in mm/s
            msg.head_acc = int(
                np.degrees(h_variance_rad) * int(1e5)
            )  # Heading accuracy estimate in degrees * 1e-5

            msg.p_dop = 0  # Position DOP * 0.01 (unitless)

            msg.head_veh = int(
                yaw_degrees * 100000
            )  # Heading of vehicle (2-D) in degrees * 1e-5
        except Exception as e:
            self.get_logger().warning(
                f"Could not create mock GPS message due to exception: {e}"
            )
            msg = None

        return msg

    def _unix_to_gps_time(self, unix_time):
        gps_epoch = 315964800  # GPS epoch in Unix time (1980-01-06 00:00:00 UTC)
        gps_time = unix_time - gps_epoch
        gps_week = int(gps_time / 604800)  # 604800 seconds in a week
        time_of_week = gps_time % 604800
        return gps_week, time_of_week

    def _get_utc_time(self, unix_time):
        utc_time = time.gmtime(unix_time)
        return (
            utc_time.tm_year,
            utc_time.tm_mon,
            utc_time.tm_mday,
            utc_time.tm_hour,
            utc_time.tm_min,
            utc_time.tm_sec,
        )
