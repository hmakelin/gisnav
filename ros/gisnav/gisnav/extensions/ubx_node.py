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
        @narrow_types(self)
        def _publish_inner(mock_gps_dict: MockGPSNode.MockGPSDict) -> None:
            self.nav_pvt(**mock_gps_dict)

        _publish_inner(mock_gps_dict)

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
        cog_rad: float,
        s_variance_m_s: float,
        timestamp: int,
        eph: float,
        epv: float,
        satellites_visible: int,
    ) -> Optional[NavPVT]:
        """Retusn UBX mock GPS message, or None if cannot be computed"""
        msg = NavPVT()

        try:
            # Convert timestamp to GPS time of week
            gps_week, time_of_week = self.unix_to_gps_time(
                timestamp / 1e6
            )  # Assuming timestamp is in microseconds

            msg.iTOW = int(time_of_week * 1000)  # GPS time of week in ms
            (
                msg.year,
                msg.month,
                msg.day,
                msg.hour,
                msg.min,
                msg.sec,
            ) = self.get_utc_time(timestamp / 1e6)

            msg.valid = (
                0x01 | 0x02 | 0x04
            )  # Assuming valid date, time, and fully resolved
            msg.tAcc = 50000000  # Time accuracy estimate in ns (50ms)
            msg.nano = 0  # Fraction of second, range -1e9 .. 1e9 (UTC)

            msg.fixType = 3  # 3D-Fix
            msg.flags = 0x01  # gnssFixOK
            msg.flags2 = 0
            msg.numSV = satellites_visible

            msg.lon = lon
            msg.lat = lat
            msg.height = int(
                altitude_ellipsoid * int(1e3)
            )  # Height above ellipsoid in mm
            msg.hMSL = int(
                altitude_amsl * int(1e3)
            )  # Height above mean sea level in mm
            msg.hAcc = int(eph * int(1e3))  # Horizontal accuracy estimate in mm
            msg.vAcc = int(epv * int(1e3))  # Vertical accuracy estimate in mm

            msg.velN = int(vel_n_m_s * int(1e3))  # NED north velocity in mm/s
            msg.velE = int(vel_e_m_s * int(1e3))  # NED east velocity in mm/s
            msg.velD = int(vel_d_m_s * int(1e3))  # NED down velocity in mm/s
            msg.gSpeed = int(
                np.sqrt(vel_n_m_s**2 + vel_e_m_s**2) * int(1e3)
            )  # Ground Speed (2-D) in mm/s
            msg.headMot = int(
                np.degrees(cog_rad) * int(1e5)
            )  # Heading of motion (2-D) in degrees * 1e-5

            msg.sAcc = int(s_variance_m_s * int(1e3))  # Speed accuracy estimate in mm/s
            msg.headAcc = int(
                np.degrees(h_variance_rad) * int(1e5)
            )  # Heading accuracy estimate in degrees * 1e-5

            msg.pDOP = 0  # Position DOP * 0.01 (unitless)

            msg.headVeh = int(
                yaw_degrees * 100000
            )  # Heading of vehicle (2-D) in degrees * 1e-5
        except AssertionError as e:
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
