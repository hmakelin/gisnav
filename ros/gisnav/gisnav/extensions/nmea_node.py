"""This module contains :class:`.NMEANode`, an extension ROS node that publishes
mock GPS (GNSS) messages as NMEA sentences to ROS
"""
from datetime import datetime
from typing import Final, Tuple

import numpy as np
import pynmea2
import rclpy.time
import tf2_ros
from nmea_msgs.msg import Sentence
from pyproj import Transformer
from rcl_interfaces.msg import ParameterDescriptor
from std_msgs.msg import Header

from .._decorators import ROS
from ..constants import ROS_TOPIC_RELATIVE_NMEA_SENTENCE
from ._mock_gps_node import MockGPSNode

_ROS_PARAM_DESCRIPTOR_READ_ONLY: Final = ParameterDescriptor(read_only=True)
"""A read only ROS parameter descriptor"""


class NMEANode(MockGPSNode):
    """Publishes mock GPS messages to FCU over NMEA protocol via a serial port"""

    ROS_D_DEM_VERTICAL_DATUM = 5703
    """Default for :attr:`.dem_vertical_datum`"""

    # EPSG code for WGS 84 and a common mean sea level datum (e.g., EGM96)
    _EPSG_WGS84 = 4326
    _EPSG_MSL = 5773  # Example: EGM96

    def __init__(self, *args, **kwargs):
        """Class initializer

        :param args: Positional arguments to parent :class:`.Node` constructor
        :param kwargs: Keyword arguments to parent :class:`.Node` constructor
        """
        super().__init__(*args, **kwargs)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Create transformers
        self._transformer_to_wgs84 = Transformer.from_crs(
            f"EPSG:{self.dem_vertical_datum}",
            f"EPSG:{self._EPSG_WGS84}",
            always_xy=True,
        )
        self._transformer_to_msl = Transformer.from_crs(
            f"EPSG:{self.dem_vertical_datum}", f"EPSG:{self._EPSG_MSL}", always_xy=True
        )

        # Subscribe
        self.odometry

    def _publish(self, mock_gps_dict: MockGPSNode.MockGPSDict) -> None:
        eph_sqrd = mock_gps_dict["eph"] ** 2
        epv_sqrd = mock_gps_dict["epv"] ** 2

        self.publish_nmea_sentences(
            rms=np.sqrt(eph_sqrd + epv_sqrd),
            sd_x=np.sqrt(eph_sqrd / 2),
            sd_y=np.sqrt(eph_sqrd / 2),
            sd_z=epv_sqrd,
            **mock_gps_dict,
        )

    def compute_rmc_parameters(
        self,
        timestamp,
        lat,
        lon,
        ground_speed_knots,
        cog,
    ) -> Tuple[str, str, str, str, str, str, float, float, str]:
        """Calculates RMC parameters based on odometry data.

        :returns: A tuple with formatted time, status, latitude, latitude direction,
                  longitude, longitude direction, speed, course, and date.
        """
        # timestamp = int(time.time() * 1e6)
        time_str = self.format_time_from_timestamp(timestamp)
        date_str = self.format_date_from_timestamp(timestamp)

        lat_nmea = self._decimal_to_nmea(lat)
        lon_nmea = self._decimal_to_nmea(lon)
        lat_dir = "N" if lat >= 0 else "S"
        lon_dir = "E" if lon >= 0 else "W"

        # Active or void status (assuming active if we have valid data)
        status = "A" if lat and lon else "V"

        return (
            time_str,
            status,
            lat_nmea,
            lat_dir,
            lon_nmea,
            lon_dir,
            ground_speed_knots,
            np.degrees(cog),
            date_str,
        )

    def publish_nmea_sentences(
        self,
        lat: float,
        lon: float,
        altitude_amsl: float,
        timestamp: int,
        vel_n_m_s: float,
        vel_e_m_s: float,
        yaw_degrees: int,
        cog: float,
        rms: float,
        eph: float,
        sd_x: float,
        sd_y: float,
        sd_z: float,
        **kwargs,
    ) -> None:
        """Outgoing NMEA mock GPS sentences

        Published sentences:
        - GGA (position fix)
        - GSA (position fix)
        - HDT (heading)
        - GST (standard deviations)
        - VTG (velocities, disabled)
        - RMC (velocities, disabled)
        """
        pdop, hdop, vdop = 0.0, 0.0, 0.0  # not relevant for GISNav

        lat_deg, lon_deg = lat / 1e7, lon / 1e7
        # Convert timestamp to hhmmss format
        time_str = self.format_time_from_timestamp(timestamp)
        self.format_date_from_timestamp(timestamp)

        # Format latitude and longitude for NMEA
        lat_nmea = self._decimal_to_nmea(lat_deg)
        lon_nmea = self._decimal_to_nmea(lon_deg)
        lat_dir = "N" if lat_deg >= 0 else "S"
        lon_dir = "E" if lon_deg >= 0 else "W"

        # Calculate ground speed in knots and course over ground
        ground_speed_knots = (
            np.sqrt(vel_n_m_s**2 + vel_e_m_s**2) * 1.94384
        )  # m/s to knots

        # The PX4 nmea.cpp driver sets s_variance_m_s to 0 if we publish velocity,
        # which will inevitable lead to failsafes triggering when the simulated GPS
        # is turned off (nsh$ failure gps -i 0 off), so we comment the VTG and RMC
        # messages out for now to not publish velocity
        header = Header()
        header.stamp = rclpy.time.Time().to_msg()
        header.frame_id = "base_link"
        rmc_params = self.compute_rmc_parameters(
            timestamp, lat, lon, ground_speed_knots, cog
        )
        self.GGA(
            header, time_str, lat_nmea, lat_dir, lon_nmea, lon_dir, altitude_amsl, hdop
        )
        self.VTG(header, np.degrees(cog), ground_speed_knots)
        self.GSA(header, pdop, hdop, vdop)
        self.HDT(header, float(yaw_degrees))
        self.GST(header, time_str, rms, eph, eph, 0.0, sd_y, sd_x, sd_z)
        self.RMC(header, *rmc_params)
        self.GSV(header)

    @ROS.publish(
        ROS_TOPIC_RELATIVE_NMEA_SENTENCE, 10
    )  # QoSPresetProfiles.SENSOR_DATA.value,
    def GGA(
        self,
        header,
        time_str: str,
        lat_nmea: str,
        lat_dir: str,
        lon_nmea: str,
        lon_dir: str,
        altitude_amsl: float,
        hdop: float,
    ) -> Sentence:
        """Returns an NMEA GPGGA sentence

        :param time_str: UTC time in HHMMSS format.
        :param lat_nmea: Latitude in decimal degrees.
        :param lat_dir: Latitude hemisphere (N for North, S for South).
        :param lon_nmea: Longitude in decimal degrees.
        :param lon_dir: Longitude hemisphere (E for East, W for West).
        :param altitude_amsl: Altitude above mean sea level in meters.
        :param hdop: Horizontal dilution of precision.
        :returns: A formatted NMEA GGA sentence as a string.
        """
        sentence = Sentence(
            header=header,
            sentence=str(
                pynmea2.GGA(
                    "GP",
                    "GGA",
                    (
                        time_str,
                        lat_nmea,
                        lat_dir,
                        lon_nmea,
                        lon_dir,
                        "1",
                        "12",
                        f"{hdop:.2f}",
                        f"{altitude_amsl:.1f}",
                        "M",
                        "0.0",  # TODO: geoid altitude at sea level - important
                        "M",
                        "",
                        "",
                    ),
                )
            ),
        )
        return sentence

    @ROS.publish(
        ROS_TOPIC_RELATIVE_NMEA_SENTENCE, 10
    )  # QoSPresetProfiles.SENSOR_DATA.value,
    def VTG(self, header, cog_degrees: float, ground_speed_knots: float) -> Sentence:
        """Returns an NMEA GPVTG sentence

        :param cog_degrees: Course over ground in degrees.
        :param ground_speed_knots: Speed over ground in knots.
        :returns: A formatted NMEA VTG sentence as a string.
        """
        return Sentence(
            header=header,
            sentence=str(
                pynmea2.VTG(
                    "GP",
                    "VTG",
                    (
                        f"{cog_degrees:.1f}",
                        "T",
                        "",
                        "M",
                        f"{ground_speed_knots:.1f}",
                        "N",
                        "",
                        "K",
                    ),
                )
            ),
        )

    @ROS.publish(
        ROS_TOPIC_RELATIVE_NMEA_SENTENCE, 10
    )  # QoSPresetProfiles.SENSOR_DATA.value,
    def GSA(self, header, pdop: float, hdop: float, vdop: float) -> Sentence:
        """Returns an NMEA GPGSA sentence

        :param pdop: Positional dilution of precision.
        :param hdop: Horizontal dilution of precision.
        :param vdop: Vertical dilution of precision.
        :returns: A formatted NMEA GSA sentence as a string.
        """
        return Sentence(
            header=header,
            sentence=str(
                pynmea2.GSA(
                    "GP",
                    "GSA",
                    (
                        "A",
                        "3",  # Mode: A=Automatic, 3=3D fix
                        *[str(sat).zfill(2) for sat in range(12)],  # 12 satellites
                        f"{pdop:.2f}",
                        f"{hdop:.2f}",
                        f"{vdop:.2f}",
                    ),
                )
            ),
        )

    @ROS.publish(
        ROS_TOPIC_RELATIVE_NMEA_SENTENCE, 10
    )  # QoSPresetProfiles.SENSOR_DATA.value,
    def HDT(self, header, yaw_deg: float) -> Sentence:
        """Returns an NMEA GPHDT sentence

        :param yaw_deg: Vehicle heading in degrees. Heading increases "clockwise" so
            that north is 0 degrees and east is 90 degrees.
        :returns: A formatted NMEA HDT sentence as a string.
        """
        return Sentence(
            header=header,
            sentence=str(pynmea2.HDT("GP", "HDT", (f"{yaw_deg:.1f}", "T"))),
        )

    @ROS.publish(
        ROS_TOPIC_RELATIVE_NMEA_SENTENCE, 10
    )  # QoSPresetProfiles.SENSOR_DATA.value,
    def GST(
        self,
        header,
        time_str: str,
        rms_deviation: float,
        std_dev_major_axis: float,
        std_dev_minor_axis: float,
        orientation_major_axis: float,
        std_dev_latitude: float,
        std_dev_longitude: float,
        std_dev_altitude: float,
    ) -> Sentence:
        """Returns an NMEA GPGST sentence.

        :param time_str: UTC time in hhmmss format.
        :param rms_deviation: RMS deviation of the pseudorange.
        :param std_dev_major_axis: Standard deviation of the semi-major axis.
        :param std_dev_minor_axis: Standard deviation of the semi-minor axis.
        :param orientation_major_axis: Orientation of the semi-major axis.
        :param std_dev_latitude: Standard deviation of latitude error.
        :param std_dev_longitude: Standard deviation of longitude error.
        :param std_dev_altitude: Standard deviation of altitude error.
        :returns: A formatted NMEA GST sentence as a string.
        """
        return Sentence(
            header=header,
            sentence=str(
                pynmea2.GST(
                    "GP",
                    "GST",
                    (
                        time_str,
                        f"{rms_deviation:.2f}",
                        f"{std_dev_major_axis:.2f}",
                        f"{std_dev_minor_axis:.2f}",
                        f"{orientation_major_axis:.1f}",
                        f"{std_dev_latitude:.2f}",
                        f"{std_dev_longitude:.2f}",
                        f"{std_dev_altitude:.2f}",
                    ),
                )
            ),
        )

    @ROS.publish(
        ROS_TOPIC_RELATIVE_NMEA_SENTENCE, 10
    )  # QoSPresetProfiles.SENSOR_DATA.value,
    def RMC(
        self,
        header,
        time_str: str,
        status: str,
        lat_nmea: str,
        lat_dir: str,
        lon_nmea: str,
        lon_dir: str,
        speed_knots: float,
        course_degrees: float,
        date_str: str,
        magnetic_variation: float = 0,
        var_dir: str = "E",
    ) -> Sentence:
        """Returns an NMEA GPRMC sentence.

        :param time_str: UTC time in hhmmss format.
        :param status: Status, 'A' for active or 'V' for void.
        :param lat_nmea: Latitude in NMEA format.
        :param lat_dir: Latitude hemisphere (N for North, S for South).
        :param lon_nmea: Longitude in NMEA format.
        :param lon_dir: Longitude hemisphere (E for East, W for West).
        :param speed_knots: Speed over ground in knots.
        :param course_degrees: Course over ground in degrees.
        :param date_str: Date in ddmmyy format.
        :param magnetic_variation: Magnetic variation in degrees (optional).
        :param var_dir: Direction of magnetic variation, 'E' or 'W' (optional).
        :returns: A formatted NMEA RMC sentence as a string.
        """
        return Sentence(
            header=header,
            sentence=str(
                pynmea2.RMC(
                    "GP",
                    "RMC",
                    (
                        time_str,
                        status,
                        lat_nmea,
                        lat_dir,
                        lon_nmea,
                        lon_dir,
                        f"{speed_knots:.1f}",
                        f"{course_degrees:.1f}",
                        date_str,
                        f"{magnetic_variation:.1f}",
                        var_dir,
                    ),
                )
            ),
        )

    def format_time_from_timestamp(self, timestamp: int) -> str:
        """Helper function to convert a POSIX timestamp to a time string in
        hhmmss.SSS format, where SSS is milliseconds.

        :param timestamp: Timestamp in microseconds
        """
        dt = datetime.fromtimestamp(timestamp / 1e6)
        return dt.strftime("%H%M%S.%f")[:10]

    def format_date_from_timestamp(self, timestamp: int) -> str:
        """Helper function to convert a POSIX timestamp to a date string in
        YYMMDD format.

        :param timestamp: Timestamp in microseconds
        """
        dt = datetime.fromtimestamp(timestamp / 1e6)
        return dt.strftime("%y%m%d")

    @staticmethod
    def _decimal_to_nmea(degrees: float) -> str:
        """Convert decimal degree to NMEA format (d)ddmm.mmmm where d is degrees and m
        is minutes (ignores sign of degrees)

        :param degrees: Decimal degree representation.
        :return: A string representing the degrees in NMEA format.
        """
        # Separate the degrees into the integer part and fractional part
        d = int(degrees)
        m = abs(degrees - d) * 60
        return f"{abs(d):02d}{m:07.4f}"

    @ROS.publish(
        ROS_TOPIC_RELATIVE_NMEA_SENTENCE, 10
    )  # QoSPresetProfiles.SENSOR_DATA.value,
    def ZDA(
        self, header, time_zone_hour_offset: int = 0, time_zone_minute_offset: int = 0
    ) -> Sentence:
        utc_now = datetime.utcnow()
        zda = pynmea2.ZDA(
            "GP",
            "ZDA",
            (
                utc_now.strftime("%H%M%S"),
                utc_now.strftime("%d"),
                utc_now.strftime("%m"),
                utc_now.strftime("%Y"),
                str(time_zone_hour_offset),
                str(time_zone_minute_offset),
            ),
        )
        return Sentence(header=header, sentence=str(zda))

    @ROS.publish(
        ROS_TOPIC_RELATIVE_NMEA_SENTENCE, 10
    )  # QoSPresetProfiles.SENSOR_DATA.value,
    def GSV(self, header) -> Sentence:
        """Returns NMEA GPGSV sentences for 12 statically defined dummy satellites.

        :returns: A formatted NMEA GSV sentences as a string.
        """

        def create_gsv_message(total_msgs, msg_num, sats_in_view, sat_info):
            """
            Creates a GSV message with the specified satellite information.

            :param total_msgs: Total number of GSV messages necessary to report all
                satellites
            :param msg_num: The current message number (1-based)
            :param sats_in_view: Total number of satellites in view
            :param sat_info: List of tuples for each satellite in this message, each
                tuple is (sat_id, elevation, azimuth, snr)
            :return: GSV NMEA sentence as a string
            """
            msg = pynmea2.GSV(
                "GP",
                "GSV",
                (str(total_msgs), str(msg_num), str(sats_in_view)) + sat_info,
            )
            return msg.render()

        # dummy satellite data
        satellites = [
            ("01", "85", "000", "99"),
            ("02", "85", "030", "99"),
            ("03", "85", "060", "99"),
            ("04", "85", "090", "99"),
            ("05", "85", "120", "99"),
            ("06", "85", "150", "99"),
            ("07", "85", "180", "99"),
            ("08", "85", "210", "99"),
            ("09", "85", "240", "99"),
            ("10", "85", "270", "99"),
            ("11", "85", "300", "99"),
            ("12", "85", "330", "99"),
        ]

        # Split into messages containing 1 satellite each
        total_messages = len(satellites) // 1 + (1 if len(satellites) % 1 != 0 else 0)
        messages = [
            create_gsv_message(total_messages, i + 1, 12, satellites[i])
            for i in range(total_messages)
        ]

        return Sentence(header=header, sentence="\r\n".join(messages))
