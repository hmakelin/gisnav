"""This module contains :class:`.NMEANode`, an extension ROS node that publishes
mock GPS (GNSS) messages to FCU over the NMEA protocol via a serial port
"""
import warnings
from datetime import datetime
from typing import Final, List, Optional, Tuple

import numpy as np
import pynmea2
import rclpy
import serial
import tf2_geometry_msgs
import tf2_ros
import tf_transformations
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from pyproj import Transformer
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from .. import _transformations as tf_
from .._decorators import ROS, narrow_types
from ..constants import ROS_TOPIC_ROBOT_LOCALIZATION_ODOMETRY

_ROS_PARAM_DESCRIPTOR_READ_ONLY: Final = ParameterDescriptor(read_only=True)
"""A read only ROS parameter descriptor"""

warnings.warn(
    "NMEANode is deprecated. Use UORBNode or UBloxNode instead.",
    DeprecationWarning,
)


class NMEANode(Node):
    """Publishes mock GPS messages to FCU over NMEA protocol via a serial port"""

    ROS_D_DEM_VERTICAL_DATUM = 5703
    """Default for :attr:`.dem_vertical_datum`"""

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

    @property
    @ROS.parameter(ROS_D_DEM_VERTICAL_DATUM, descriptor=_ROS_PARAM_DESCRIPTOR_READ_ONLY)
    def dem_vertical_datum(self) -> Optional[int]:
        """DEM vertical datum

        > [!IMPORTANT]
        > Must match DEM that is published in :attr:`.GISNode.orthoimage`
        """

    @property
    @ROS.parameter(ROS_D_PORT, descriptor=_ROS_PARAM_DESCRIPTOR_READ_ONLY)
    def port(self) -> Optional[str]:
        """Serial port for outgoing NMEA messages"""

    @property
    @ROS.parameter(ROS_D_BAUDRATE, descriptor=_ROS_PARAM_DESCRIPTOR_READ_ONLY)
    def baudrate(self) -> Optional[int]:
        """Baudrate for outgoing NMEA messages"""

    def _odometry_cb(self, msg: Odometry) -> None:
        """Callback for :attr:`.odometry`"""
        self._publish(msg)

    @property
    @ROS.subscribe(
        ROS_TOPIC_ROBOT_LOCALIZATION_ODOMETRY,
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=_odometry_cb,
    )
    def odometry(self) -> Optional[Odometry]:
        """Subscribed filtered odometry from ``robot_localization`` package EKF node,
        or None if unknown"""

    def _publish(self, odometry: Odometry) -> None:
        @narrow_types(self)
        def _publish_inner(odometry: Odometry) -> None:
            pose = odometry.pose.pose
            # WGS 84 longitude and latitude, and AGL altitude in meters
            # TODO: this is alt above ellipsoid, not agl
            lon, lat, alt_agl = tf_.ecef_to_wgs84(
                pose.position.x, pose.position.y, pose.position.z
            )

            alt_agl += (
                3  # adjust for ellipsoid amsl for SITL TODO publish over nmea instead
            )

            timestamp = tf_.usec_from_header(odometry.header)
            # timestamp = int(time.time() * 1e6)

            # Heading (yaw := z axis rotation) variance, assume no covariances
            pose_cov = odometry.pose.covariance.reshape((6, 6))
            std_dev_c_z = pose_cov[5, 5]
            std_dev_c_z**2

            # WGS 84 ellipsoid and AMSL altitudes
            altitudes = self._convert_to_wgs84(
                lat,
                lon,
                alt_agl,
            )
            if altitudes is not None:
                alt_ellipsoid, alt_amsl = altitudes
            else:
                return None

            # Make satellites_visible value unrealistic but technically valid to make
            # GISNav generated mock GPS messages easy to identify. Do not make this
            # zero because the messages might then get rejected because of too low
            # satellite count.
            np.iinfo(np.uint8).max

            # Pose variance: eph (horizontal error SD) and epv (vertical error SD),
            # assume no covariances
            x_var = pose_cov[0, 0]
            y_var = pose_cov[1, 1]
            eph = np.sqrt(x_var + y_var)
            z_var = pose_cov[2, 2]
            epv = np.sqrt(z_var)
            rms = np.sqrt(x_var + y_var + z_var)

            # 3D velocity
            # Twist in ENU -> remap to NED here by swapping x and y axes and inverting
            # z axis
            # TODO: map is not published by GISNav - should use an ENU map frame
            #  published by gisnav instead
            twist = odometry.twist.twist

            # TODO: should be able to use the stamp here or extrapolate? instead
            #  of getting latest
            transform = tf_.get_transform(
                self,
                "map",
                "camera_optical",
                rclpy.time.Time(),  # odometry.header.stamp
            )

            # Need to convert linear twist vector to stamped version becase
            # do_transform_vector3 expects stamped
            vector_stamped = Vector3Stamped(header=odometry.header, vector=twist.linear)
            vector_stamped.header.frame_id = odometry.child_frame_id
            if transform is None:
                # TODO: do this better
                return None
            linear_enu = tf2_geometry_msgs.do_transform_vector3(
                vector_stamped, transform
            )
            linear_enu = linear_enu.vector
            vel_n_m_s = linear_enu.y
            vel_e_m_s = linear_enu.x
            # vel_d_m_s = -linear_enu.z

            # Heading
            vehicle_yaw_degrees = tf_.extract_yaw(pose.orientation)
            # TODO: should be able to use the stamp here or extrapolate? instead
            #  of getting latest
            transform_earth_to_map = tf_.get_transform(
                self, "map", "earth", rclpy.time.Time()  # odometry.header.stamp
            )

            if transform_earth_to_map is not None:
                pose_map = tf2_geometry_msgs.do_transform_pose(
                    pose, transform_earth_to_map
                )
                euler = tf_transformations.euler_from_quaternion(
                    tf_.as_np_quaternion(pose_map.orientation).tolist()
                )
                yaw_rad = euler[2]  # ENU frame
                yaw_rad = -yaw_rad  # NED frame ("heading")

                if yaw_rad < 0:
                    yaw_rad = 2 * np.pi + yaw_rad

                # re-center yaw to [0, 2*pi), it should be at [-pi, pi) before
                # re-centering
                vehicle_yaw_degrees = np.degrees(yaw_rad)
                # vehicle_yaw_degrees = int(vehicle_yaw_degrees % 360)
                # MAVLink yaw definition 0 := not available
                vehicle_yaw_degrees = (
                    360 if vehicle_yaw_degrees == 0 else vehicle_yaw_degrees
                )

            # Speed variance, assume no covariances
            twist_cov = odometry.twist.covariance.reshape((6, 6))
            # Twist in ENU -> remap to NED here by swapping x and y axes, z axis
            # inversion should not affect variance
            vel_n_m_s_var = twist_cov[1, 1]
            vel_e_m_s_var = twist_cov[0, 0]
            vel_d_m_s_var = twist_cov[2, 2]
            vel_n_m_s_var + vel_e_m_s_var + vel_d_m_s_var

            # Course over ground and its variance
            def _calculate_cog_variance(
                vel_n_m_s, vel_e_m_s, vel_n_m_s_var, vel_e_m_s_var
            ) -> float:
                numerator = (vel_e_m_s_var * vel_n_m_s**2) + (
                    vel_n_m_s_var * vel_e_m_s**2
                )
                denominator = (vel_e_m_s**2 + vel_n_m_s**2) ** 2

                # Calculate the variance of the CoG in radians
                cog_var = numerator / denominator

                # TODO handle possible exceptions arising from variance exploding at 0
                #  velocity (as it should)
                return float(cog_var)

            def _calculate_course_over_ground(
                east_velocity: float, north_velocity: float
            ) -> float:
                """Calculates course over ground from east and north velocities.

                :param east_velocity: The velocity towards the east in meters per
                    second.
                :param north_velocity: The velocity towards the north in meters per
                    second.
                :return: The course over ground in degrees from the north, in the range
                    [0, 2 * pi).

                The course over ground is calculated using the arctangent of the east
                and north velocities. The result is adjusted to ensure it is within
                the [0, 2 * pi) range.
                """
                # TODO: do not send this sentence if the variance is too high (i.e.
                #  vehicle is not moving)
                magnitude = np.sqrt(east_velocity**2 + north_velocity**2)

                if east_velocity >= 0 and north_velocity >= 0:
                    # top-right quadrant
                    course_over_ground_radians = np.arcsin(east_velocity / magnitude)
                elif east_velocity >= 0 and north_velocity < 0:
                    # bottom-right quadrant
                    course_over_ground_radians = 0.5 * np.pi + np.arcsin(
                        -north_velocity / magnitude
                    )
                elif east_velocity < 0 and north_velocity < 0:
                    # bottom-left quadrant
                    course_over_ground_radians = np.pi + np.arcsin(
                        -east_velocity / magnitude
                    )
                elif east_velocity < 0 and north_velocity >= 0:
                    # top-left quadrant
                    course_over_ground_radians = 1.5 * np.pi + np.arcsin(
                        north_velocity / magnitude
                    )
                else:
                    # todo: this is unreachable?
                    course_over_ground_radians = 0.0

                return course_over_ground_radians

            # Compute course over ground - pay attention to sine only being
            # defined for 0<=theta<=90

            # Compute course over ground variance
            # cog_variance_rad = _calculate_cog_variance(
            #    vel_n_m_s, vel_e_m_s, vel_n_m_s_var, vel_e_m_s_var
            # )
            # _calculate_course_over_ground(vel_e_m_s, vel_n_m_s)

            nmea_sentences = self.nmea_sentences(
                lat,
                lon,
                alt_amsl,
                timestamp,
                vel_n_m_s,
                vel_e_m_s,
                vehicle_yaw_degrees,
                0.0,  # eph,  # TODO: fix
                0.0,  # eph,
                0.0,  # epv,
                odometry,
                rms,
                eph,
                np.sqrt(x_var),
                np.sqrt(y_var),
                epv,
            )
            if nmea_sentences is not None:
                self._write_nmea_to_serial(nmea_sentences)

        _publish_inner(odometry)

    def compute_rmc_parameters(
        self, odometry: Odometry
    ) -> Tuple[str, str, str, str, str, str, float, float, str]:
        """Calculates RMC parameters based on odometry data.

        :param odometry: The odometry data from which to extract GPS details.
        :returns: A tuple with formatted time, status, latitude, latitude direction,
                  longitude, longitude direction, speed, course, and date.
        """
        # Assume position and timestamp extraction as before
        pose = odometry.pose.pose
        lon, lat, _ = tf_.ecef_to_wgs84(
            pose.position.x, pose.position.y, pose.position.z
        )
        timestamp = tf_.usec_from_header(odometry.header)
        # timestamp = int(time.time() * 1e6)
        time_str = self.format_time_from_timestamp(timestamp)
        date_str = self.format_date_from_timestamp(timestamp)

        # Convert speed from m/s to knots
        twist = odometry.twist.twist
        speed_knots: float = (
            np.sqrt(twist.linear.x**2 + twist.linear.y**2) * 1.94384
        )  # m/s to knots

        # Calculate course over ground in degrees
        course_degrees: float = (
            np.degrees(np.arctan2(twist.linear.y, twist.linear.x)) % 360
        )

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
            speed_knots,
            course_degrees,
            date_str,
        )

    @narrow_types
    def nmea_sentences(
        self,
        lat_deg: float,
        lon_deg: float,
        altitude_amsl: float,
        timestamp: int,
        vel_n_m_s: float,
        vel_e_m_s: float,
        yaw_degrees: float,
        pdop: float,
        hdop: float,
        vdop: float,
        odometry: Odometry,
        rms: float,
        eph: float,
        sd_x: float,
        sd_y: float,
        sd_z: float,
    ) -> Optional[List[str]]:
        """Outgoing NMEA mock GPS sentences

        Published sentences:
        - GGA (position fix)
        - GSA (position fix)
        - HDT (heading)
        - GST (standard deviations)
        - VTG (velocities, disabled)
        - RMC (velocities, disabled)
        """
        # Convert timestamp to hhmmss format
        time_str = self.format_time_from_timestamp(timestamp)
        self.format_date_from_timestamp(timestamp)

        # Format latitude and longitude for NMEA
        lat_nmea = self._decimal_to_nmea(lat_deg)
        lon_nmea = self._decimal_to_nmea(lon_deg)
        lat_dir = "N" if lat_deg >= 0 else "S"
        lon_dir = "E" if lon_deg >= 0 else "W"

        # Calculate ground speed in knots and course over ground
        # ground_speed_knots = (
        #    np.sqrt(vel_n_m_s**2 + vel_e_m_s**2) * 1.94384
        # )  # m/s to knots

        # The PX4 nmea.cpp driver sets s_variance_m_s to 0 if we publish velocity,
        # which will inevitable lead to failsafes triggering when the simulated GPS
        # is turned off (nsh$ failure gps -i 0 off), so we comment the VTG and RMC
        # messages out for now to not publish velocity
        self.compute_rmc_parameters(odometry)
        return [
            self.GGA(
                time_str, lat_nmea, lat_dir, lon_nmea, lon_dir, altitude_amsl, hdop
            ),
            # self.VTG(cog_degrees, ground_speed_knots),
            self.GSA(pdop, hdop, vdop),
            self.HDT(yaw_degrees),
            self.GST(time_str, rms, eph, eph, 0.0, sd_y, sd_x, sd_z),
            # self.RMC(*rmc_params),
            self.GSV,
        ]

    @staticmethod
    def GGA(
        time_str: str,
        lat_nmea: str,
        lat_dir: str,
        lon_nmea: str,
        lon_dir: str,
        altitude_amsl: float,
        hdop: float,
    ) -> str:
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
        return str(
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
        )

    @staticmethod
    def VTG(cog_degrees: float, ground_speed_knots: float) -> str:
        """Returns an NMEA GPVTG sentence

        :param cog_degrees: Course over ground in degrees.
        :param ground_speed_knots: Speed over ground in knots.
        :returns: A formatted NMEA VTG sentence as a string.
        """
        return str(
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
        )

    @staticmethod
    def GSA(pdop: float, hdop: float, vdop: float) -> str:
        """Returns an NMEA GPGSA sentence

        :param pdop: Positional dilution of precision.
        :param hdop: Horizontal dilution of precision.
        :param vdop: Vertical dilution of precision.
        :returns: A formatted NMEA GSA sentence as a string.
        """
        return str(
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
        )

    @staticmethod
    def HDT(yaw_deg: float):
        """Returns an NMEA GPHDT sentence

        :param yaw_deg: Vehicle heading in degrees. Heading increases "clockwise" so
            that north is 0 degrees and east is 90 degrees.
        :returns: A formatted NMEA HDT sentence as a string.
        """
        return str(pynmea2.HDT("GP", "HDT", (f"{yaw_deg:.1f}", "T")))

    @staticmethod
    def GST(
        time_str: str,
        rms_deviation: float,
        std_dev_major_axis: float,
        std_dev_minor_axis: float,
        orientation_major_axis: float,
        std_dev_latitude: float,
        std_dev_longitude: float,
        std_dev_altitude: float,
    ) -> str:
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
        return str(
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
        )

    @staticmethod
    def RMC(
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
    ) -> str:
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
        return str(
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

    @narrow_types
    def _convert_to_wgs84(
        self, lat: float, lon: float, elevation: float
    ) -> Optional[Tuple[float, float]]:
        """Converts elevation or altitude from :attr:`.dem_vertical_datum` to WGS 84.

        :param lat: Latitude in decimal degrees.
        :param lon: Longitude in decimal degrees.
        :param elevation: Elevation in the specified datum.
        :return: A tuple containing elevation above WGS 84 ellipsoid and AMSL.
        """
        _, _, wgs84_elevation = self._transformer_to_wgs84.transform(
            lon, lat, elevation
        )
        _, _, msl_elevation = self._transformer_to_msl.transform(lon, lat, elevation)

        return wgs84_elevation, msl_elevation

    def _write_nmea_to_serial(self, nmea_sentences: List[str]) -> None:
        """Writes a collection of NMEA sentences to :attr:`.port`.

        :param nmea_sentences: A list of NMEA sentences to be written to the serial port
        """
        with serial.Serial(self.port, self.baudrate, timeout=1) as ser:
            for sentence in nmea_sentences:
                ser.write((sentence + "\r\n").encode())

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

    def ZDA(
        self, time_zone_hour_offset: int = 0, time_zone_minute_offset: int = 0
    ) -> str:
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
        return str(zda)

    @property
    def GSV(self) -> str:
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

        return "\r\n".join(messages)
