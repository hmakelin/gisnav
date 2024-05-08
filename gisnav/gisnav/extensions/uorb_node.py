"""This module contains the :class:`.UORBNode` :term:`extension` :term:`node`
that publishes PC4 uORB SensorGps (GNSS) messages to the micro-ros agent middleware
"""
from typing import Final, Optional, Tuple

import numpy as np
import rclpy
import tf2_geometry_msgs
import tf2_ros
import tf_transformations
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from px4_msgs.msg import SensorGps
from pyproj import Transformer
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from .. import _transformations as tf_
from .._decorators import ROS, narrow_types
from ..constants import ROS_TOPIC_SENSOR_GPS

_ROS_PARAM_DESCRIPTOR_READ_ONLY: Final = ParameterDescriptor(read_only=True)
"""A read only ROS parameter descriptor"""


class UORBNode(Node):
    """A node that publishes a mock GPS message to autopilot middleware"""

    ROS_D_DEM_VERTICAL_DATUM = 5703
    """Default :term:`DEM` vertical datum"""

    # EPSG code for WGS 84 and a common mean sea level datum (e.g., EGM96)
    _EPSG_WGS84 = 4326
    _EPSG_MSL = 5773  # Example: EGM96

    def __init__(self, *args, **kwargs):
        """Class initializer

        :param args: Positional arguments to parent :class:`.Node` constructor
        :param kwargs: Keyword arguments to parent :class:`.Node` constructor
        """
        super().__init__(*args, **kwargs)

        self._mock_gps_pub = self.create_publisher(
            SensorGps,
            ROS_TOPIC_SENSOR_GPS,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

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
        """:term:`DEM` vertical datum (must match DEM that is published in
        :attr:`.GISNode.orthoimage`)
        """

    def _odometry_cb(self, msg: Odometry) -> None:
        """Callback for :attr:`.odometry`"""
        self._publish(msg)

    @property
    # @ROS.max_delay_ms(messaging.DELAY_SLOW_MS) - gst plugin does not enable timestamp?
    @ROS.subscribe(
        "/robot_localization/odometry/filtered",
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=_odometry_cb,
    )
    def odometry(self) -> Optional[Odometry]:
        """Camera info message including the camera intrinsics matrix"""

    def _publish(self, odometry: Odometry) -> None:
        @narrow_types(self)
        def _publish_inner(odometry: Odometry) -> None:
            pose = odometry.pose.pose
            # WGS 84 longitude and latitude, and AGL altitude in meters
            # TODO: this is alt above ellipsoid, not agl
            lon, lat, alt_agl = tf_.ecef_to_wgs84(
                pose.position.x, pose.position.y, pose.position.z
            )

            timestamp = tf_.usec_from_header(odometry.header)

            # Heading (yaw := z axis rotation) variance, assume no covariances
            pose_cov = odometry.pose.covariance.reshape((6, 6))
            std_dev_c_z = pose_cov[5, 5]
            h_variance_rad = std_dev_c_z**2

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
            satellites_visible = np.iinfo(np.uint8).max

            # Pose variance: eph (horizontal error SD) and epv (vertical error SD),
            # assume no covariances
            x_var = pose_cov[0, 0]
            y_var = pose_cov[1, 1]
            eph = np.sqrt(x_var + y_var)
            z_var = pose_cov[2, 2]
            epv = np.sqrt(z_var)

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
            vel_d_m_s = -linear_enu.z

            # Heading
            # vehicle_yaw_degrees = tf_.extract_yaw(pose.orientation)
            # TODO: should be able to use the stamp here or extrapolate? instead
            #  of getting latest
            transform_earth_to_map = tf_.get_transform(
                self, "map", "earth", rclpy.time.Time()  # odometry.header.stamp
            )

            if transform_earth_to_map is None:
                self.get_logger().warning(
                    "Could not determine heading - skpping publishing SensorGps."
                )
                # TODO Set yaw to zero and make yaw/heading accuracy
                #  np.nan in this case?
                return None

            pose_map = tf2_geometry_msgs.do_transform_pose(pose, transform_earth_to_map)
            euler = tf_transformations.euler_from_quaternion(
                tf_.as_np_quaternion(pose_map.orientation).tolist()
            )
            yaw_rad = euler[2]  # ENU frame
            yaw_rad = -yaw_rad  # NED frame ("heading")

            if yaw_rad < 0:
                yaw_rad = 2 * np.pi + yaw_rad

            # re-center yaw to [0, 2*pi), it should be at [-pi, pi) before re-centering
            vehicle_yaw_degrees = np.degrees(yaw_rad)
            vehicle_yaw_degrees = int(vehicle_yaw_degrees % 360)
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
            s_variance_m_s = vel_n_m_s_var + vel_e_m_s_var + vel_d_m_s_var

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
                """
                Calculate the course over ground from east and north velocities.

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
            cog = _calculate_course_over_ground(vel_e_m_s, vel_n_m_s)

            # Compute course over ground variance
            cog_variance_rad = _calculate_cog_variance(
                vel_n_m_s, vel_e_m_s, vel_n_m_s_var, vel_e_m_s_var
            )

            self.sensor_gps(
                int(lat * 1e7),
                int(lon * 1e7),
                alt_ellipsoid,
                alt_amsl,
                vehicle_yaw_degrees,
                h_variance_rad,
                vel_n_m_s,
                vel_e_m_s,
                vel_d_m_s,
                cog,
                cog_variance_rad,
                s_variance_m_s,
                timestamp,
                eph,
                epv,
                satellites_visible,
            )

        _publish_inner(odometry)

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
        """Outgoing mock :term:`GNSS` :term:`message` when :attr:`use_sensor_gps`
        is ``True``

        Uses the release/1.14 tag version of :class:`px4_msgs.msg.SensorGps`
        """
        yaw_rad = np.radians(yaw_degrees)

        msg = SensorGps()
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

    @narrow_types
    def _convert_to_wgs84(
        self, lat: float, lon: float, elevation: float
    ) -> Optional[Tuple[float, float]]:
        """
        Convert :term:`elevation` or :term:`altitude` from a specified vertical
        datum to :term:`WGS 84`.

        :param lat: Latitude in decimal degrees.
        :param lon: Longitude in decimal degrees.
        :param elevation: Elevation in the specified datum.
        :return: A tuple containing :term:`elevation` above :term:`WGS 84` and
            :term:`AMSL`.
        """
        _, _, wgs84_elevation = self._transformer_to_wgs84.transform(
            lon, lat, elevation
        )
        _, _, msl_elevation = self._transformer_to_msl.transform(lon, lat, elevation)

        return wgs84_elevation, msl_elevation
