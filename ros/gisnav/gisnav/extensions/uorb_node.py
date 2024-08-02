"""This module contains :class:`.UORBNode`, an extension ROS node that publishes PX4
uORB :class:`.SensorGps` (GNSS) messages to the uXRCE-DDS middleware
"""
from typing import Final, Optional, Tuple

import numpy as np
import rclpy
import tf2_geometry_msgs
import tf2_ros
import tf_transformations
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PointStamped, TwistWithCovariance, Vector3
from nav_msgs.msg import Odometry
from px4_msgs.msg import SensorGps
from pyproj import Transformer
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from .. import _transformations as tf_
from .._decorators import ROS, narrow_types
from ..constants import ROS_TOPIC_ROBOT_LOCALIZATION_ODOMETRY, ROS_TOPIC_SENSOR_GPS

_ROS_PARAM_DESCRIPTOR_READ_ONLY: Final = ParameterDescriptor(read_only=True)
"""A read only ROS parameter descriptor"""


class UORBNode(Node):
    """A node that publishes PX4 uORB :class:`.SensorGps` (GNSS) messages to the
    uXRCE-DDS middleware"""

    ROS_D_DEM_VERTICAL_DATUM = 5703
    """Default for :attr:`.dem_vertical_datum`"""

    _REQUIRED_ODOMETRY_MESSAGES_BEFORE_PUBLISH = 10
    """Number of required odometry messages before we start publishing

    This gives some time for the internal state of the EKF to catch up with the actual
    state in case it starts from zero. Ideally we should be able to initialize both
    pose and twist and not have to wait for the filter state to catch up.
    """

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

        self._tf_buffer = tf2_ros.Buffer(rclpy.duration.Duration(seconds=30))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._latest_global_match_stamp: Optional[Time] = None

        # Create transformers
        self._transformer_to_wgs84 = Transformer.from_crs(
            f"EPSG:{self.dem_vertical_datum}",
            f"EPSG:{self._EPSG_WGS84}",
            always_xy=True,
        )
        self._transformer_to_msl = Transformer.from_crs(
            f"EPSG:{self.dem_vertical_datum}", f"EPSG:{self._EPSG_MSL}", always_xy=True
        )

        self._received_odometry_counter: int = 0

        # Subscribe
        self.odometry

    @property
    @ROS.parameter(ROS_D_DEM_VERTICAL_DATUM, descriptor=_ROS_PARAM_DESCRIPTOR_READ_ONLY)
    def dem_vertical_datum(self) -> Optional[int]:
        """DEM vertical datum

        > [!IMPORTANT]
        > Must match DEM that is published in :attr:`.GISNode.orthoimage`
        """

    def _odometry_cb(self, msg: Odometry) -> None:
        """Callback for :attr:`.odometry`"""
        if msg.header.frame_id == "gisnav_odom":
            if (
                self._received_odometry_counter
                >= self._REQUIRED_ODOMETRY_MESSAGES_BEFORE_PUBLISH
            ):
                # Only publish mock GPS messages from VO odometry
                # Using odometry derived from global EKF would greatly overestimate
                # velocity because the map to odom transform jumps around - vehicle is
                # not actually doing that.
                self._publish(msg)
            else:
                remaining = (
                    self._REQUIRED_ODOMETRY_MESSAGES_BEFORE_PUBLISH
                    - self._received_odometry_counter
                )
                self.get_logger().info(
                    f"Waiting for filter state to catch up - still need "
                    f"{remaining} more messages"
                )
                self._received_odometry_counter += 1
        else:
            assert msg.header.frame_id == "gisnav_map"
            self._latest_global_match_stamp = msg.header.stamp

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
            mock_gps_dict = tf_.odom_to_typed_dict(self, odometry)

            if mock_gps_dict is not None:
                self.sensor_gps(**mock_gps_dict)
            else:
                self.get_logger().warning("Skipping publishing SensorGps")

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

    def _transform_twist_with_covariance(
        self, twist_with_cov, stamp, from_frame, to_frame
    ):
        # Transform the linear component
        ts = rclpy.time.Time(seconds=stamp.sec, nanoseconds=stamp.nanosec)
        point = PointStamped()
        point.header.frame_id = from_frame
        point.header.stamp = ts.to_msg()  # stamp
        point.point.x = twist_with_cov.twist.linear.x
        point.point.y = twist_with_cov.twist.linear.y
        point.point.z = twist_with_cov.twist.linear.z

        try:
            # Get the transformation matrix
            transform = tf_.lookup_transform(
                self._tf_buffer,
                to_frame,
                from_frame,
                time_duration=(stamp, rclpy.duration.Duration(seconds=0.2)),
                logger=self.get_logger(),
            )
            if transform is None:
                return None
            # Set transform linear component to zero, only use orientation since
            # we are applying this to a velocity
            transform.transform.translation = Vector3()
            transformed_point = tf2_geometry_msgs.do_transform_point(point, transform)

            quat = [
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ]
            # Get the rotation matrix from the quaternion
            rot_matrix = tf_transformations.quaternion_matrix(quat)[
                :3, :3
            ]  # We only need the 3x3 rotation part

            # The Jacobian for linear velocity is just the rotation matrix
            J = rot_matrix

            # Extract the linear velocity covariance (3x3)
            linear_cov = np.array(twist_with_cov.covariance).reshape(6, 6)[:3, :3]

            # Transform the covariance
            transformed_linear_cov = J @ linear_cov @ J.T

            # Create a new TwistWithCovariance
            transformed_twist_with_cov = TwistWithCovariance()
            transformed_twist_with_cov.twist.linear = Vector3(
                x=transformed_point.point.x,
                y=transformed_point.point.y,
                z=transformed_point.point.z,
            )
            # Keep the original angular component
            transformed_twist_with_cov.twist.angular = twist_with_cov.twist.angular

            # Update the covariance
            transformed_cov = np.zeros((6, 6))
            transformed_cov[:3, :3] = transformed_linear_cov
            transformed_cov[3:, 3:] = np.array(twist_with_cov.covariance).reshape(6, 6)[
                3:, 3:
            ]  # Keep original angular covariance
            transformed_twist_with_cov.covariance = transformed_cov.flatten().tolist()

            return transformed_twist_with_cov

        except tf2_ros.TransformException as ex:
            self.get_logger().error(f"Could not transform twist with covariance: {ex}")
            return None
