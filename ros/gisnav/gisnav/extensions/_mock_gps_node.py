"""Abstract base class for :class:`.UORBNode` and :class:`.UBXNode`
"""
from abc import ABC, abstractmethod
from typing import Final, Optional, Tuple, TypedDict

import numpy as np
import rclpy
import tf2_geometry_msgs
import tf2_ros
import tf_transformations
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PointStamped, PoseStamped, TwistWithCovariance, Vector3
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


class MockGPSNode(Node, ABC):
    """A node that publishes mock GPS emssages to FCU"""

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

    class MockGPSDict(TypedDict):
        lat: int
        lon: int
        altitude_ellipsoid: float
        altitude_amsl: float
        yaw_degrees: int
        h_variance_rad: float
        vel_n_m_s: float
        vel_e_m_s: float
        vel_d_m_s: float
        cog: float
        cog_variance_rad: float
        s_variance_m_s: float
        timestamp: int
        eph: float
        epv: float
        satellites_visible: int

    def odom_to_typed_dict(
        self: rclpy.node.Node, odometry: Odometry
    ) -> Optional[MockGPSDict]:
        """Parses odometry message into a structure used as input for mock GPS
        messages
        """
        # TODO use inverse publish rate for duration
        assert odometry.header.frame_id == "gisnav_odom"

        # Do not use timestamp from VO, use latest global match timestamp
        # instead. This is to avoid interpolating in the gisnav_map frame which
        # leads to bad results because the observations there are very sparse if
        # e.g. running on CPU only
        if self._latest_global_match_stamp is None:
            self.get_logger().warning(
                "No global match timestamps yet, skipping publishing mock GPS msg"
            )
            return None
        odom_to_earth = tf_.lookup_transform(
            self._tf_buffer,
            "earth",
            self._odometry.header.frame_id,
            (
                self._latest_global_match_stamp,
                rclpy.duration.Duration(seconds=0.2),
            ),  # (odometry.header.stamp, rclpy.duration.Duration(seconds=0.2)),
            logger=self.get_logger(),
        )
        if odom_to_earth is None:
            self.get_logger().warning(
                f"Could not determine transform from {odometry.header.frame_id} "
                f"to earth"
            )
            return None

        odom_time = rclpy.time.Time(
            seconds=odometry.header.stamp.sec,
            nanoseconds=odometry.header.stamp.nanosec,
        )
        # transform time is same as self._latest_global_match_stamp?
        transform_time = rclpy.time.Time(
            seconds=odom_to_earth.header.stamp.sec,
            nanoseconds=odom_to_earth.header.stamp.nanosec,
        )
        try:
            transform_bridge = self._tf_buffer.lookup_transform_full(
                "gisnav_base_link",
                transform_time,
                "gisnav_base_link",
                odom_time,
                "gisnav_odom",
                rclpy.duration.Duration(seconds=0.2),
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            self.get_logger().warning(f"Could not get transform to old odom frame: {e}")
            return None

        pose = PoseStamped(header=odometry.header, pose=odometry.pose.pose)

        pose = tf_.add_transform_stamped(pose, transform_bridge)
        pose = tf_.transform_to_pose(pose)

        pose = tf2_geometry_msgs.do_transform_pose(pose.pose, odom_to_earth)
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
        twist_with_covariance = odometry.twist
        twist_with_covariance = self._transform_twist_with_covariance(
            twist_with_covariance,
            odometry.header.stamp,
            "gisnav_base_link",
            "gisnav_map_ned",
        )
        if twist_with_covariance is None:
            self.get_logger().warning("Could not determine twist covariances")
            return None

        vel_n_m_s = twist_with_covariance.twist.linear.x
        vel_e_m_s = twist_with_covariance.twist.linear.y
        vel_d_m_s = twist_with_covariance.twist.linear.z

        # Heading
        transform_earth_to_map = tf_.lookup_transform(
            self._tf_buffer,
            "gisnav_map",
            "earth",
            time_duration=(
                odometry.header.stamp,
                rclpy.duration.Duration(seconds=0.2),
            ),
            logger=self.get_logger(),
        )
        pose_map = tf2_geometry_msgs.do_transform_pose(pose, transform_earth_to_map)
        euler = tf_transformations.euler_from_quaternion(
            tf_.as_np_quaternion(pose_map.orientation).tolist()
        )
        yaw_rad = euler[2]  # ENU frame
        yaw_rad = -yaw_rad  # NED frame ("heading")

        if yaw_rad < 0:
            yaw_rad = 2 * np.pi + yaw_rad

        # re-center yaw to [0, 2*pi), it should be at [-pi, pi) before re-centering
        yaw_rad += np.pi / 2
        vehicle_yaw_degrees = np.degrees(yaw_rad)
        vehicle_yaw_degrees = int(vehicle_yaw_degrees % 360)
        # MAVLink yaw definition 0 := not available
        vehicle_yaw_degrees = 360 if vehicle_yaw_degrees == 0 else vehicle_yaw_degrees

        # Speed variance, assume no covariances
        twist_cov = twist_with_covariance.covariance.reshape((6, 6))
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
            Calculates course over ground from east and north velocities.

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
            elif east_velocity >= 0 > north_velocity:
                # bottom-right quadrant
                course_over_ground_radians = 0.5 * np.pi + np.arcsin(
                    -north_velocity / magnitude
                )
            elif east_velocity < 0 and north_velocity < 0:
                # bottom-left quadrant
                course_over_ground_radians = np.pi + np.arcsin(
                    -east_velocity / magnitude
                )
            elif east_velocity < 0 <= north_velocity:
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

        return {
            "lat": int(lat * 1e7),
            "lon": int(lon * 1e7),
            "altitude_ellipsoid": alt_ellipsoid,
            "altitude_amsl": alt_amsl,
            "yaw_degrees": vehicle_yaw_degrees,
            "h_variance_rad": h_variance_rad,
            "vel_n_m_s": vel_n_m_s,
            "vel_e_m_s": vel_e_m_s,
            "vel_d_m_s": vel_d_m_s,
            "cog": cog,
            "cog_variance_rad": cog_variance_rad,
            "s_variance_m_s": s_variance_m_s,
            "timestamp": timestamp,
            "eph": eph,
            "epv": epv,
            "satellites_visible": satellites_visible,
        }

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
                mock_gps_dict = self.odom_to_typed_dict(msg)
                if mock_gps_dict is not None:
                    self._publish(mock_gps_dict)
                else:
                    self.get_logger().info(
                        "Mock GPS dictionary was None - skipping publishing."
                    )
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

    @abstractmethod
    @narrow_types
    def _publish(self, mock_gps_dict: MockGPSDict) -> None:
        raise NotImplementedError

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
