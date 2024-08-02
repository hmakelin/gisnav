"""Helper functions for ROS messaging"""
from collections import namedtuple
from typing import Optional, Tuple, TypedDict, Union, cast

import numpy as np
import rclpy.time
import tf2_geometry_msgs
import tf2_ros
import tf_transformations
from builtin_interfaces.msg import Time
from geographic_msgs.msg import BoundingBox
from geometry_msgs.msg import (
    PoseStamped,
    PoseWithCovarianceStamped,
    Quaternion,
    TransformStamped,
    TwistWithCovarianceStamped,
)
from nav_msgs.msg import Odometry
from pyproj import Proj, transform
from rclpy.node import Node
from std_msgs.msg import Header

from .constants import FrameID

BBox = namedtuple("BBox", "left bottom right top")


def usec_from_header(header: Header) -> int:
    """Returns timestamp in microseconds from :class:`.std_msgs.msg.Header`
    stamp
    """
    return int(1e6 * header.stamp.sec + 1e-3 * header.stamp.nanosec)


def as_ros_quaternion(q: np.ndarray) -> Quaternion:
    """Converts (x, y, z, w) format numpy array quaternion to ROS
    :class:`geometric_msg.msg.Quaternion`

    .. seealso:
        :meth:`.ros_to_np_quaternion`

    :param q: NumPy array quaternion of shape in (x, y, z, w) format
    :return: ROS quaternion message
    """
    q = q.squeeze()
    assert q.shape == (4,)
    return Quaternion(x=q[0].item(), y=q[1].item(), z=q[2].item(), w=q[3].item())


def as_np_quaternion(q: Quaternion) -> np.ndarray:
    """Converts ROS :class:`geometric_msg.msg.Quaternion` to (x, y, z, w)
    format numpy array quaternion

    .. seealso:
        :meth:`.np_to_ros_quaternion`

    :param q: ROS quaternion message
    :return: NumPy array quaternion in (x, y, z, w) format
    """
    return np.array([q.x, q.y, q.z, q.w])


def bounding_box_to_bbox(msg: BoundingBox) -> BBox:
    """Converts :class:`geographic_msgs.msg.BoundingBox` to :class:`.BBox`"""
    return BBox(
        msg.min_pt.longitude,
        msg.min_pt.latitude,
        msg.max_pt.longitude,
        msg.max_pt.latitude,
    )


def create_transform_msg(
    stamp,
    parent_frame: FrameID,  # TODO: remove this arg - it should be in the stamp
    child_frame: FrameID,
    q: tuple,
    translation_vector: np.ndarray,
) -> TransformStamped:
    transform = TransformStamped()

    # transform.header.stamp = self.get_clock().now().to_msg()
    transform.header.stamp = stamp
    transform.header.frame_id = parent_frame
    transform.child_frame_id = child_frame

    transform.transform.rotation.x = q[0]
    transform.transform.rotation.y = q[1]
    transform.transform.rotation.z = q[2]
    transform.transform.rotation.w = q[3]

    transform.transform.translation.x = translation_vector[0]
    transform.transform.translation.y = translation_vector[1]
    transform.transform.translation.z = translation_vector[2]

    return transform


def create_pose_msg(
    stamp: Time,
    frame_id: FrameID,
    r: np.ndarray,
    t: np.ndarray,
) -> Optional[PoseStamped]:
    try:
        H = matrices_to_homogenous(r, t)
        q = tf_transformations.quaternion_from_matrix(H)
    except np.linalg.LinAlgError:
        return None

    pose = PoseStamped()

    pose.header.stamp = stamp
    pose.header.frame_id = frame_id

    pose.pose.orientation.x = q[0]
    pose.pose.orientation.y = q[1]
    pose.pose.orientation.z = q[2]
    pose.pose.orientation.w = q[3]

    pose.pose.position.x = float(t[0])
    pose.pose.position.y = float(t[1])
    pose.pose.position.z = float(t[2])

    return pose


def pose_to_transform(
    pose_stamped_msg: Union[PoseStamped, PoseWithCovarianceStamped],
    child_frame_id: FrameID,
) -> TransformStamped:
    # Create a new TransformStamped message
    transform_stamped = TransformStamped()

    # Copy the header
    transform_stamped.header = pose_stamped_msg.header

    # Copy the pose information to the transform
    pose = (
        pose_stamped_msg.pose
        if isinstance(pose_stamped_msg, PoseStamped)
        else pose_stamped_msg.pose.pose
    )
    transform_stamped.transform.translation.x = pose.position.x
    transform_stamped.transform.translation.y = pose.position.y
    transform_stamped.transform.translation.z = pose.position.z
    transform_stamped.transform.rotation = pose.orientation

    # Set the child and parent frame IDs
    transform_stamped.child_frame_id = child_frame_id

    return transform_stamped


def transform_to_pose(
    transform_stamped_msg: TransformStamped,
) -> PoseStamped:
    pose_stamped = PoseStamped()
    pose_stamped.header = transform_stamped_msg.header
    pose_stamped.pose.position.x = transform_stamped_msg.transform.translation.x
    pose_stamped.pose.position.y = transform_stamped_msg.transform.translation.y
    pose_stamped.pose.position.z = transform_stamped_msg.transform.translation.z
    pose_stamped.pose.orientation = transform_stamped_msg.transform.rotation
    return pose_stamped


def get_transform(
    node: Node, target_frame: FrameID, source_frame: FrameID, stamp: Time
) -> TransformStamped:
    try:
        trans = node._tf_buffer.lookup_transform(target_frame, source_frame, stamp)
        return trans
    except (
        tf2_ros.LookupException,
        tf2_ros.ConnectivityException,
        tf2_ros.ExtrapolationException,
    ) as e:
        # Todo: implement more granular exception handling
        node.get_logger().warn(
            f"Could not retrieve transformation from {source_frame} to "
            f"{target_frame} {e}."
        )
        return None


def lookup_transform(
    buffer: tf2_ros.Buffer,
    target_frame: FrameID,
    source_frame: FrameID,
    time_duration: Optional[Tuple[Time, rclpy.duration.Duration]] = None,
    logger=None,
) -> TransformStamped:
    """Returns transform from source to target frame, or None if not available"""
    transform: Optional[TransformStamped] = None
    try:
        if time_duration is not None:
            # Get transform at desired timestamp
            transform = buffer.lookup_transform(
                target_frame, source_frame, time_duration[0], time_duration[1]
            )
        else:
            # Get latest transform instead
            transform = buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time()
            )
    except (
        # np.linalg.LinAlgError,
        tf2_ros.LookupException,
        tf2_ros.ConnectivityException,
        tf2_ros.ExtrapolationException,
    ) as e:
        if logger is not None:
            if time_duration is not None:
                logger.debug(
                    f"Cannot transform {source_frame} to {target_frame} frame "
                    f"at desired timestamp, using latest frame instead: {e}"
                )
                return lookup_transform(
                    buffer, target_frame, source_frame, logger=logger
                )
            else:
                logger.warning(
                    f"Cannot transform {source_frame} to {target_frame} frame: {e}"
                )

    return transform


# TODO: refactor out this function
def extract_yaw(q: Quaternion) -> float:
    """Calculate the yaw angle from a quaternion in the ENU frame.

    Returns yaw with origin centered at North (i.e. applies a 90 degree adjustment).

    :param q: A list containing the quaternion [qx, qy, qz, qw].
    :return: The yaw angle in degrees.
    """
    enu_yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
    enu_yaw_deg = np.degrees(enu_yaw)

    # Convert ENU yaw to heading with North as origin
    heading = 90.0 - enu_yaw_deg

    # Normalize to [0, 360) range
    heading = (heading + 360) % 360

    return heading


# TODO: refactor out this function
def extract_roll(q: Quaternion) -> float:
    """Calculate the roll angle from a quaternion

    :param q: A quaternion represented as a Quaternion object in (x, y, z, w)
        format
    :return: The roll angle in degrees
    """
    roll = np.arctan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x**2 + q.y**2))
    roll_deg = np.degrees(roll)

    # Normalize to [0, 360) range
    roll_deg = (roll_deg + 360) % 360

    return roll_deg


def matrices_to_homogenous(r, t) -> np.ndarray:
    """3D pose in homogenous form for convenience"""
    H = np.eye(4)
    H[:3, :3] = r
    H[:3, 3] = t.squeeze()
    return H


def affine_to_proj(M: np.ndarray) -> FrameID:
    """Returns a PROJ string describing a CRS that is defined by converting the image
    pixel coordinates (x, y) to WGS 84 coordinates (lon, lat) using the provided
    affine transformation.

    :returns: PROJ string representing the affine transformation
    """
    assert M.shape == (3, 4) or M.shape == (4, 4)

    # Build the PROJ string
    # Assume only translation and rotation in x and y plane, scaling in any plane -
    # this makes the proj string shorter
    proj_str: FrameID = cast(
        FrameID,
        (
            f"+proj=affine "
            f"+xoff={M[0, 3]} +yoff={M[1, 3]} +zoff={M[2, 3]} "
            f"+s11={M[0, 0]} +s12={M[0, 1]} +s13={M[0, 2]} "
            f"+s21={M[1, 0]} +s22={M[1, 1]} +s23={M[1, 2]} "
            f"+s31={M[2, 0]} +s32={M[2, 1]} +s33={M[2, 2]} "
            f"+no_defs +type=crs +datum=WGS84"
        ),
    )

    return proj_str


def proj_to_affine(proj_str: str) -> np.ndarray:
    """Returns the affine transformation matrix M that corresponds to the provided
    PROJ string. The PROJ string should be in the format used by the `affine_to_proj`
    function.

    :param proj_str: PROJ string representing an affine transformation
    :returns: 3x3 affine transformation matrix M
    """
    # Extract the coefficients from the PROJ string
    tokens = proj_str.replace("=", " ").split()
    xoff = float(tokens[tokens.index("+xoff") + 1])
    yoff = float(tokens[tokens.index("+yoff") + 1])
    zoff = float(tokens[tokens.index("+zoff") + 1])
    s11 = float(tokens[tokens.index("+s11") + 1])
    s12 = float(tokens[tokens.index("+s12") + 1])
    s13 = float(tokens[tokens.index("+s13") + 1])
    s21 = float(tokens[tokens.index("+s21") + 1])
    s22 = float(tokens[tokens.index("+s22") + 1])
    s23 = float(tokens[tokens.index("+s23") + 1])
    s31 = float(tokens[tokens.index("+s31") + 1])
    s32 = float(tokens[tokens.index("+s32") + 1])
    s33 = float(tokens[tokens.index("+s33") + 1])

    # Build the affine transformation matrix M
    M = np.array([[s11, s12, s13, xoff], [s21, s22, s23, yoff], [s31, s32, s33, zoff]])

    return M


def wgs84_to_ecef(lon: float, lat: float, alt: float) -> Tuple[float, float, float]:
    """Convert :term:`WGS 84` geodetic coordinates to ECEF (Earth-Centered, Earth-Fixed)
    coordinates.

    :param lon: Longitude in decimal degrees.
    :param lat: Latitude in decimal degrees.
    :param alt: Altitude above the WGS84 ellipsoid in meters.
    :return: A tuple (x, y, z) representing ECEF coordinates in meters.

    Uses the Proj4 library to transform from geographic (latitude, longitude, altitude)
    coordinates to Cartesian coordinates (x, y, z) in the ECEF system (``earth`` frame
    in :term:`REP 105`).
    """
    proj_wgs84 = Proj(proj="latlong", datum="WGS84")
    proj_ecef = Proj(proj="geocent", datum="WGS84")
    x, y, z = transform(proj_wgs84, proj_ecef, lon, lat, alt, radians=False)
    return x, y, z


def ecef_to_wgs84(x: float, y: float, z: float) -> Tuple[float, float, float]:
    """Convert ECEF (Earth-Centered, Earth-Fixed) coordinates to :term:`WGS 84`
    geodetic coordinates.

    :param x: ECEF x-coordinate in meters.
    :param y: ECEF y-coordinate in meters.
    :param z: ECEF z-coordinate in meters.
    :return: A tuple (lon, lat, alt) representing longitude, latitude, and altitude
        in the WGS84 coordinate system.

    Uses the Proj4 library to transform from Cartesian coordinates (x, y, z) in the
    ECEF system (``earth`` frame in :term:`REP 105`) to geographic (latitude,
    longitude, altitude) coordinates.
    """
    proj_wgs84 = Proj(proj="latlong", datum="WGS84")
    proj_ecef = Proj(proj="geocent", datum="WGS84")
    lon, lat, alt = transform(proj_ecef, proj_wgs84, x, y, z, radians=False)
    return lon, lat, alt


def enu_to_ecef_matrix(lon: float, lat: float) -> np.ndarray:
    """Generate the rotation matrix for converting ENU coordinates at a given
    longitude (lon) and latitude (lat) to ECEF coordinates.

    :param lon: Longitude in decimal degrees.
    :param lat: Latitude in decimal degrees.
    :return: Rotation matrix (3x3) for the transformation.
    """
    lon, lat = np.radians(lon), np.radians(lat)

    slat, clat = np.sin(lat), np.cos(lat)
    slon, clon = np.sin(lon), np.cos(lon)

    R = np.array(
        [
            [
                -slon,
                -slat * clon,
                clat * clon,
            ],  # ECEF x-axis expressed in ENU coordinates
            [clon, -slat * slon, clat * slon],  # ECEF y-axis
            [0, clat, slat],  # ECEF z-axis
        ]
    )
    return R


def poses_to_twist(
    pose2: PoseWithCovarianceStamped, pose1: PoseStamped
) -> TwistWithCovarianceStamped:
    """pose2 is newer pose with covariances, pose1 is older pose (identity pose in
    VO)
    """
    # Time step between poses
    t2, t1 = usec_from_header(pose2.header), usec_from_header(pose1.header)

    time_step = (t2 - t1) / 1e6  # seconds

    # Calculate linear velocities
    dx = pose2.pose.pose.position.x - pose1.pose.position.x
    dy = pose2.pose.pose.position.y - pose1.pose.position.y
    dz = pose2.pose.pose.position.z - pose1.pose.position.z

    # Calculate angular velocities using quaternion differences
    q1 = [
        pose1.pose.orientation.x,
        pose1.pose.orientation.y,
        pose1.pose.orientation.z,
        pose1.pose.orientation.w,
    ]
    q2 = [
        pose2.pose.pose.orientation.x,
        pose2.pose.pose.orientation.y,
        pose2.pose.pose.orientation.z,
        pose2.pose.pose.orientation.w,
    ]

    q_diff = tf_transformations.quaternion_multiply(
        q2, tf_transformations.quaternion_inverse(q1)
    )

    # Converting quaternion to rotation vector (axis-angle)
    angle = 2 * np.arccos(q_diff[3])  # Compute the rotation angle
    axis = q_diff[:3] / np.sin(angle / 2)  # Normalize the axis
    rotation_vector = angle * axis  # Multiply angle by the normalized axis

    ang_vel = rotation_vector / time_step

    # Create TwistStamped message
    twist = TwistWithCovarianceStamped()
    twist.header.stamp = pose2.header.stamp
    twist.header.frame_id = pose2.header.frame_id
    twist.twist.twist.linear.x = dx / time_step
    twist.twist.twist.linear.y = dy / time_step
    twist.twist.twist.linear.z = dz / time_step
    twist.twist.twist.angular.x = ang_vel[0]
    twist.twist.twist.angular.y = ang_vel[1]
    twist.twist.twist.angular.z = ang_vel[2]

    twist.twist.covariance = pose2.pose.covariance  # todo: could make these smaller?

    return twist


def create_identity_pose_stamped(x, y, z):
    pose_stamped = PoseStamped()

    pose_stamped.header.stamp = rclpy.time.Time().to_msg()
    pose_stamped.header.frame_id = "camera_optical"

    # Set the position coordinates
    pose_stamped.pose.position.x = x
    pose_stamped.pose.position.y = y
    pose_stamped.pose.position.z = z

    # Set the orientation to identity (no rotation)
    pose_stamped.pose.orientation.x = 0.0
    pose_stamped.pose.orientation.y = 0.0
    pose_stamped.pose.orientation.z = 0.0
    pose_stamped.pose.orientation.w = 1.0

    return pose_stamped


def angle_off_nadir(quaternion):
    """Angle off nadir in radians"""
    # Convert quaternion to a rotation matrix
    rotation_matrix = tf_transformations.quaternion_matrix(quaternion)[:3, :3]

    # Camera's forward direction in the camera frame (assuming camera facing
    # positive x-axis)
    camera_forward = np.array([1, 0, 0])

    # Transform the camera's forward direction to the base frame
    camera_forward_base = np.dot(rotation_matrix, camera_forward)

    # Nadir direction in the base frame (assuming nadir is negative z-axis
    # in base frame)
    nadir_direction = np.array([0, 0, -1])

    # Calculate the angle between the forward vector and nadir direction
    cos_theta = np.dot(camera_forward_base, nadir_direction) / (
        np.linalg.norm(camera_forward_base) * np.linalg.norm(nadir_direction)
    )
    angle_off_nadir = np.arccos(
        np.clip(cos_theta, -1.0, 1.0)
    )  # Clip to handle numerical inaccuracies

    return angle_off_nadir


def add_transform_stamped(
    transform1: Union[TransformStamped, PoseStamped], transform2: TransformStamped
) -> TransformStamped:
    # Convert the TransformStamped objects to transformation matrices
    translation1 = (
        transform1.transform.translation
        if isinstance(transform1, TransformStamped)
        else transform1.pose.position
    )
    rotation1 = (
        transform1.transform.rotation
        if isinstance(transform1, TransformStamped)
        else transform1.pose.orientation
    )

    transform1_matrix = tf_transformations.concatenate_matrices(
        tf_transformations.translation_matrix(
            (
                translation1.x,
                translation1.y,
                translation1.z,
            )
        ),
        tf_transformations.quaternion_matrix(
            (
                rotation1.x,
                rotation1.y,
                rotation1.z,
                rotation1.w,
            )
        ),
    )

    transform2_matrix = tf_transformations.concatenate_matrices(
        tf_transformations.translation_matrix(
            (
                transform2.transform.translation.x,
                transform2.transform.translation.y,
                transform2.transform.translation.z,
            )
        ),
        tf_transformations.quaternion_matrix(
            (
                transform2.transform.rotation.x,
                transform2.transform.rotation.y,
                transform2.transform.rotation.z,
                transform2.transform.rotation.w,
            )
        ),
    )

    # Multiply the transformation matrices to get the composed transformation
    combined_matrix = tf_transformations.concatenate_matrices(
        transform1_matrix, transform2_matrix
    )

    # Extract the translation and rotation from the combined matrix
    translation = tf_transformations.translation_from_matrix(combined_matrix)
    rotation = tf_transformations.quaternion_from_matrix(combined_matrix)

    # Create a new TransformStamped object for the result
    combined_transform = TransformStamped()
    # combined_transform.header.stamp = rclpy.clock.Clock().now().to_msg()
    # combined_transform.header.stamp = transform1.header.stamp
    # combined_transform.header.frame_id = transform1.header.frame_id
    combined_transform.header = transform1.header
    combined_transform.child_frame_id = transform2.child_frame_id

    combined_transform.transform.translation.x = translation[0]
    combined_transform.transform.translation.y = translation[1]
    combined_transform.transform.translation.z = translation[2]
    combined_transform.transform.rotation.x = rotation[0]
    combined_transform.transform.rotation.y = rotation[1]
    combined_transform.transform.rotation.z = rotation[2]
    combined_transform.transform.rotation.w = rotation[3]

    return combined_transform


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


# TODO: implement this better - very brittle as we use some custome node methods here
#  like self._convert_to_wgs84. Make an abstract base class (_MockGPSNode) for
#  UBXNode and UORBNode instead?
def odom_to_typed_dict(
    self: rclpy.node.Node, odometry: Odometry
) -> Optional[MockGPSDict]:
    """Parses odometry message into a structure used as input for mock GPS messages"""
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
    odom_to_earth = lookup_transform(
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

    pose = add_transform_stamped(pose, transform_bridge)
    pose = transform_to_pose(pose)

    pose = tf2_geometry_msgs.do_transform_pose(pose.pose, odom_to_earth)
    # WGS 84 longitude and latitude, and AGL altitude in meters
    # TODO: this is alt above ellipsoid, not agl

    lon, lat, alt_agl = ecef_to_wgs84(pose.position.x, pose.position.y, pose.position.z)

    timestamp = usec_from_header(odometry.header)

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
    transform_earth_to_map = lookup_transform(
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
        as_np_quaternion(pose_map.orientation).tolist()
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
        numerator = (vel_e_m_s_var * vel_n_m_s**2) + (vel_n_m_s_var * vel_e_m_s**2)
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
            course_over_ground_radians = np.pi + np.arcsin(-east_velocity / magnitude)
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
