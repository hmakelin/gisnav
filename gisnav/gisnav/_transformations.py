"""Helper functions for ROS messaging"""
from collections import namedtuple
from typing import Optional, Tuple, Union, cast

import cv2
import numpy as np
import rclpy.time
import tf2_ros
import tf_transformations
from builtin_interfaces.msg import Time
from geographic_msgs.msg import BoundingBox
from geometry_msgs.msg import (
    Pose,
    PoseStamped,
    PoseWithCovarianceStamped,
    Quaternion,
    TransformStamped,
    Twist,
)
from nav_msgs.msg import Odometry
from pyproj import Transformer
from rclpy.node import Node
from sensor_msgs.msg import TimeReference
from std_msgs.msg import Header

from .constants import FrameID

BBox = namedtuple("BBox", "left bottom right top")


def create_header(
    node: Node, frame_id: str = "", time_reference: Optional[TimeReference] = None
) -> Header:
    """Creates a class:`std_msgs.msg.Header` for an outgoing ROS message

    If `time_reference` is provided, time_reference.header.stamp -
    time_reference.time_ref is subtracted from the message header stamp. This
    could e.g. be the difference between the local system time and the foreign
    :term:`FCU` time.

    :param frame_id: Header frame_id value
    :param time_reference: Optional time reference for synchronizing the
        timestamp
    :return: ROS message header
    """
    now = node.get_clock().now()
    header = Header()
    header.stamp.sec = now.seconds_nanoseconds()[0]
    header.stamp.nanosec = now.seconds_nanoseconds()[1]
    header.frame_id = frame_id

    if time_reference is not None:
        # sync time
        header.stamp = (
            rclpy.time.Time.from_msg(header.stamp)
            - (
                rclpy.time.Time.from_msg(time_reference.header.stamp)
                - rclpy.time.Time.from_msg(time_reference.time_ref)
            )
        ).to_msg()

    return header


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


def transform_to_pose(transform_stamped_msg):  # , frame_id: str):
    # Create a new PoseStamped message
    pose_stamped = PoseStamped()

    # Copy the header
    pose_stamped.header = transform_stamped_msg.header

    # Set the specified frame_id
    # pose_stamped.header.frame_id = frame_id

    # Copy the transform information to the pose
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


def visualize_camera_position(image, t, title):
    """Shows transform translation x and y position on image"""
    # current image timestamp does not yet have the transform but this should
    # get the previous one
    # x, y = int(t[0]), int(
    #    height - t[1]
    # )  # move height origin from bottom to top left for cv2
    x, y = t[0:2].squeeze().tolist()
    x, y = int(x), int(y)
    image = cv2.circle(np.array(image), (x, y), 5, (0, 255, 0), -1)
    cv2.imshow(title, image)
    cv2.waitKey(1)


def visualize_camera_corners(image, corners, title):
    """Shows transform translation x and y position on image"""
    # current image timestamp does not yet have the transform but this should
    # get the previous one
    # x, y = int(t[0]), int(
    #    height - t[1]
    # )  # move height origin from bottom to top left for cv2
    for i, corner in enumerate(corners):
        x, y = corner[0:2].squeeze().tolist()
        x, y = int(x), int(y)

        # image = cv2.circle(np.array(image), (x, y), 5, (0, 255, 0), -1)
        font = cv2.FONT_HERSHEY_SIMPLEX
        fontScale = 1
        color = (0, 255, 0)
        thickness = 2
        text = "TL" if i == 0 else "BR"
        image = cv2.putText(
            image, text, (x, y), font, fontScale, color, thickness, cv2.LINE_AA
        )
        image = cv2.circle(np.array(image), (x, y), 5, (0, 255, 0), -1)

    cv2.imshow(title, image)
    cv2.waitKey(1)


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


def pose_stamped_diff_to_odometry(
    pose1: PoseStamped, pose2: PoseStamped, child_frame_id: str = "camera"
) -> Odometry:
    """Returns and Odometry message from the difference of two PoseStamped messages

    :param pose1: Older PoseStamped message
    :param pose2: Newer PoseStamped message
    """
    assert pose1.header.frame_id == pose2.header.frame_id, (
        f"Frame IDs do not match: pose1 {pose1.header.frame_id} "
        f"and pose2: {pose2.header.frame_id}"
    )

    # Initialize an Odometry message
    odometry_msg = Odometry()

    # Set the frame_id and child_frame_id
    odometry_msg.header.frame_id = pose2.header.frame_id

    # TODO base_link to camera published elsewhere, assume this is camera
    odometry_msg.child_frame_id = child_frame_id

    # Set the position from the newer PoseStamped message
    odometry_msg.pose.pose = pose2.pose

    # Calculate position difference
    dx = pose2.pose.position.x - pose1.pose.position.x
    dy = pose2.pose.position.y - pose1.pose.position.y
    dz = pose2.pose.position.z - pose1.pose.position.z

    # Calculate orientation difference using quaternions
    q1 = [
        pose1.pose.orientation.x,
        pose1.pose.orientation.y,
        pose1.pose.orientation.z,
        pose1.pose.orientation.w,
    ]
    q2 = [
        pose2.pose.orientation.x,
        pose2.pose.orientation.y,
        pose2.pose.orientation.z,
        pose2.pose.orientation.w,
    ]
    q_diff = tf_transformations.quaternion_multiply(
        tf_transformations.quaternion_inverse(q1), q2
    )

    # Calculate angular velocity (this is a simplified approach; for more accuracy,
    # consider time differences)
    # Note: Assuming small time difference between messages for simplicity. For more
    # precise applications, use actual time difference.
    angular_velocity = tf_transformations.euler_from_quaternion(q_diff)

    # Set the linear and angular velocities in the Odometry message
    # Assuming the time delta between pose1 and pose2 is 1 second for simplicity.
    # Replace with actual time difference if available.
    time_delta = (pose2.header.stamp - pose1.header.stamp).to_sec()
    if time_delta > 0:
        odometry_msg.twist.twist.linear.x = dx / time_delta
        odometry_msg.twist.twist.linear.y = dy / time_delta
        odometry_msg.twist.twist.linear.z = dz / time_delta

        odometry_msg.twist.twist.angular.x = angular_velocity[0] / time_delta
        odometry_msg.twist.twist.angular.y = angular_velocity[1] / time_delta
        odometry_msg.twist.twist.angular.z = angular_velocity[2] / time_delta
    else:
        # Handle zero or negative time delta if necessary
        pass

    return odometry_msg


# TODO: use this in the odometry function?
def pose_stamped_diff(pose1: PoseStamped, pose2: PoseStamped) -> PoseStamped:
    """Returns a Transform that is the diff between the Pose messages

    :param pose1: Older PoseStamped message
    :param pose2: Newer PoseStamped message
    """
    assert pose1.header.frame_id == pose2.header.frame_id, (
        f"Frame IDs do not match: pose1 {pose1.header.frame_id} "
        f"and pose2: {pose2.header.frame_id}"
    )

    # Initialize an Odometry message
    pose_msg = pose1

    # Calculate position difference
    pose2.pose.position.x - pose1.pose.position.x
    pose2.pose.position.y - pose1.pose.position.y
    pose2.pose.position.z - pose1.pose.position.z
    pose_msg.pose.position.x = pose2.pose.position.x - pose1.pose.position.x
    pose_msg.pose.position.y = pose2.pose.position.y - pose1.pose.position.y
    pose_msg.pose.position.z = pose2.pose.position.z - pose1.pose.position.z

    return pose_msg


def pose_stamped_to_matrices(
    pose_stamped: Union[PoseStamped, PoseWithCovarianceStamped],
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    # Extract the orientation quaternion from the PoseStamped message
    # todo clean up implementation
    pose = (
        pose_stamped.pose
        if isinstance(pose_stamped, PoseStamped)
        else pose_stamped.pose.pose
    )
    quaternion = (
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w,
    )

    # Extract the position vector from the PoseStamped message
    position = (
        pose.position.x,
        pose.position.y,
        pose.position.z,
    )

    # Convert the quaternion to a rotation matrix
    rotation_matrix = tf_transformations.quaternion_matrix(quaternion)[:3, :3]

    # The translation vector is already in the correct format, but let's make it a
    # numpy array
    translation_vector = np.array(position)

    H = matrices_to_homogenous(rotation_matrix, translation_vector)

    return H, rotation_matrix, translation_vector


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


def proj_to_affine(proj_str: FrameID) -> np.ndarray:
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


def pose_to_twist(pose: Pose, time_diff_sec: int) -> Twist:
    # Initialize the Twist message
    twist = Twist()

    # Assuming pose.position and pose.orientation are populated
    # Calculate linear velocities
    twist.linear.x = pose.position.x / time_diff_sec
    twist.linear.y = pose.position.y / time_diff_sec
    twist.linear.z = pose.position.z / time_diff_sec

    # Convert quaternion to Euler angles for easier angular velocity calculation
    euler = tf_transformations.euler_from_quaternion(
        [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    )

    # Assuming the change in orientation is small enough to treat as linear
    twist.angular.x = euler[0] / time_diff_sec
    twist.angular.y = euler[1] / time_diff_sec
    twist.angular.z = euler[2] / time_diff_sec

    return twist


def lonlat_to_easting_northing(
    longitude: float, latitude: float
) -> Tuple[float, float]:
    """Converts WGS 84 longitude and latitude to meters from WGS 84 origin"""
    # Determine the UTM zone
    utm_zone = int((longitude + 180) / 6) + 1

    # Create a transformer to convert from latitude/longitude to UTM
    transformer = Transformer.from_crs(
        "epsg:4326", f"epsg:326{utm_zone:02d}", always_xy=True
    )

    # Convert the latitude and longitude to UTM
    easting, northing = transformer.transform(longitude, latitude)

    # TODO: consider passing proj_strings around
    # Adjust the easting to make it global (relative to the prime meridian) because
    # we do not want to be passing proj strings around: we want to have only one
    # "map_gisnav" map frame.

    # Calculate the central meridian of the UTM zone
    prime_meridian = (utm_zone - 1) * 6 - 180 + 3
    # Calculate the offset in meters from the prime meridian
    offset = (prime_meridian + 180) / 6 * 1000000 - 500000
    # Adjust the easting value
    global_easting = easting + offset

    return global_easting, northing


def easting_northing_to_lonlat(
    global_easting: float, northing: float
) -> Tuple[float, float]:
    """Converts easting and northing in meters from WGS 84 origin to longitude and
    latitude in degrees
    """
    # Estimate the approximate central meridian from the global easting
    approximate_central_meridian = ((global_easting + 500000) / 1000000) * 6 - 180

    # Estimate the UTM zone from the central meridian
    utm_zone = int((approximate_central_meridian + 180) / 6)

    # Calculate the offset in meters from the prime meridian
    prime_meridian = (utm_zone - 1) * 6 - 180 + 3
    offset = (prime_meridian + 180) / 6 * 1000000 - 500000

    # Reverse the adjustment to the easting value to get the original easting
    original_easting = global_easting - offset

    # Create a transformer to convert from UTM to latitude/longitude
    transformer = Transformer.from_crs(
        f"epsg:326{utm_zone:02d}", "epsg:4326", always_xy=True
    )

    # Convert the original easting and northing to latitude and longitude
    longitude, latitude = transformer.transform(original_easting, northing)

    return longitude, latitude
