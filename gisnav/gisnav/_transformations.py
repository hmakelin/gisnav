"""Helper functions for ROS messaging"""
from collections import namedtuple
from typing import Optional, Tuple

import cv2
import numpy as np
import rclpy.time
import tf2_ros
import tf_transformations
from builtin_interfaces.msg import Time
from geographic_msgs.msg import BoundingBox
from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
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
    parent_frame: FrameID,
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

    pose.pose.position.x = t[0]
    pose.pose.position.y = t[1]
    pose.pose.position.z = t[2]

    return pose


def pose_to_transform(
    pose_stamped_msg, parent_frame_id: FrameID, child_frame_id: FrameID
):
    # Create a new TransformStamped message
    transform_stamped = TransformStamped()

    # Copy the header
    transform_stamped.header = pose_stamped_msg.header

    # Copy the pose information to the transform
    transform_stamped.transform.translation.x = pose_stamped_msg.pose.position.x
    transform_stamped.transform.translation.y = pose_stamped_msg.pose.position.y
    transform_stamped.transform.translation.z = pose_stamped_msg.pose.position.z
    transform_stamped.transform.rotation = pose_stamped_msg.pose.orientation

    # Set the child and parent frame IDs
    transform_stamped.child_frame_id = child_frame_id
    transform_stamped.header.frame_id = parent_frame_id

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
        trans = node.tf_buffer.lookup_transform(target_frame, source_frame, stamp)
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
    image = cv2.circle(np.array(image), (x, y), 5, (0, 255, 0), -1)
    cv2.imshow(title, image)
    cv2.waitKey(1)


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
    pose_stamped: PoseStamped,
) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    # Extract the orientation quaternion from the PoseStamped message
    quaternion = (
        pose_stamped.pose.orientation.x,
        pose_stamped.pose.orientation.y,
        pose_stamped.pose.orientation.z,
        pose_stamped.pose.orientation.w,
    )

    # Extract the position vector from the PoseStamped message
    position = (
        pose_stamped.pose.position.x,
        pose_stamped.pose.position.y,
        pose_stamped.pose.position.z,
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
    H[:3, 3] = t
    return H
