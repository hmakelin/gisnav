"""Helper functions for ROS messaging"""
from collections import namedtuple
from typing import Optional

import cv2
import numpy as np
import rclpy.time
import tf2_ros
from geographic_msgs.msg import BoundingBox
from geometry_msgs.msg import Quaternion, TransformStamped
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


def get_transform(
    node: Node, target_frame: FrameID, source_frame: FrameID, stamp
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


def visualize_transform(transform, image, height, title):
    """Shows transform translation x and y position on image"""
    t = np.array(
        (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        )
    )
    # current image timestamp does not yet have the transform but this should
    # get the previous one
    x, y = int(t[0]), int(
        height - t[1]
    )  # move height origin from bottom to top left for cv2
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
