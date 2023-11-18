"""Helper functions for ROS messaging"""
import time
from typing import Final, Literal

import numpy as np
import tf2_ros
import tf_transformations
from rclpy.duration import Duration
from geographic_msgs.msg import BoundingBox
from geometry_msgs.msg import Quaternion, TransformStamped
from rclpy.node import Node
from std_msgs.msg import Header

from ._data import BBox

# region ROS topic names
ROS_TOPIC_SENSOR_GPS: Final = "/fmu/in/sensor_gps"
"""Name of ROS topic for outgoing :class:`px4_msgs.msg.SensorGps` messages
over PX4 DDS bridge"""

ROS_TOPIC_CAMERA_INFO: Final = "/camera/camera_info"
"""Name of ROS topic for :class:`sensor_msgs.msg.CameraInfo` messages"""

ROS_TOPIC_IMAGE: Final = "/camera/image_raw"
"""Name of ROS topic for :class:`sensor_msgs.msg.Image` messages"""

# endregion ROS topic names
DELAY_DEFAULT_MS: Final = 2000
"""Max delay for things like global position"""


DELAY_FAST_MS: Final = 500
"""Max delay for messages with fast dynamics that go "stale" quickly, e.g.
local position and attitude. The delay can be a bit higher than is
intuitive because the vehicle EKF should be able to fuse things with
fast dynamics with higher lags as long as the timestamps are accurate.
"""


def create_header(frame_id: str = "") -> Header:
    """Creates a class:`std_msgs.msg.Header` for an outgoing ROS message

    :param frame_id: Header frame_id value
    :return: ROS message header
    """
    # TODO: use rclpy clock to create stamp
    time_ns = time.time_ns()
    sec = int(time_ns / 1e9)
    nanosec = int(time_ns - (1e9 * sec))
    header = Header()
    header.stamp.sec = sec
    header.stamp.nanosec = nanosec
    header.frame_id = frame_id
    return header


def usec_from_header(header: Header) -> int:
    """Returns timestamp in microseconds from :class:`.std_msgs.msg.Header` stamp"""
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


FrameID = Literal[
    "wgs_84",
    "query",
    "reference",
    "base_link",
    "camera",
    "gimbal",
    "map",
    "world",
]
"""Allowed ROS header frame_ids (used by tf2)

    The 'orthoimage', 'query_image', and 'reference_image' frames are all image
    planes in the pinhole camera model. The first and second (x and y) axes are
    parallel to the `camera` frame x and y axes.

    The 'base_link' frame is defined as the vehicle body :term:`FRD` frame.

    The 'camera' frame follows the pinhole camera model convention of axes where
    first axis points to right of camera aperture, second axis points down from
    aperture, and the third axis points into the viewing direction. This should
    not be described as a camera "FRD" frame as forward implies in the direction
    of the viewing axis, which is the third and not first axis in this convention.

    The 'camera_frd' frame is a more intuitive definition of the camera axes
    where the forward direction is in the direction of the optical viewing axis.
"""


def create_transform_msg(
    stamp,
    parent_frame: FrameID,
    child_frame: FrameID,
    rotation_matrix: np.ndarray,
    translation_vector: np.ndarray,
):
    transform = TransformStamped()

    # transform.header.stamp = self.get_clock().now().to_msg()
    transform.header.stamp = stamp
    transform.header.frame_id = parent_frame
    transform.child_frame_id = child_frame

    # Convert rotation matrix to quaternion
    #rotation = Rotation.from_matrix(rotation_matrix)
    #q = rotation.as_quat()
    rotation_4x4 = np.eye(4)
    rotation_4x4[:3, :3] = rotation_matrix
    q = tf_transformations.quaternion_from_matrix(rotation_4x4)

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
        # Look up the transformation
        #node.tf_buffer.can_transform(target_frame, source_frame, stamp, timeout=Duration(seconds=5.0))
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
