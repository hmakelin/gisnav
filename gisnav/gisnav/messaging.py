"""Helper functions for ROS messaging"""
import time
from typing import Final, Literal, Optional
import cv2

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
    "reference",
    "base_link",
    "camera",
    "gimbal",
    "map",
    "world",
]
"""Allowed ROS header frame_ids (used by tf2)
    
The 'camera' and 'world' frames are coordinate systems in the cv2 pinhole 
camera model. The 'camera' frame follows the convention where the x axis
points to the right from the body of the camera and z-axis forward along
the optical axis, it is not the 'camera_optical' frame where the x axis 
points in the direction of the optical axis.

The 'base_link' frame is defined as the vehicle body :term:`FRD` frame.

The 'reference' frame is the :term:'reference' arrays coordinate frame
where the origin is the bottom left (ROS convention, not numpy/cv2 top left
convention). x axis is the width axis, y axis is height.
"""


def create_transform_msg(
    stamp,
    parent_frame: FrameID,
    child_frame: FrameID,
    q: tuple,
    translation_vector: np.ndarray,
) -> Optional[TransformStamped]:
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

def visualize_transform(transform, image, height, title):
    """Shows transform translation x and y position on image"""
    t = np.array((transform.transform.translation.x, transform.transform.translation.y,
                  transform.transform.translation.z))
    # current image timestamp does not yet have the transform but this should get the previous one
    x, y = int(t[0]), int(height - t[1])  # move height origin from bottom to top left for cv2
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
