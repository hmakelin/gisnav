"""Helper functions for ROS messaging"""
import time
import numpy as np

from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion

from ..assertions import assert_type, assert_shape


def create_header(frame_id: str = '') -> Header:
    """Creates a class:`std_msgs.msg.Header` for an outgoing ROS message

    :param frame_id: Header frame_id value
    :return: ROS message header
    """
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
    """Converts (x, y, z, w) format numpy array quaternion to ROS :class:`geometric_msg.msg.Quaternion`

    .. seealso:
        :meth:`.ros_to_np_quaternion`

    :param q: NumPy array quaternion of shape in (x, y, z, w) format
    :return: ROS quaternion message
    """
    assert_type(q, np.ndarray)
    q = q.squeeze()
    assert_shape(q, (4,))
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


def as_np_quaternion(q: Quaternion) -> np.ndarray:
    """Converts (x, y, z, w) format numpy array quaternion to ROS :class:`geometric_msg.msg.Quaternion`

    .. seealso:
        :meth:`.np_to_ros_quaternion`

    :param q: ROS quaternion message
    :return: NumPy array quaternion in (x, y, z, w) format
    """
    return np.array([q.x, q.y, q.z, q.w])
