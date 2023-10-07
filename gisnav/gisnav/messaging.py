"""Helper functions for ROS messaging"""
import time

import numpy as np
import re
from geographic_msgs.msg import BoundingBox, GeoPoint
from geometry_msgs.msg import Quaternion
from scipy.spatial.transform import Rotation
from std_msgs.msg import Header

from ._assertions import assert_shape, assert_type
from ._data import BBox

# region ROS topic names
ROS_TOPIC_GPS_INPUT = "/mavros/gps_input/gps_input"
"""Name of ROS topic for outgoing :class:`mavros_msgs.msg.GPSINPUT` messages
over MAVROS"""

ROS_TOPIC_HIL_GPS = "/mavros/hil/gps"
"""Name of ROS topic for outgoing :class:`mavros_msgs.msg.HilGPS` messages
over MAVROS"""

ROS_TOPIC_SENSOR_GPS = "/fmu/in/sensor_gps"
"""Name of ROS topic for outgoing :class:`px4_msgs.msg.SensorGps` messages
over PX4 DDS bridge"""

ROS_TOPIC_CAMERA_INFO = "/camera/camera_info"
"""Name of ROS topic for :class:`sensor_msgs.msg.CameraInfo` messages"""

ROS_TOPIC_IMAGE = "/camera/image_raw"
"""Name of ROS topic for :class:`sensor_msgs.msg.Image` messages"""

ROS_TOPIC_HOME_POSITION = "/mavros/home_position/home"
"""Name of ROS topic for :class:`mavros_msgs.msg.HomePosition` messages"""

# endregion ROS topic names


DELAY_SLOW_MS = 10000
"""Max delay for messages where updates are not needed nor expected often,
e.g. home position
"""


DELAY_DEFAULT_MS = 2000
"""Max delay for things like global position"""


DELAY_FAST_MS = 500
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
    assert_type(q, np.ndarray)
    q = q.squeeze()
    assert_shape(q, (4,))
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


def wxyz_to_xyzw_q(q: np.ndarray) -> np.ndarray:
    """Converts quaternion from (w, x, y, z) to (x, y, z, w) format

    .. note::
        (w, x, y, z) is used by e.g. :class:`px4_msgs.msg.VehicleAttitude`
        while (x, y, z, w) is used by e.g. :class:`mavros_msgs.msg.Quaternion`
        and also SciPy.

    :param q: Quaternion in (w, x, y, z) format
    :return: Quaternion in (x, y, z, w) format
    """
    q_out = q.squeeze()
    assert_shape(q_out, (4,))
    q_out = np.append(q_out[1:], q_out[0])
    q_out = q_out.reshape(q.shape)  # Re-add potential padding
    return q_out


def bbox_to_bounding_box(bbox: BBox) -> BoundingBox:
    """Converts :class:`.BBox` to ROS :class:`geographic_msgs.msg.BoundingBox`"""
    return BoundingBox(
        min_pt=GeoPoint(latitude=bbox.bottom, longitude=bbox.left, altitude=np.nan),
        max_pt=GeoPoint(latitude=bbox.top, longitude=bbox.right, altitude=np.nan),
    )


def bounding_box_to_bbox(msg: BoundingBox) -> BBox:
    """Converts :class:`geographic_msgs.msg.BoundingBox` to :class:`.BBox`"""
    return BBox(
        msg.min_pt.longitude,
        msg.min_pt.latitude,
        msg.max_pt.longitude,
        msg.max_pt.latitude,
    )


def quaternion_to_rotation_matrix(q):
    """Convert a ROS2 geometry_msgs Quaternion to a numpy rotation matrix."""
    # Construct a rotation object from ROS2 geometry_msgs
    rotation = Rotation.from_quat([q.x, q.y, q.z, q.w])

    # Convert the rotation object to a rotation matrix
    rotation_matrix = rotation.as_matrix()

    return rotation_matrix


def rotation_matrix_to_quaternion(matrix):
    """Convert a numpy rotation matrix to a ROS2 geometry_msgs Quaternion."""

    # Construct a rotation object from the rotation matrix
    rotation = Rotation.from_matrix(matrix)

    # Convert the rotation object to a quaternion
    quat = rotation.as_quat()

    # Create a ROS2 geometry_msgs Quaternion message
    q = Quaternion()
    q.x = quat[0]
    q.y = quat[1]
    q.z = quat[2]
    q.w = quat[3]

    return q


def corners_to_bounding_box(corners):
    """
    Generates a geographic_msgs/BoundingBox message from the FOV corners.

    :param corners: A dictionary containing the coordinates (latitude, longitude)
        of the four corners of the FOV.
    :return: A geographic_msgs/BoundingBox message representing the bounding
        box of the FOV.
    """
    # Extract the latitudes and longitudes from the corners
    latitudes = [corner.lat for corner in corners.values()]
    longitudes = [corner.lon for corner in corners.values()]

    # Create the bounding box
    bounding_box = BoundingBox()
    bounding_box.min_latitude = min(latitudes)
    bounding_box.min_longitude = min(longitudes)
    bounding_box.max_latitude = max(latitudes)
    bounding_box.max_longitude = max(longitudes)

    return bounding_box


def off_nadir_angle_v2(quaternion: np.ndarray):
    """
    Off-nadir angle (magnitude) of quaternion assuming nadir origin

    :param quaternion: Quaternion in (x, y, z, w) format
    :raise: ValueError if quaternion is invalid
    """
    # Normalize the quaternion
    norm = np.linalg.norm(quaternion)
    if norm > 0:
        quaternion = quaternion / norm
    else:
        raise ValueError("Invalid quaternion: norm is zero")

    # Calculate rotation angle in radians
    angle = 2 * np.arccos(quaternion[3])

    # Convert to degrees
    angle_degrees = np.degrees(angle)

    return angle_degrees


def off_nadir_angle(q):
    # Rotated vector
    rotated_x = 2.0 * (q.x * q.z - q.w * q.y)
    rotated_y = 2.0 * (q.y * q.z + q.w * q.x)
    rotated_z = q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z

    # Down direction
    down_x, down_y, down_z = 0.0, 0.0, -1.0

    # Dot product of rotated vector and down direction
    dot_product = rotated_x * down_x + rotated_y * down_y + rotated_z * down_z

    # Clamp dot_product to avoid floating-point precision issues
    dot_product = max(min(dot_product, 1.0), -1.0)

    # Compute the angle between the rotated vector and down direction
    angle_rad = np.arccos(dot_product)

    # Convert the angle to degrees
    angle_deg = np.degrees(angle_rad)

    return angle_deg

@staticmethod
def to_proj_string(r, t, utm_zone):
    """Converts rotation matrix and translation vector into proj string

    :param r: Rotation matrix (3, 3) in local frame
    :param t: Translation vector (3, 1) in local frame
    :param utm_zone: Universal Transverse Mercator (UTM) zone
    :return: Proj string describing the local coordinate system
    """

    # Extract rotation angle from the rotation matrix (in radians)
    rotation_angle_rad = np.arctan2(r[1, 0], r[0, 0])

    # Convert to degrees
    rotation_angle_deg = np.degrees(rotation_angle_rad)

    # Extract translations
    translation_x, translation_y = t

    # Create PROJ string
    proj_string = f"+proj=utm +zone={utm_zone} +x_0={translation_x} +y_0={translation_y} +alpha={rotation_angle_deg} +units=m +ellps=WGS84"

    return proj_string

@staticmethod
def from_proj_string(proj_string):
    """Converts proj string into rotation matrix and translation vector

    :param proj_string: Proj string describing the local coordinate system
    :return: Rotation matrix (3, 3) and translation vector (3, 1)
    """

    # Regular expressions to match proj parameters
    re_zone = r"\+zone=(\d+)"
    re_x0 = r"\+x_0=([-\d.]+)"
    re_y0 = r"\+y_0=([-\d.]+)"
    re_alpha = r"\+alpha=([-\d.]+)"

    # Extract parameters from proj string
    utm_zone = int(re.search(re_zone, proj_string).group(1))
    translation_x = float(re.search(re_x0, proj_string).group(1))
    translation_y = float(re.search(re_y0, proj_string).group(1))
    rotation_angle_deg = float(re.search(re_alpha, proj_string).group(1))

    # Convert rotation angle to radians
    rotation_angle_rad = np.radians(rotation_angle_deg)

    # Construct rotation matrix
    r = np.array([[np.cos(rotation_angle_rad), -np.sin(rotation_angle_rad), 0],
                  [np.sin(rotation_angle_rad), np.cos(rotation_angle_rad), 0],
                  [0, 0, 1]])

    # Construct translation vector
    t = np.array([translation_x, translation_y, 0])

    return r, t, utm_zone
