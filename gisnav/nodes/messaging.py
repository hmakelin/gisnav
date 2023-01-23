"""Helper functions for ROS messaging"""
import time
import numpy as np
from typing import Union

from std_msgs.msg import Header
from geometry_msgs.msg import Quaternion
from geographic_msgs.msg import GeoPoint, GeoPointStamped, BoundingBox

from ..assertions import assert_type, assert_shape
from ..geo import GeoPt
from ..data import BBox


# region ROS topic names
ROS_TOPIC_BOUNDING_BOX = 'gisnav/bounding_box'
"""Name of ROS topics into which :class:`geographic_msgs.msg.BoundingBox` will be published"""

ROS_TOPIC_VEHICLE_GEOPOSE = 'gisnav/vehicle_geopose'
"""Name of ROS topic into which :class:`geographic_msgs.msg.GeoPoseStamped` will be published"""

ROS_TOPIC_VEHICLE_GEOPOSE_ESTIMATE = 'gisnav/vehicle_geopose/estimate'
"""Name of ROS topic into which :class:`geographic_msgs.msg.GeoPoseStamped` estimate will be published"""

ROS_TOPIC_VEHICLE_ALTITUDE = 'gisnav/vehicle_altitude'
"""Name of ROS topics into which :class:`mavros_msgs.msg.Altitude` will be published"""

ROS_TOPIC_VEHICLE_ALTITUDE_ESTIMATE = 'gisnav/vehicle_altitude/estimate'
"""Name of ROS topics into which :class:`mavros_msgs.msg.Altitude` estimate will be published"""

ROS_TOPIC_GIMBAL_QUATERNION = 'gisnav/gimbal_quaternion'
"""Name of ROS topics into which :class:`geometry_msgs.msg.Quaternion` will be published"""

ROS_TOPIC_HOME_GEOPOINT = 'gisnav/home_geopoint'
"""Name of ROS topics into which :class:`geographic_msgs.msg.GeoPointStamped` will be published"""

ROS_TOPIC_ORTHOIMAGE = 'gisnav/orthoimage_3d'
"""ROS publish topic for :class:`.OrthoImage3D` message"""

ROS_TOPIC_TERRAIN_ALTITUDE = 'gisnav/terrain_altitude'
"""ROS publish topic for :class:`mavros_msgs.msg.Altitude` message"""

ROS_TOPIC_TERRAIN_GEOPOINT = 'gisnav/terrain_geopoint'
"""ROS publish topic for :class:`geographic_msgs.msg.GeoPointStamped` message"""

ROS_TOPIC_EGM96_HEIGHT = 'gisnav/egm96_height'
"""ROS publish topic for EGM96 ellipsoid height :class:`std_msgs.msg.Float32` message"""

ROS_TOPIC_GPS_INPUT = '/mavros/gps_input/gps_input'
"""Name of ROS topic for outgoing :class:`mavros_msgs.msg.GPSINPUT` messages over MAVROS"""

ROS_TOPIC_SENSOR_GPS = '/fmu/in/sensor_gps'
"""Name of ROS topic for outgoing :class:`px4_msgs.msg.SensorGps` messages over PX4 microRTPS bridge"""
# endregion ROS topic names


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
    return Quaternion(x=q[0].item(), y=q[1].item(), z=q[2].item(), w=q[3].item())


def as_np_quaternion(q: Quaternion) -> np.ndarray:
    """Converts (x, y, z, w) format numpy array quaternion to ROS :class:`geometric_msg.msg.Quaternion`

    .. seealso:
        :meth:`.np_to_ros_quaternion`

    :param q: ROS quaternion message
    :return: NumPy array quaternion in (x, y, z, w) format
    """
    return np.array([q.x, q.y, q.z, q.w])


def wxyz_to_xyzw_q(q: np.ndarray) -> np.ndarray:
    """Converts quaternion from (w, x, y, z) to (x, y, z, w) format

    .. note::
        (w, x, y, z) is used by e.g. :class:`px4_msgs.msg.VehicleAttitude` while (x, y, z, w) is used by e.g.
        :class:`mavros_msgs.msg.Quaternion` and also SciPy.

    :param q: Quaternion in (w, x, y, z) format
    :return: Quaternion in (x, y, z, w) format
    """
    q_out = q.squeeze()
    assert_shape(q_out, (4,))
    q_out = np.append(q_out[1:], q_out[0])
    q_out = q_out.reshape(q.shape)  # Re-add potential padding
    return q_out


def geopoint_to_geopt(msg: Union[GeoPoint, GeoPointStamped]) -> GeoPt:
    """Convert :class:`geographic_msgs.msg.GeoPoint` or :class:`geographic_msgs.msg.GeoPointStamped` to :class:`.GeoPt`

    :param msg: ROS GeoPoint(Stamped) message
    :return: GeoPt instance
    """
    if isinstance(msg, GeoPoint):
        return GeoPt(x=msg.longitude, y=msg.latitude)
    else:
        assert isinstance(msg, GeoPointStamped)
        return GeoPt(x=msg.position.longitude, y=msg.position.latitude)


def bbox_to_bounding_box(bbox: BBox) -> BoundingBox:
    """Converts :class:`.BBox` to ROS :class:`geographic_msgs.msg.BoundingBox`"""
    return BoundingBox(
        min_pt=GeoPoint(latitude=bbox.bottom, longitude=bbox.left, altitude=np.nan),
        max_pt=GeoPoint(latitude=bbox.top, longitude=bbox.right, altitude=np.nan)
    )


def bounding_box_to_bbox(msg: BoundingBox) -> BBox:
    """Converts :class:`geographic_msgs.msg.BoundingBox` to :class:`.BBox`"""
    return BBox(msg.min_pt.longitude, msg.min_pt.latitude, msg.max_pt.longitude, msg.max_pt.latitude)
