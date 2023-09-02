"""Module containing functions to create mock :term:`ROS` messages for testing

These functions are shared between :ref:`Unit tests` and :ref:`Launch tests`
and are therefore kept in a separate module.

This module also defines default values for a starting global position and
orientation that are appropriate for the :term:`KSQL` airport simulation.
"""
import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix

from gisnav.messaging import create_header

VEHICLE_LATITUDE_DEGREES = 37.523640
"""Default mock :term:`vehicle` :term:`KSQL` airport starting location
:term:`WGS 84` latitude
"""


VEHICLE_LONGITUDE_DEGREES = -122.255122
"""Default mock :term:`vehicle` :term:`KSQL` airport starting location
:term:`WGS 84` longitude
"""


VEHICLE_ELLIPSOID_ALTITUDE_METERS = 100.0
"""Default mock :term:`vehicle` :term:`KSQL` airport starting location
:term:`ellipsoid` altitude in meters
"""


VEHICLE_ENU_QUATERNION = np.array([0.0, 0.0, 0.0, 1.0])
"""Default mock :term:`vehicle` :term:`KSQL` airport starting location
:term:`orientation` quaternion in :term:`ENU` frame in (x, y, z, w) format.
"""


def navsatfix(
    vehicle_lat_degrees: float = VEHICLE_LATITUDE_DEGREES,
    vehicle_lon_degrees: float = VEHICLE_LONGITUDE_DEGREES,
    vehicle_alt_ellipsoid_meters: float = VEHICLE_ELLIPSOID_ALTITUDE_METERS,
) -> NavSatFix:
    """Returns a mock :class:`sensor_msgs.msg.NavSatFix` :term:`ROS` message
    based on given :term:`vehicle` :term:`WGS 84` latitude and longitude
    coordinates and :term:`ellipsoid` altitude in meters.

    :param vehicle_lat_degrees: Vehicle WGS 84 latitude coordinate in degrees
    :param vehicle_lon_degrees: Vehicle WGS 84 longitude coordinate in degrees
    :param vehicle_alt_ellipsoid_meters: Vehicle ellipsoid altitude in meters
    :return: A :class:`sensor_msgs.msg.NavSatFix` message representing
        the vehicle's :term:`global position`
    """
    navsatfix_msg = NavSatFix()
    navsatfix_msg.header = create_header("base_link")
    navsatfix_msg.latitude = vehicle_lat_degrees
    navsatfix_msg.longitude = vehicle_lon_degrees
    navsatfix_msg.altitude = vehicle_alt_ellipsoid_meters
    return navsatfix_msg


def vehicle_pose(
    vehicle_enu_quaternion: np.ndarray = VEHICLE_ENU_QUATERNION,
) -> PoseStamped:
    """Returns a mock :class:`sensor_msgs.msg.PoseStamped` :term:`ROS` message
    based on given :term:`vehicle` :term:`ENU` quaternion.

    :param vehicle_enu_quaternion: Vehicle ENU quaternion in (x, y, z, w) format
    :return: A :class:`sensor_msgs.msg.PoseStamped` message representing
        the vehicle's :term:`orientation`
    """
    pose = PoseStamped()
    pose.pose.orientation.x = vehicle_enu_quaternion[0]
    pose.pose.orientation.y = vehicle_enu_quaternion[1]
    pose.pose.orientation.z = vehicle_enu_quaternion[2]
    pose.pose.orientation.w = vehicle_enu_quaternion[3]
    return pose
