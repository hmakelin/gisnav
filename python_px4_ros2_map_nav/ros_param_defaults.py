"""This module contains default values for the ROS parameters declared in
:meth:`python_px4_ros2_map_nav.map_nav_node.MapNavNode._declare_ros_params`."""
from python_px4_ros2_map_nav.util import LatLon


class Defaults:
    """Class that contains default values for ROS parameter server.

    These values are used by :meth:`python_px4_ros2_map_nav.map_nav_node.MapNavNode._declare_ros_params`. To run a
    different configuration, you should edit the input YAML file instead and leave these defaults untouched.
    """
    WMS_URL = 'http://localhost:8080/wms'
    WMS_VERSION = '1.1.1'
    WMS_LAYER = 'Imagery'
    WMS_SRS = 'EPSG:4326'
    WMS_REQUEST_TIMEOUT = 5  # seconds

    MISC_AFFINE_THRESHOLD = 5  # degrees
    MISC_PUBLISH_FREQUENCY = 40  # Hz
    MISC_EXPORT_POSITION = 'position.json'
    MISC_EXPORT_PROJECTION = 'projection.json'
    MISC_MAX_PITCH = 30  # degrees
    MISC_VISUALIZE_HOMOGRAPHY = False

    MAP_UPDATE_INITIAL_GUESS = LatLon(37.523640, -122.255122)  # ksql_airport.world
    MAP_UPDATE_UPDATE_DELAY = 1  # seconds
    MAP_UPDATE_DEFAULT_ALTITUDE = 130  # meters
    MAP_UPDATE_GIMBAL_PROJECTION = True
    MAP_UPDATE_MAX_PITCH = 30  # degrees
    MAP_UPDATE_MAX_MAP_RADIUS = 1000  # meters
    MAP_UPDATE_MAP_RADIUS_METERS_DEFAULT = 400  # meters
    MAP_UPDATE_UPDATE_MAP_CENTER_THRESHOLD = 50  # meters
    MAP_UPDATE_UPDATE_MAP_RADIUS_THRESHOLD = 50  # meters



