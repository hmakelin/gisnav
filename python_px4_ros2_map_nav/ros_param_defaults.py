"""This module contains default values for the ROS parameters declared in
:meth:`python_px4_ros2_map_nav.map_nav_node.MapNavNode._declare_ros_params`."""
from python_px4_ros2_map_nav.util import LatLon


class Defaults:
    """Class that contains default values for ROS parameter server.

    These values are used by :meth:`python_px4_ros2_map_nav.map_nav_node.MapNavNode._declare_ros_params`. To run a
    different configuration, you should edit the input YAML file instead and leave these defaults untouched.
    """
    WMS_URL = 'http://localhost:8080/wms'
    """WMS server endpoint URL"""

    WMS_VERSION = '1.1.1'
    """WMS server version"""

    WMS_LAYER = 'Imagery'
    """WMS server map layer name
    
    The layer should cover the flight area of the vehicle at high resolution.
    """

    WMS_SRS = 'EPSG:4326'
    """WMS server supported SRS"""

    WMS_REQUEST_TIMEOUT = 5
    """WMS client request timeout in seconds"""

    MISC_MOCK_GPS = True
    """Indicates that estimated vehicle position, velocity and heading should be published as a mock GPS message.
    
    Otherwise estimates will be published as vehicle visual odometry."""

    MISC_PUBLISH_FREQUENCY = 40
    """Publishing frequency for outgoing :class:`px4_msgs.msg.VehicleVisualOdometry` messages"""

    MISC_EXPORT_POSITION = 'position.json'
    """Filename for exporting GeoJSON containing estimated field of view and position"""

    MISC_EXPORT_PROJECTION = 'projection.json'
    """Filename for exporting GeoJSON containing projected field of view (FOV) and FOV center"""

    MISC_MAX_PITCH = 30
    """Maximum camera pitch in degrees from nadir for attempting a match against map
    
    See also :py:attr:`~MAP_UPDATE_MAP_PITCH`.
    """

    MISC_COVARIANCE_ESTIMATION_LENGTH = 20
    """Determines how many observations are used to estimate the covariance matrices for 
    :class:`px4_msgs.msg.VehicleVisualOdometry` messages.
    
    Too low a value may cause lead to too low an estimate of variance and cause instability in EKF2 local position
    estimate.
    """

    MAP_UPDATE_INITIAL_GUESS = LatLon(37.523640, -122.255122)  # ksql_airport.worl
    """Initial guess of vehicle's rough global position
    
    The default value is the starting position defined in the ksql_airport.world file. This position is used when no
    information regarding vehicle's global position has been received via the PX4-ROS 2 bridge (i.e. via
    :class:`px4_msgs.msg.VehicleGlobalPosition` or :class:`px4_msgs.msg.VehicleLocalPosition` message). This may happen
    for example when GPS fusion is turned off in PX4's EKF2_AID_MASK parameter.
    
    Because the guess is used to retrieve a map, it does not have to be a very precise estimate of the vehicle's
    location. It has to be precise enough that the field of view of the camera will eventually be contained within the
    map which is retrieved for the rough location.
    """

    MAP_UPDATE_UPDATE_DELAY = 1  # seconds
    """Delay in seconds for throttling WMS GetMap requests.
    
    When the camera is mounted on a gimbal and is not static, this delay should be set quite low to ensure that whenever
    camera field of view is moved to some other location, the map update request will follow very soon after. The field
    of view of the camera projected on the ground may move much faster than the vehicle itself.
    """

    MAP_UPDATE_DEFAULT_ALTITUDE = 130
    """Default altitude in meters used for WMS GetMap request map bounding box estimation
    
    This value is not used for estimating the altitude of the vehicle itself. A good value is the maximum or expected
    operating altitude of the vehicle.
    """

    MAP_UPDATE_GIMBAL_PROJECTION = True
    """Flag to enable map updates based on field of view (FOV) projected onto ground
    
    When this flag is enabled, map rasters are retrieved for the expected center of the camera FOV instead of the
    expected position of the vehicle, which increases the chances that the FOV is fully contained in the map raster.
    This increases the chances of getting a good map match.
    """

    MAP_UPDATE_MAX_PITCH = 30
    """Maximum camera pitch in degrees from nadir for attempting to update the stored map
    
    This limit only applies when camera field of view (FOV) projection - 'gimbal_projection' - is enabled. This value
    will prevent unnecessary WMS GetMap requests when the camera FOV is e.g. far in the horizon.
    
    See also :py:attr:`~MISC_MAP_PITCH`.
    """

    MAP_UPDATE_MAX_MAP_RADIUS = 1000
    """Maximum map radius for WMS GetMap request bounding boxes
    
    This limit prevents unintentionally requesting very large maps if camera field of view is projected far into the
    horizon. This may happen e.g. if :py:attr:`~MAP_UPDATE_MAX_PITCH` is set too high relative to the camera's vertical
    view angle.
    """

    MAP_UPDATE_MAP_RADIUS_METERS_DEFAULT = 400
    """Default radius for WMS GetMap request bounding boxes if nothing is provided"""

    MAP_UPDATE_UPDATE_MAP_CENTER_THRESHOLD = 50
    """A map translation threshold that, if exceeded, a new map is retrieved.
    
    This prevents unnecessary WMS GetMap requests to replace an old map with a new map that is almost from the same
    location. This parameter is used during :meth:`map_nav_node.MapNavNode._should_update_map` calls.
    """

    MAP_UPDATE_UPDATE_MAP_RADIUS_THRESHOLD = 50
    """A map radius threshold that, if exceeded, causes a new map to be retrieved.

    This prevents unnecessary WMS GetMap requests to replace an old map with a new map that is from the same location
    and almost the same size. This parameter is used during :meth:`map_nav_node.MapNavNode._should_update_map` calls.
    """




