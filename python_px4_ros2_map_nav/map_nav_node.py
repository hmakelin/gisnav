"""Module that contains the MapNavNode ROS 2 node."""
import rclpy
import os
import traceback
import yaml
import math
import cProfile
import io
import pstats
import numpy as np
import cv2
import time

# Import and configure torch for multiprocessing
import torch
try:
    torch.multiprocessing.set_start_method('spawn', force=True)
except RuntimeError:
    pass
torch.set_num_threads(1)

from multiprocessing.pool import Pool, AsyncResult  # Used for WMS client process, not for torch
from pyproj import Geod
from typing import Optional, Union, Tuple, get_args, List
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from owslib.wms import WebMapService
from geojson import Point, Polygon, Feature, FeatureCollection, dump

from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation
from functools import partial, lru_cache
from python_px4_ros2_map_nav.util import setup_sys_path, convert_from_pix_to_wgs84, BBox, Dim, get_bbox_center, \
    rotate_and_crop_map, visualize_homography, get_fov, LatLon, fov_center, get_angle, rotate_point, \
    create_src_corners, RPY, LatLonAlt, ImageFrame, assert_type, assert_ndim, assert_len, assert_shape, MapFrame
from python_px4_ros2_map_nav.ros_param_defaults import Defaults
from px4_msgs.msg import VehicleVisualOdometry, VehicleAttitude, VehicleLocalPosition, VehicleGlobalPosition, \
    GimbalDeviceAttitudeStatus, GimbalDeviceSetAttitude
from sensor_msgs.msg import CameraInfo, Image

# Add the share folder to Python path
share_dir, superglue_dir = setup_sys_path()

# Import this after util.setup_sys_path has been called
from python_px4_ros2_map_nav.superglue import SuperGlue


@lru_cache(maxsize=1)
def _cached_wms_client(url: str, version_: str, timeout_: int) -> WebMapService:
    """Returns a cached WMS client.

    The WMS client is instantiated in a dedicated process so it is lru_cache'd to avoid recurrent instantiations.

    :param url: WMS server endpoint url
    :param version_: WMS server version
    :param timeout_: WMS request timeout seconds
    :return:
    """
    assert_type(str, url)
    assert_type(str, version_)
    assert_type(int, timeout_)
    try:
        return WebMapService(url, version=version_, timeout=timeout_)
    except Exception as e:
        raise e  # TODO: handle gracefully (e.g. ConnectionRefusedError)


class MapNavNode(Node):
    """ROS 2 Node that publishes position estimate based on visual match of drone video to map of same location."""
    # scipy Rotations: {‘X’, ‘Y’, ‘Z’} for intrinsic, {‘x’, ‘y’, ‘z’} for extrinsic rotations
    EULER_SEQUENCE = 'YXZ'

    # Minimum matches for homography estimation, should be at least 4
    MINIMUM_MATCHES = 4

    # Encoding of input video (input to CvBridge)
    IMAGE_ENCODING = 'bgr8'  # E.g. gscam2 only supports bgr8 so this is used to override encoding in image header

    # Local frame reference for px4_msgs.msg.VehicleVisualOdometry messages
    LOCAL_FRAME_NED = 0

    # Ellipsoid model used by pyproj
    PYPROJ_ELLIPSOID = 'WGS84'

    # Default name of config file
    CONFIG_FILE_DEFAULT = "params.yml"

    # Minimum and maximum publish frequencies for EKF2 fusion
    MINIMUM_PUBLISH_FREQUENCY = 30
    MAXIMUM_PUBLISH_FREQUENCY = 50

    # Logs a warning if publish frequency is close to the bounds of desired publish frequency
    VVO_PUBLISH_FREQUENCY_WARNING_PADDING = 3

    # Maps properties to microRTPS bridge topics and message definitions
    # TODO: get rid of static TOPICS and dynamic _topics dictionaries - just use one dictionary, initialize it in constructor?
    TOPIC_NAME_KEY = 'topic_name'
    CLASS_KEY = 'class'
    SUBSCRIBE_KEY = 'subscribe'  # Used as key in both Matcher.TOPICS and Matcher._topics
    PUBLISH_KEY = 'publish'  # Used as key in both Matcher.TOPICS and Matcher._topics
    VEHICLE_VISUAL_ODOMETRY_TOPIC_NAME = 'VehicleVisualOdometry_PubSubTopic'  # TODO: Used when publishing, do this in some bette way
    TOPICS = [
        {
            TOPIC_NAME_KEY: 'VehicleLocalPosition_PubSubTopic',
            CLASS_KEY: VehicleLocalPosition,
            SUBSCRIBE_KEY: True
        },
        {
            TOPIC_NAME_KEY: 'VehicleGlobalPosition_PubSubTopic',
            CLASS_KEY: VehicleGlobalPosition,
            SUBSCRIBE_KEY: True
        },
        {
            TOPIC_NAME_KEY: 'VehicleAttitude_PubSubTopic',
            CLASS_KEY: VehicleAttitude,
            SUBSCRIBE_KEY: True
        },
        {
            TOPIC_NAME_KEY: 'GimbalDeviceAttitudeStatus_PubSubTopic',
            CLASS_KEY: GimbalDeviceAttitudeStatus,
            SUBSCRIBE_KEY: True
        },
        {
            TOPIC_NAME_KEY: 'GimbalDeviceSetAttitude_PubSubTopic',
            CLASS_KEY: GimbalDeviceSetAttitude,
            SUBSCRIBE_KEY: True
        },
        {
            TOPIC_NAME_KEY: 'camera_info',
            CLASS_KEY: CameraInfo,
            SUBSCRIBE_KEY: True
        },
        {
            TOPIC_NAME_KEY: 'image_raw',
            CLASS_KEY: Image,
            SUBSCRIBE_KEY: True
        },
        {
            TOPIC_NAME_KEY: VEHICLE_VISUAL_ODOMETRY_TOPIC_NAME,
            CLASS_KEY: VehicleVisualOdometry,
            PUBLISH_KEY: True
        }
    ]

    def __init__(self, node_name: str, share_directory: str, superglue_directory: str,
                 config: str = CONFIG_FILE_DEFAULT) -> None:
        """Initializes the ROS 2 node.

        :param node_name: Name of the node
        :param share_directory: Path of the share directory with configuration and other files
        :param superglue_directory: Path of the directory with SuperGlue related files
        :param config: Path to the config file in the share folder
        """
        assert_type(str, node_name)
        super().__init__(node_name)
        self.name = node_name
        assert_type(str, share_directory)
        assert_type(str, superglue_directory)
        assert_type(str, config)
        self._share_dir = share_directory
        self._superglue_dir = superglue_directory

        # Setup config and declare ROS parameters
        self._config = self._load_config(config)
        params = self._config.get(node_name, {}).get('ros__parameters')
        assert_type(dict, params)
        self._declare_ros_params(params)

        # WMS client and requests in a separate process
        self._wms_results = None  # Must check for None when using this
        self._wms_pool = Pool(1)  # Do not increase the process count, it should be 1

        # Setup map update timer
        self._map_update_timer = self._setup_map_update_timer()

        # Dict for storing all microRTPS bridge subscribers and publishers
        self._topics = {self.PUBLISH_KEY: {}, self.SUBSCRIBE_KEY: {}}
        self._setup_topics()

        # Setup vehicle visual odometry publisher timer
        self._publish_timer = self._setup_publish_timer()
        self._publish_timestamp = None

        # Converts image_raw to cv2 compatible image
        self._cv_bridge = CvBridge()

        # Setup SuperGlue
        self._stored_inputs = None  # Must check for None when using this
        self._superglue_results = None  # Must check for None when using this
        # Do not increase the process count, it should be 1
        self._superglue_pool = torch.multiprocessing.Pool(1, initializer=self._superglue_init_worker,
                                                          initargs=(self._config.get(self.name, {}), ))

        # Used for pyproj transformations
        self._geod = Geod(ellps=self.PYPROJ_ELLIPSOID)

        # Must check for None when using these
        # self._image_frame = None  # Not currently used / needed
        self._map_frame = None
        self._previous_map_frame = None

        self._local_origin = None  # Local frame origin WGS84 coordinates

        # Properties that are mapped to microRTPS bridge topics, must check for None when using them
        self._camera_info = None
        self._vehicle_local_position = None
        self._vehicle_global_position = None
        self._vehicle_attitude = None
        self._gimbal_device_attitude_status = None
        self._gimbal_device_set_attitude = None
        self._vehicle_visual_odometry = None  # To be published by _timer_callback (see _timer property)

    @property
    def name(self) -> dict:
        """Node name."""
        return self._name

    @name.setter
    def name(self, value: str) -> None:
        assert_type(str, value)
        self._name = value

    @property
    def _config(self) -> dict:
        """ROS parameters and other configuration info."""
        return self.__config

    @_config.setter
    def _config(self, value: dict) -> None:
        assert_type(dict, value)
        self.__config = value

    @property
    def _wms_pool(self) -> Pool:
        """Web Map Service client for fetching map rasters."""
        return self.__wms_pool

    @_wms_pool.setter
    def _wms_pool(self, value: Pool) -> None:
        assert_type(Pool, value)
        self.__wms_pool = value

    @property
    def _wms_results(self) -> Optional[AsyncResult]:
        """Asynchronous results from a WMS client request."""
        return self.__wms_results

    @_wms_results.setter
    def _wms_results(self, value: Optional[AsyncResult]) -> None:
        assert_type(get_args(Optional[AsyncResult]), value)
        self.__wms_results = value

    @property
    def _map_update_timer(self) -> rclpy.timer.Timer:
        """Timer for throttling map update WMS requests."""
        return self.__map_update_timer

    @_map_update_timer.setter
    def _map_update_timer(self, value: rclpy.timer.Timer) -> None:
        assert_type(rclpy.timer.Timer, value)
        self.__map_update_timer = value

    @property
    def _superglue_pool(self) -> torch.multiprocessing.Pool:
        """Pool for running SuperGlue in dedicated process."""
        return self.__superglue_pool

    @_superglue_pool.setter
    def _superglue_pool(self, value: torch.multiprocessing.Pool) -> None:
        # TODO assert type
        #assert_type(torch.multiprocessing.Pool, value)
        self.__superglue_pool = value

    @property
    def _stored_inputs(self) -> dict:
        """Inputs stored at time of launching a new asynchronous match that are needed for processing its results.

        See :meth:`~_process_matches` for description of keys and values stored in the dictionary.
        """
        return self.__stored_inputs

    @_stored_inputs.setter
    def _stored_inputs(self, value: Optional[dict]) -> None:
        assert_type(get_args(Optional[dict]), value)
        self.__stored_inputs = value

    @property
    def _superglue_results(self) -> Optional[AsyncResult]:
        """Asynchronous results from a SuperGlue process."""
        return self.__superglue_results

    @_superglue_results.setter
    def _superglue_results(self, value: Optional[AsyncResult]) -> None:
        assert_type(get_args(Optional[AsyncResult]), value)
        self.__superglue_results = value

    @property
    def _superglue(self) -> SuperGlue:
        """SuperGlue graph neural network (GNN) estimator for matching keypoints between images."""
        return self.__superglue

    @_superglue.setter
    def _superglue(self, value: SuperGlue) -> None:
        assert_type(SuperGlue, value)
        self.__superglue = value

    @property
    def _publish_timer(self) -> rclpy.timer.Timer:
        """Timer for controlling publish frequency of outgoing VehicleVisualOdometry messages."""
        return self.__timer

    @_publish_timer.setter
    def _publish_timer(self, value: rclpy.timer.Timer) -> None:
        assert_type(rclpy.timer.Timer, value)
        self.__timer = value

    @property
    def _publish_timestamp(self) -> Optional[int]:
        """Timestamp in of when last VehicleVisualOdometry message was published."""
        return self.__publish_timestamp

    @_publish_timestamp.setter
    def _publish_timestamp(self, value: Optional[int]) -> None:
        assert_type(get_args(Optional[int]), value)
        self.__publish_timestamp = value

    @property
    def _topics(self) -> dict:
        """Dictionary that stores all rclpy publishers and subscribers."""
        return self.__topics

    @_topics.setter
    def _topics(self, value: dict) -> None:
        assert_type(dict, value)
        self.__topics = value

    @property
    def _vehicle_visual_odometry(self) -> Optional[VehicleVisualOdometry]:
        """Outgoing VehicleVisualOdometry message waiting to be published."""
        return self.__vehicle_visual_odometry

    @_vehicle_visual_odometry.setter
    def _vehicle_visual_odometry(self, value: Optional[VehicleVisualOdometry]) -> None:
        assert_type(get_args(Optional[VehicleVisualOdometry]), value)
        self.__vehicle_visual_odometry = value

    @property
    def _geod(self) -> Geod:
        """Pyproj Geod for performing geodetic computations."""
        return self.__geod

    @_geod.setter
    def _geod(self, value: Geod) -> None:
        assert_type(Geod, value)
        self.__geod = value

    @property
    def _local_origin(self) -> Optional[LatLonAlt]:
        """Local frame origin WGS84 coordinates."""
        return self.__local_origin

    @_local_origin.setter
    def _local_origin(self, value: Optional[LatLonAlt]) -> None:
        assert_type(get_args(Optional[LatLonAlt]), value)
        self.__local_origin = value

    @property
    def _share_dir(self) -> str:
        """Path to share directory"""
        return self.__share_dir

    @_share_dir.setter
    def _share_dir(self, value: str) -> None:
        assert_type(str, value)
        self.__share_dir = value

    @property
    def _superglue_dir(self) -> str:
        """Path to SuperGlue directory."""
        return self.__superglue_dir

    @_superglue_dir.setter
    def _superglue_dir(self, value: str) -> None:
        assert_type(str, value)
        self.__superglue_dir = value

    @property
    def _map_frame(self) -> Optional[MapFrame]:
        """The map raster from the WMS server response along with supporting metadata."""
        return self.__map_frame

    @_map_frame.setter
    def _map_frame(self, value: Optional[MapFrame]) -> None:
        assert_type(get_args(Optional[MapFrame]), value)
        self.__map_frame = value

    @property
    def _cv_bridge(self) -> CvBridge:
        """CvBridge that decodes incoming PX4-ROS 2 bridge images to cv2 images."""
        return self.__cv_bridge

    @_cv_bridge.setter
    def _cv_bridge(self, value: CvBridge) -> None:
        assert_type(CvBridge, value)
        self.__cv_bridge = value

    @property
    def _previous_map_frame(self) -> Optional[MapFrame]:
        """The previous map frame which is compared to current map frame to determine need for another update."""
        return self.__previous_map_frame

    @_previous_map_frame.setter
    def _previous_map_frame(self, value: Optional[MapFrame]) -> None:
        assert_type(get_args(Optional[MapFrame]), value)
        self.__previous_map_frame = value

    @property
    def _camera_info(self) -> Optional[CameraInfo]:
        """CameraInfo received via the PX4-ROS 2 bridge."""
        return self.__camera_info

    @_camera_info.setter
    def _camera_info(self, value: Optional[CameraInfo]) -> None:
        assert_type(get_args(Optional[CameraInfo]), value)
        self.__camera_info = value

    @property
    def _vehicle_local_position(self) -> Optional[VehicleLocalPosition]:
        """VehicleLocalPosition received via the PX4-ROS 2 bridge."""
        return self.__vehicle_local_position

    @_vehicle_local_position.setter
    def _vehicle_local_position(self, value: Optional[VehicleLocalPosition]) -> None:
        assert_type(get_args(Optional[VehicleLocalPosition]), value)
        self.__vehicle_local_position = value

    @property
    def _vehicle_global_position(self) -> Optional[VehicleGlobalPosition]:
        """VehicleGlobalPosition received via the PX4-ROS 2 bridge."""
        return self.__vehicle_global_position

    @_vehicle_global_position.setter
    def _vehicle_global_position(self, value: Optional[VehicleGlobalPosition]) -> None:
        assert_type(get_args(Optional[VehicleGlobalPosition]), value)
        self.__vehicle_global_position = value

    @property
    def _vehicle_attitude(self) -> Optional[VehicleAttitude]:
        """VehicleAttitude received via the PX4-ROS 2 bridge."""
        return self.__vehicle_attitude

    @_vehicle_attitude.setter
    def _vehicle_attitude(self, value: Optional[VehicleAttitude]) -> None:
        assert_type(get_args(Optional[VehicleAttitude]), value)
        self.__vehicle_attitude = value

    @property
    def _gimbal_device_attitude_status(self) -> Optional[GimbalDeviceAttitudeStatus]:
        """GimbalDeviceAttitudeStatus received via the PX4-ROS 2 bridge."""
        return self.__gimbal_device_attitude_status

    @_gimbal_device_attitude_status.setter
    def _gimbal_device_attitude_status(self, value: Optional[GimbalDeviceAttitudeStatus]) -> None:
        assert_type(get_args(Optional[GimbalDeviceAttitudeStatus]), value)
        self.__gimbal_device_attitude_status = value

    @property
    def _gimbal_device_set_attitude(self) -> Optional[GimbalDeviceSetAttitude]:
        """GimbalDeviceSetAttitude received via the PX4-ROS 2 bridge."""
        return self.__gimbal_device_set_attitude

    @_gimbal_device_set_attitude.setter
    def _gimbal_device_set_attitude(self, value: Optional[GimbalDeviceSetAttitude]) -> None:
        assert_type(get_args(Optional[GimbalDeviceSetAttitude]), value)
        self.__gimbal_device_set_attitude = value

    def _setup_publish_timer(self) -> rclpy.timer.Timer:
        """Sets up a timer to control the publish rate of vehicle visual odometry.

        :return: The timer instance
        """
        frequency = self.get_parameter('misc.publish_frequency').get_parameter_value().integer_value
        assert_type(int, frequency)
        if not 0 <= frequency:
            error_msg = f'Publish frequency must be >0 Hz ({frequency} provided).'
            self.get_logger().error(error_msg)
            raise ValueError(error_msg)
        if not self.MINIMUM_PUBLISH_FREQUENCY <= frequency <= self.MAXIMUM_PUBLISH_FREQUENCY:
            warn_msg = f'Publish frequency should be between {self.MINIMUM_PUBLISH_FREQUENCY} and ' \
                       f'{self.MAXIMUM_PUBLISH_FREQUENCY} Hz ({frequency} provided) for EKF2 filter.'
            self.get_logger().warn(warn_msg)
        timer_period = 1.0 / frequency
        self.get_logger().debug(f'Setting up publish timer with period {timer_period} / frequency {frequency} Hz.')
        timer = self.create_timer(timer_period, self._vehicle_visual_odometry_timer_callback)
        return timer

    def _setup_map_update_timer(self) -> rclpy.timer.Timer:
        """Sets up a timer to throttle map update requests.

        :return: The timer instance
        """
        timer_period = self.get_parameter('map_update.update_delay').get_parameter_value().integer_value
        assert_type(int, timer_period)
        if not 0 <= timer_period:
            error_msg = f'Map update delay must be >0 seconds ({timer_period} provided).'
            self.get_logger().error(error_msg)
            raise ValueError(error_msg)
        timer = self.create_timer(timer_period, self._map_update_timer_callback)
        return timer

    def _latlonalt_from_vehicle_global_position(self) -> LatLonAlt:
        """Returns lat, lon and alt from VehicleGlobalPosition.

        The individual values of the LatLonAlt tuple may be None if vehicle global position is not available.

        :return: LatLonAlt, or None if not available."""
        lat, lon, alt = None, None, None
        if self._vehicle_global_position is not None:
            assert hasattr(self._vehicle_global_position, 'lat') and hasattr(self._vehicle_global_position, 'lon') and \
                   hasattr(self._vehicle_global_position, 'alt')
            lat, lon, alt = self._vehicle_global_position.lat, self._vehicle_global_position.lon, \
                            self._vehicle_global_position.alt
            assert_type(get_args(Union[int, float]), lat)
            assert_type(get_args(Union[int, float]), lon)
            assert_type(get_args(Union[int, float]), alt)
        return LatLonAlt(lat, lon, alt)

    def _alt_from_vehicle_local_position(self) -> Optional[float]:
        """Returns altitude from vehicle local position or None if not available.

        :return: Altitude in meters or None if not available"""
        if self._vehicle_local_position is not None:
            if self._vehicle_local_position.z_valid:
                self.get_logger().debug('Using VehicleLocalPosition.z for altitude.')
                return abs(self._vehicle_local_position.z)
            elif self._vehicle_local_position.dist_bottom_valid:
                self.get_logger().debug('Using VehicleLocalPosition.dist_bottom for altitude.')
                return abs(self._vehicle_local_position.dist_bottom)
            else:
                return None
        else:
            return None

    def _latlonalt_from_initial_guess(self) ->  Tuple[Optional[float], Optional[float], Optional[float]]:
        """Returns lat, lon and altitude from provided values, or None if not available."""
        return self.get_parameter('map_update.initial_guess.lat').get_parameter_value().double_value, \
               self.get_parameter('map_update.initial_guess.lon').get_parameter_value().double_value, \
               self.get_parameter('map_update.default_altitude').get_parameter_value().double_value

    def _map_update_timer_callback(self) -> None:
        """Attempts to update stored map at regular intervals.

        Calls :meth:`~_update_map` if the center and altitude coordinates for the new map raster are available and the
        :meth:`~_should_update_map` check passes.

        :return:
        """
        # Try to get lat, lon, alt from VehicleGlobalPosition if available
        latlonalt = self._latlonalt_from_vehicle_global_position()
        assert_type(LatLonAlt, latlonalt)

        # If altitude was not available in VehicleGlobalPosition, try to get it from VehicleLocalPosition
        if latlonalt.alt is None:
            self.get_logger().debug('Could not get altitude from VehicleGlobalPosition - trying VehicleLocalPosition '
                                   'instead.')
            latlonalt = LatLonAlt(latlonalt.lat, latlonalt.lon, self._alt_from_vehicle_local_position())

        # If some of latlonalt are still None, try to get from provided initial guess and default alt
        if not all(latlonalt):
            # Warn, not debug, since this is a static guess
            self.get_logger().warn('Could not get (lat, lon, alt) tuple from VehicleGlobalPosition nor '
                                   'VehicleLocalPosition, checking if initial guess has been provided.')
            latlonalt_guess = self._latlonalt_from_initial_guess()
            latlonalt = tuple(latlonalt[i] if latlonalt[i] is not None else latlonalt_guess[i] for i in
                              range(len(latlonalt)))
            latlonalt = LatLonAlt(*latlonalt)

        # Cannot determine vehicle global position
        if not all(latlonalt):
            self.get_logger().warn(f'Could not determine vehicle global position (latlonalt: {latlonalt}) and therefore'
                                   f' cannot update map.')
            return

        # Project principal point if required
        if self._use_gimbal_projection():
            fov_center_ = self._projected_field_of_view_center(latlonalt)
            if fov_center_ is None:
                self.get_logger().warn('Could not project field of view center. Using vehicle position for map center '
                                       'instead.')
            else:
                # Position at camera altitude but above the projected field of view center
                latlonalt = LatLonAlt(fov_center_.lat, fov_center_.lon, latlonalt.alt)

        # Get map size based on altitude and update map if needed
        map_radius = self._get_dynamic_map_radius(latlonalt.alt)
        if self._should_update_map(latlonalt, map_radius):
            self._update_map(latlonalt, map_radius)
        else:
            self.get_logger().debug('Map center and radius not changed enough to update map yet, '
                                    'or previous results are not ready.')

    def _vehicle_visual_odometry_timer_callback(self) -> None:
        """Publishes the vehicle visual odometry message at given intervals.

        :return:
        """
        if self._vehicle_visual_odometry is not None:
            assert_type(VehicleVisualOdometry, self._vehicle_visual_odometry)
            now = time.time_ns()
            if self._publish_timestamp is not None:
                assert now > self._publish_timestamp
                hz = 1e9 / (now - self._publish_timestamp)
                self.get_logger().debug(
                    f'Publishing vehicle visual odometry message:\n{self._vehicle_visual_odometry}. '
                    f'Publish frequency {hz} Hz.')

                # Warn if we are close to the bounds of acceptable frequency range
                warn_padding = self.VVO_PUBLISH_FREQUENCY_WARNING_PADDING
                if not self.MINIMUM_PUBLISH_FREQUENCY + warn_padding < hz < self.MAXIMUM_PUBLISH_FREQUENCY - warn_padding:
                    self.get_logger().warn(f'Publish frequency {hz} Hz is close to or outside of bounds of required '
                                           f'frequency range of [{self.MINIMUM_PUBLISH_FREQUENCY}, '
                                           f'{self.MAXIMUM_PUBLISH_FREQUENCY}] Hz for EKF2 fusion.')

            self._publish_timestamp = now
            self._topics.get(self.PUBLISH_KEY).get(self.VEHICLE_VISUAL_ODOMETRY_TOPIC_NAME)\
                .publish(self._vehicle_visual_odometry)
        else:
            self.get_logger().debug('Vehicle visual odometry publishing timer triggered but there was nothing to '
                                    'publish.')

    def _declare_ros_params(self, config: dict) -> None:
        """Declares ROS parameters from a config file.

        Uses defaults from :py:mod:`python_px4_ros2_map_nav.ros_param_defaults` if values are not provided.

        :param config: The value of the ros__parameters key from the parsed configuration file.
        :return:
        """
        namespace = 'wms'
        self.declare_parameters(namespace, [
            ('url', config.get(namespace, {}).get('url', Defaults.WMS_URL), ParameterDescriptor(read_only=True)),
            ('version', config.get(namespace, {})
             .get('version', Defaults.WMS_VERSION), ParameterDescriptor(read_only=True)),
            ('layer', config.get(namespace, {}).get('layer', Defaults.WMS_LAYER)),
            ('srs', config.get(namespace, {}).get('srs', Defaults.WMS_SRS)),
            ('request_timeout', config.get(namespace, {}).get('request_timeout', Defaults.WMS_REQUEST_TIMEOUT))
        ])

        namespace = 'misc'
        self.declare_parameters(namespace, [
            ('affine_threshold', config.get(namespace, {}).get('affine_threshold', Defaults.MISC_AFFINE_THRESHOLD)),
            ('publish_frequency', config.get(namespace, {})
             .get('publish_frequency', Defaults.MISC_PUBLISH_FREQUENCY), ParameterDescriptor(read_only=True)),
            ('export_position', config.get(namespace, {}).get('export_position', Defaults.MISC_EXPORT_POSITION)),
            ('export_projection', config.get(namespace, {}).get('export_projection', Defaults.MISC_EXPORT_PROJECTION)),
            ('max_pitch', config.get(namespace, {}).get('max_pitch', Defaults.MISC_MAX_PITCH)),
            ('visualize_homography', config.get(namespace, {})
             .get('visualize_homography', Defaults.MISC_VISUALIZE_HOMOGRAPHY))
        ])

        namespace = 'map_update'
        self.declare_parameters(namespace, [
            ('initial_guess.lat', config.get(namespace, {}).get('initial_guess', {})
             .get('lat', Defaults.MAP_UPDATE_INITIAL_GUESS.lat)),
            ('initial_guess.lon', config.get(namespace, {}).get('initial_guess', {})
             .get('lon', Defaults.MAP_UPDATE_INITIAL_GUESS.lon)),
            ('update_delay', config.get(namespace, {})
             .get('update_delay', Defaults.MAP_UPDATE_UPDATE_DELAY), ParameterDescriptor(read_only=True)),
            ('default_altitude', config.get(namespace, {})
             .get('default_altitude', Defaults.MAP_UPDATE_DEFAULT_ALTITUDE)),
            ('gimbal_projection', config.get(namespace, {})
             .get('gimbal_projection', Defaults.MAP_UPDATE_GIMBAL_PROJECTION)),
            ('max_map_radius', config.get(namespace, {}).get('max_map_radius', Defaults.MAP_UPDATE_MAX_MAP_RADIUS)),
            ('map_radius_meters_default', config.get(namespace, {})
             .get('map_radius_meters_default', Defaults.MAP_UPDATE_MAP_RADIUS_METERS_DEFAULT)),
            ('update_map_center_threshold', config.get(namespace, {})
             .get('update_map_center_threshold', Defaults.MAP_UPDATE_UPDATE_MAP_CENTER_THRESHOLD)),
            ('update_map_radius_threshold', config.get(namespace, {})
             .get('update_map_radius_threshold', Defaults.MAP_UPDATE_UPDATE_MAP_RADIUS_THRESHOLD)),
            ('max_pitch', config.get(namespace, {}).get('max_pitch', Defaults.MAP_UPDATE_MAX_PITCH))
        ])

    def _load_config(self, yaml_file: str) -> dict:
        """Loads config from the provided YAML file.

        :param yaml_file: Path to the yaml file
        :return: The loaded yaml file as dictionary
        """
        assert_type(str, yaml_file)
        with open(os.path.join(self._share_dir, yaml_file), 'r') as f:
            try:
                config = yaml.safe_load(f)
                self.get_logger().info(f'Loaded config:\n{config}.')
                return config
            except Exception as e:
                self.get_logger().error(f'Could not load config file {yaml_file} because of exception:'
                                        f'\n{e}\n{traceback.print_exc()}')

    def _use_gimbal_projection(self) -> bool:
        """Checks if map rasters should be retrieved for projected field of view instead of vehicle position.

        :return: True if projected principal point should be used.
        """
        gimbal_projection_flag = self.get_parameter('map_update.gimbal_projection').get_parameter_value().bool_value
        if type(gimbal_projection_flag) is bool:
            return gimbal_projection_flag
        else:
            self.get_logger().warn(f'Could not read gimbal projection flag: {gimbal_projection_flag}. Assume False.')
            return False

    def _restrict_affine(self) -> bool:
        """Checks if homography matrix should be restricted to an affine transformation (nadir facing camera).

        :return: True if homography matrix should be restricted to a 2D affine transformation.
        """
        restrict_affine_threshold = self.get_parameter('misc.affine_threshold').get_parameter_value().integer_value
        assert_type(get_args(Union[int, float]), restrict_affine_threshold)
        camera_pitch = self._camera_pitch()
        if camera_pitch is not None:
            if abs(camera_pitch) <= restrict_affine_threshold:
                return True
            else:
                return False
        else:
            self.get_logger().warn(f'Could not get camera pitch - cannot assume affine 2D transformation.')
            return False

    def _setup_topics(self) -> None:
        """Creates and stores publishers and subscribers for microRTPS bridge topics.

        :return:
        """
        for topic in self.TOPICS:
            topic_name = topic.get(self.TOPIC_NAME_KEY, None)
            class_ = topic.get(self.CLASS_KEY, None)
            assert topic_name is not None, f'Topic name not provided in topic: {topic}.'
            assert class_ is not None, f'Class not provided in topic: {topic}.'

            publish = topic.get(self.PUBLISH_KEY, None)
            if publish is not None:
                assert_type(bool, publish)
                self._topics.get(self.PUBLISH_KEY).update({topic_name: self._create_publisher(topic_name, class_)})

            subscribe = topic.get(self.SUBSCRIBE_KEY, None)
            if subscribe is not None:
                assert_type(bool, subscribe)
                self._topics.get(self.SUBSCRIBE_KEY).update({topic_name: self._create_subscriber(topic_name, class_)})

        self.get_logger().info(f'Topics setup complete:\n{self._topics}.')

    def _create_publisher(self, topic_name: str, class_: object) -> rclpy.publisher.Publisher:
        """Sets up an rclpy publisher.

        :param topic_name: Name of the microRTPS topic
        :param class_: Message definition class (e.g. px4_msgs.msg.VehicleVisualOdometry)
        :return: The publisher instance
        """
        return self.create_publisher(class_, topic_name, 10)  # TODO: add explicit QoSProfile instead of depth

    def _create_subscriber(self, topic_name: str, class_: object) -> rclpy.subscription.Subscription:
        """Sets up an rclpy subscriber.

        :param topic_name: Name of the microRTPS topic
        :param class_: Message definition class (e.g. px4_msgs.msg.VehicleLocalPosition)
        :return: The subscriber instance
        """
        callback_name = topic_name.lower() + '_callback'
        callback = getattr(self, callback_name, None)
        assert callback is not None, f'Missing callback implementation for {callback_name}.'
        return self.create_subscription(class_, topic_name, callback, 10)  # TODO: add explicit QoSProfile

    def _get_bbox(self, latlon: Union[LatLon, LatLonAlt], radius_meters: Optional[Union[int, float]] = None) -> BBox:
        """Gets the bounding box containing a circle with given radius centered at given lat-lon fix.

        :param latlon: Center of the bounding box
        :param radius_meters: Radius of the circle in meters enclosed by the bounding box
        :return: The bounding box
        """
        if radius_meters is None:
            radius_meters = self.get_parameter('map_update.map_radius_meters_default')\
                .get_parameter_value().integer_value
        assert_type(get_args(Union[LatLon, LatLonAlt]), latlon)
        assert_type(get_args(Union[int, float]), radius_meters)
        corner_distance = math.sqrt(2) * radius_meters  # Distance to corner of square enclosing circle of radius
        ul = self._move_distance(latlon, (-45, corner_distance))
        lr = self._move_distance(latlon, (135, corner_distance))
        return BBox(ul.lon, lr.lat, lr.lon, ul.lat)

    def _get_distance_of_fov_center(self, fov_wgs84: np.ndarray) -> float:
        """Calculate distance between middle of sides of field of view (FOV) based on triangle similarity.

        :param fov_wgs84: The WGS84 corner coordinates of the FOV
        :return:
        """
        # TODO: assert shape of fov_wgs84
        midleft = ((fov_wgs84[0] + fov_wgs84[1]) * 0.5).squeeze()
        midright = ((fov_wgs84[2] + fov_wgs84[3]) * 0.5).squeeze()
        _, __, dist = self._geod.inv(midleft[1], midleft[0], midright[1], midright[0])  # TODO: use distance method here
        return dist

    def _distance(self, latlon1: Union[LatLon, LatLonAlt], latlon2: Union[LatLon, LatLonAlt]) -> float:
        """Returns distance between two points in meters.

        :param latlon1: The first point
        :param latlon2: The second point
        :return: The distance in meters
        """
        assert_type(get_args(Union[LatLon, LatLonAlt]), latlon1)
        assert_type(get_args(Union[LatLon, LatLonAlt]), latlon2)
        _, __, dist = self._geod.inv(latlon1.lon, latlon1.lat, latlon2.lon, latlon2.lat)
        return dist

    def _move_distance(self, latlon: Union[LatLon, LatLonAlt], azmth_dist: Tuple[Union[int, float], Union[int, float]])\
            -> LatLon:
        """Returns the point that is a given distance in the direction of azimuth from the origin point.

        :param latlon: Origin point
        :param azmth_dist: Tuple containing azimuth in degrees and distance in meters: (azimuth, distance)
        :return: The point that is given meters away in the azimuth direction from origin
        """
        assert_type(tuple, azmth_dist)
        assert_type(get_args(Union[LatLon, LatLonAlt]), latlon)
        azmth, dist = azmth_dist  # TODO: silly way of providing these args just to map over a zipped list in _update_map, fix it
        assert_type(get_args(Union[int, float]), azmth)
        assert_type(get_args(Union[int, float]), dist)
        lon, lat, azmth = self._geod.fwd(latlon.lon, latlon.lat, azmth, dist)
        return LatLon(lat, lon)

    def _map_size_with_padding(self) -> Optional[Tuple[int, int]]:
        """Returns map size with padding for rotation without clipping corners.

        :return: Map size tuple (height, width) or None if the info is not available
        """
        dim = self._img_dim()
        if dim is None:
            self.get_logger().warn(f'Dimensions not available - returning None as map size.')
            return None
        assert_type(Dim, dim)
        diagonal = math.ceil(math.sqrt(dim.width ** 2 + dim.height ** 2))
        assert_type(int, diagonal)  # TODO: What if this is float?
        return diagonal, diagonal

    def _map_dim_with_padding(self) -> Optional[Dim]:
        """Returns map dimensions with padding for rotation without clipping corners.

        :return: Map dimensions or None if the info is not available
        """
        map_size = self._map_size_with_padding()
        if map_size is None:
            self.get_logger().warn(f'Map size with padding not available - returning None as map dimensions.')
            return None
        assert_type(tuple, map_size)
        assert_len(map_size, 2)
        return Dim(*map_size)

    def _declared_img_size(self) -> Optional[Tuple[int, int]]:
        """Returns image resolution size as it is declared in the latest CameraInfo message.

        :return: Image resolution tuple (height, width) or None if not available
        """
        if self._camera_info is not None:
            # TODO: assert or check hasattr?
            return self._camera_info.height, self._camera_info.width  # numpy order: h, w, c --> height first
        else:
            self.get_logger().warn('Camera info was not available - returning None as declared image size.')
            return None

    def _img_dim(self) -> Optional[Dim]:
        """Returns image dimensions as it is declared in the latest CameraInfo message.

        :return: Image dimensions or None if not available
        """
        declared_size = self._declared_img_size()
        if declared_size is None:
            self.get_logger().warn('CDeclared size not available - returning None as image dimensions.')
            return None
        assert_type(tuple, declared_size)
        assert_len(declared_size, 2)
        return Dim(*declared_size)

    def _project_gimbal_fov(self, translation: np.ndarray) -> Optional[np.ndarray]:
        """Returns field of view (FOV) BBox projected using gimbal attitude and camera intrinsics information.

        :param altitude_meters: Altitude of camera in meters  # TODO: why is altitude an arg but other info are retrieved inside the function?
        :return: Projected FOV bounding box in pixel coordinates or None if not available
        """
        assert_shape(translation, (3,))
        rpy = self._get_camera_rpy()
        if rpy is None:
            self.get_logger().warn('Could not get RPY - cannot project gimbal fov.')
            return None

        r = Rotation.from_euler(self.EULER_SEQUENCE, list(rpy), degrees=True).as_matrix()
        e = np.hstack((r, np.expand_dims(translation, axis=1)))
        assert_shape(e, (3, 4))

        if self._camera_info is None:
            self.get_logger().warn('Could not get camera info - cannot project gimbal fov.')
            return None
        h, w = self._img_dim()
        # TODO: assert h w not none and integers? and divisible by 2?

        # Intrinsic matrix
        k = np.array(self._camera_info.k).reshape([3, 3])

        # Project image corners to z=0 plane (ground)
        src_corners = create_src_corners(h, w)
        assert_shape(src_corners, (4, 1, 2))
        e = np.delete(e, 2, 1)  # Remove z-column, making the matrix square
        p = np.matmul(k, e)
        try:
            p_inv = np.linalg.inv(p)
        except np.linalg.LinAlgError as e:
            self.get_logger().error(f'Could not invert the projection matrix: {p}. RPY was {rpy}. Trace:'
                                    f'\n{e},\n{traceback.print_exc()}.')
            return None

        assert_shape(p_inv, (3, 3))

        dst_corners = cv2.perspectiveTransform(src_corners, p_inv)  # TODO: use util.get_fov here?
        assert_shape(dst_corners, src_corners.shape)
        dst_corners = dst_corners.squeeze()  # TODO: See get_fov usage elsewhere -where to do squeeze if at all?

        return dst_corners

    def _vehicle_local_position_ref_latlonalt_timestamp(self) -> Tuple[Optional[LatLonAlt], Optional[int]]:
        """Returns vehicle local frame reference origin.

        :return: Local reference frame origin location in WGS84 or None if not available
        """
        if self._vehicle_local_position is None:
            self.get_logger().warn('Could not get vehicle local position - returning None as local frame reference.')
            return None, None

        if self._vehicle_local_position.xy_global is True and self._vehicle_local_position.z_global is True:
            assert_type(int, self._vehicle_local_position.timestamp)
            return LatLonAlt(self._vehicle_local_position.ref_lat, self._vehicle_local_position.ref_lon,
                             self._vehicle_local_position.ref_alt), self._vehicle_local_position.timestamp
        else:
            # TODO: z may not be needed - make a separate _ref_latlon method!
            self.get_logger().warn('No valid global reference for local frame origin - returning None.')
            return None, None

    @staticmethod
    def _get_azimuth(x: float, y: float) -> float:
        """Get azimuth of position x and y coordinates.

        :param x: Meters towards east
        :param y: Meters towards north

        :return: Azimuth in degrees
        """
        rads = math.atan2(y, x)
        rads = rads if rads > 0 else rads + 2*math.pi  # Counter-clockwise from east
        rads = -rads + math.pi/2  # Clockwise from north
        return math.degrees(rads)

    def _projected_field_of_view_center(self, origin: LatLonAlt) -> Optional[LatLon]:
        """Returns WGS84 coordinates of projected camera field of view (FOV).

        :param origin: Camera position  # TODO: why is this an argument but all else is not?
        :return: Center of the FOV or None if not available
        """
        if self._camera_info is not None:
            pitch = self._camera_pitch()  # TODO: _project_gimbal_fov uses _get_camera_rpy - redundant calls
            if pitch is None:
                self.get_logger().warn('Camera pitch not available, cannot project gimbal field of view.')
                return None
            assert 0 <= abs(pitch) <= 90, f'Pitch {pitch} was outside of expected bounds [0, 90].' # TODO: need to handle outside of bounds, cannot assert
            pitch_rad = math.radians(pitch)
            assert origin.alt is not None
            assert hasattr(origin, 'alt')
            hypotenuse = origin.alt * math.tan(pitch_rad)  # Distance from camera origin to projected principal point
            cx = hypotenuse*math.sin(pitch_rad)
            cy = hypotenuse*math.cos(pitch_rad)
            translation = np.array([-cx, -cy, origin.alt])
            gimbal_fov_pix = self._project_gimbal_fov(translation)

            # Convert gimbal field of view from pixels to WGS84 coordinates
            if gimbal_fov_pix is not None:
                azmths = list(map(lambda x: self._get_azimuth(x[0], x[1]), gimbal_fov_pix))
                dists = list(map(lambda x: math.sqrt(x[0] ** 2 + x[1] ** 2), gimbal_fov_pix))
                zipped = list(zip(azmths, dists))
                to_wgs84 = partial(self._move_distance, origin)
                gimbal_fov_wgs84 = np.array(list(map(to_wgs84, zipped)))
                ### TODO: add some sort of assertion hat projected FoV is contained in size and makes sense

                # Use projected field of view center instead of global position as map center
                map_center_latlon = fov_center(gimbal_fov_wgs84)  # TODO: use cx, cy and not fov corners, polygon center != principal point

                # Export to file in GIS readable format
                export_projection = self.get_parameter('misc.export_projection').get_parameter_value().string_value
                if export_projection is not None:
                    self._export_position(map_center_latlon, gimbal_fov_wgs84, export_projection)
            else:
                self.get_logger().warn('Could not project camera FoV, getting map raster assuming nadir-facing camera.')
                return None
        else:
            self.get_logger().debug('Camera info not available, cannot project FoV, defaulting to global position.')
            return None

        return map_center_latlon  # TODO: using principal point for updating map no good, too close to bottom fov. Principal point still needed but not for updating map.

    def _update_map(self, center: Union[LatLon, LatLonAlt], radius: Union[int, float]) -> None:
        """Gets latest map from WMS server for given location and radius and saves it.

        :param center: WGS84 coordinates of map to be retrieved
        :param radius: Radius in meters of circle to be enclosed by the map raster
        :return:
        """
        self.get_logger().info(f'Updating map at {center}, radius {radius} meters.')
        assert_type(get_args(Union[LatLon, LatLonAlt]), center)
        assert_type(get_args(Union[int, float]), radius)
        max_radius = self.get_parameter('map_update.max_map_radius').get_parameter_value().integer_value
        # TODO: need to recover from this, e.g. if its more than max_radius, warn and use max instead. Users could crash this by setting radius to above max radius
        assert 0 < radius <= max_radius, f'Radius should be between 0 and {max_radius}.'

        bbox = self._get_bbox(center, radius)  # TODO: should these things be moved to args? Move state related stuff up the call stack all in the same place. And isnt this a static function anyway?
        assert_type(BBox, bbox)

        map_size = self._map_size_with_padding()
        if map_size is None:
            self.get_logger().warn('Map size not yet available - skipping WMS request.')
            return None

        # Build and send WMS request
        url = self.get_parameter('wms.url').get_parameter_value().string_value
        version = self.get_parameter('wms.version').get_parameter_value().string_value
        layer_str = self.get_parameter('wms.layer').get_parameter_value().string_value
        srs_str = self.get_parameter('wms.srs').get_parameter_value().string_value
        assert_type(str, url)
        assert_type(str, version)
        assert_type(str, layer_str)
        assert_type(str, srs_str)
        try:
            self.get_logger().info(f'Getting map for bbox: {bbox}, layer: {layer_str}, srs: {srs_str}.')
            if self._wms_results is not None:
                assert self._wms_results.ready(), f'Update map was called while previous results were not yet ready.'  # Should not happen - check _should_update_map conditions
            timeout = self.get_parameter('wms.request_timeout').get_parameter_value().integer_value
            self._wms_results = self._wms_pool.starmap_async(
                self._wms_pool_worker, [(LatLon(center.lat, center.lon), radius, bbox, map_size, url, version,  # TODO: conersion of center to LatLon may be redundant?
                                         layer_str, srs_str, timeout)],
                callback=self.wms_pool_worker_callback, error_callback=self.wms_pool_worker_error_callback)
        except Exception as e:
            self.get_logger().error(f'Something went wrong with WMS worker:\n{e},\n{traceback.print_exc()}.')
            return None

    def wms_pool_worker_callback(self, result: List[MapFrame]) -> None:
        """Handles result from WMS pool worker.

        :param result: Results from the asynchronous call (a collection containing a single MapFrame)
        :return:
        """
        assert_len(result, 1)
        result = result[0]
        self.get_logger().info(f'WMS callback for bbox: {result.bbox}.')
        assert_type(MapFrame, result)
        if self._map_frame is not None:
            self._previous_map_frame = self._map_frame
        self._map_frame = result
        assert self._map_frame.image.shape[0:2] == self._map_size_with_padding(), \
            'Decoded map is not the specified size.'  # TODO: make map size with padding an argument?

    def wms_pool_worker_error_callback(self, e: BaseException) -> None:
        """Handles errors from WMS pool worker.
        :param e: Exception returned by the worker
        :return:
        """
        self.get_logger().error(f'Something went wrong with WMS process:\n{e},\n{traceback.print_exc()}.')

    @staticmethod
    def _wms_pool_worker(center: LatLon, radius: Union[int, float], bbox: BBox,
                         map_size: Tuple[int, int], url: str, version: str, layer_str: str, srs_str: str, timeout: int)\
            -> MapFrame:
        """Gets latest map from WMS server for given location and radius and returns it.

        :param center: Center of the map to be retrieved
        :param radius: Radius in meters of the circle to be enclosed by the map
        :param bbox: Bounding box of the map
        :param map_size: Map size tuple (height, width)
        :param url: WMS server url
        :param version: WMS server version
        :param layer_str: WMS server layer
        :param srs_str: WMS server SRS
        :param timeout: WMS client request timeout in seconds
        :return: MapFrame containing the map raster and supporting metadata
        """
        """"""
        # TODO: computation of bbox could be pushed in here - would just need to make Matcher._get_bbox pickle-able
        assert_type(str, url)
        assert_type(str, version)
        assert_type(int, timeout)
        wms_client = _cached_wms_client(url, version, timeout)
        assert wms_client is not None
        assert_type(BBox, bbox)
        assert(all(isinstance(x, int) for x in map_size))
        assert_type(str, layer_str)
        assert_type(str, srs_str)
        assert_type(LatLon, center)
        assert_type(get_args(Union[int, float]), radius)
        try:
            map_ = wms_client.getmap(layers=[layer_str], srs=srs_str, bbox=bbox, size=map_size, format='image/png',
                                     transparent=True)
            # TODO: what will map_ be if the reqeust times out? will an error be raised?
        except Exception as e:
            raise e  # TODO: need to do anything here or just pass it on?

        # Decode response from WMS server
        map_ = np.frombuffer(map_.read(), np.uint8)
        map_ = cv2.imdecode(map_, cv2.IMREAD_UNCHANGED)
        assert_type(np.ndarray, map_)
        assert_ndim(map_, 3)
        map_frame = MapFrame(center, radius, bbox, map_)
        return map_frame

    @staticmethod
    def _superglue_init_worker(config: dict):
        """Initializes SuperGlue in a dedicated process.

        :param config: SuperGlue config
        :return:
        """
        superglue_conf = config.get('superglue', None)
        assert_type(dict, superglue_conf)
        global superglue
        superglue = SuperGlue(superglue_conf)

    @staticmethod
    def _superglue_pool_worker(img: np.ndarray, map_: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Finds matching keypoints between input images.

        :param img: The first image
        :param map_: The second image
        :return: Tuple of two lists containing matching keypoints in img and map, respectively
        """
        """"""
        assert_type(np.ndarray, img)
        assert_type(np.ndarray, map_)
        try:
            return superglue.match(img, map_)
        except Exception as e:
            raise e  # TODO: need to do anything here or just pass it on?

    def image_raw_callback(self, msg: Image) -> None:
        """Handles latest image frame from camera.

        :param msg: The Image message from the PX4-ROS 2 bridge to decode
        :return:
        """
        self.get_logger().debug('Camera image callback triggered.')
        assert_type(Image, msg)

        cv_image = self._cv_bridge.imgmsg_to_cv2(msg, self.IMAGE_ENCODING)

        # Check that image dimensions match declared dimensions
        img_size = self._declared_img_size()
        if img_size is not None:
            cv_img_shape = cv_image.shape[0:2]
            assert cv_img_shape == img_size, f'Converted cv_image shape {cv_img_shape} did not match declared image ' \
                                             f'shape {img_size}.'

        # Process image frame
        image_frame = ImageFrame(cv_image, msg.header.frame_id, msg.header.stamp)

        # TODO: store image_frame as self._image_frame and move the stuff below into a dedicated self._matching_timer?
        if self._should_match():
            assert self._superglue_results is None or self._superglue_results.ready()
            inputs = self._match_inputs(image_frame)
            for k, v in inputs.items():
                if v is None:
                    if k not in ['local_frame_origin_position', 'timestamp']:  # TODO: use self._local_origin here to get rid of this clause?
                        self.get_logger().warn(f'Key {k} value {v} in match input arguments, cannot process matches.')
                        return

            camera_yaw = inputs.get('camera_yaw', None)
            map_frame = inputs.get('map_frame', None)
            img_dim = inputs.get('img_dim', None)
            assert all((camera_yaw, map_frame, img_dim))  # Redundant (see above 'for k, v in inputs.items(): ...')

            self._stored_inputs = inputs
            map_cropped = inputs.get('map_cropped')
            assert_type(np.ndarray, map_cropped)

            self.get_logger().debug(f'Matching image with timestamp {image_frame.stamp} to map.')
            self._match(image_frame, map_cropped)

    def _camera_yaw(self) -> Optional[int]:  # TODO: int or float?
        """Returns camera yaw in degrees.

        :return: Camera yaw in degrees, or None if not available
        """
        rpy = self._get_camera_rpy()
        if rpy is None:
            self.get_logger().warn(f'Could not get camera RPY - cannot return yaw.')
            return None
        assert_type(RPY, rpy)
        camera_yaw = rpy.yaw
        return camera_yaw

    def _get_camera_rpy(self) -> Optional[RPY]:
        """Returns roll-pitch-yaw euler vector.

        :return: An RPY tuple
        """
        gimbal_attitude = self._gimbal_attitude()
        if gimbal_attitude is None:
            self.get_logger().warn('Gimbal attitude not available, cannot return RPY.')
            return None
        assert hasattr(gimbal_attitude, 'q'), 'Gimbal attitude quaternion not available - cannot compute RPY.'

        pitch_index = self._pitch_index()
        assert pitch_index != -1, 'Could not identify pitch index in gimbal attitude, cannot return RPY.'

        yaw_index = self._yaw_index()
        assert yaw_index != -1, 'Could not identify yaw index in gimbal attitude, cannot return RPY.'

        gimbal_euler = Rotation.from_quat(gimbal_attitude.q).as_euler(self.EULER_SEQUENCE, degrees=True)
        if self._vehicle_local_position is None:
            self.get_logger().warn('VehicleLocalPosition is unknown, cannot get heading. Cannot return RPY.')
            return None

        heading = self._vehicle_local_position.heading
        heading = math.degrees(heading)
        assert -180 <= heading <= 180, f'Unexpected heading value: {heading} degrees ([-180, 180] expected).'
        gimbal_yaw = gimbal_euler[yaw_index]
        assert -180 <= gimbal_yaw <= 180, f'Unexpected gimbal yaw value: {gimbal_yaw} ([-180, 180] expected).'

        pitch = -(90 + gimbal_euler[pitch_index])  # TODO: ensure abs(pitch) <= 90?
        if pitch < 0:
            # Gimbal pitch and yaw flip over when abs(gimbal_yaw) should go over 90, adjust accordingly
            assert self.EULER_SEQUENCE == 'YXZ'  # Tested with 'YXZ' this configuration
            gimbal_yaw = 180 - gimbal_yaw
            pitch = 180 + pitch

        self.get_logger().debug('Assuming stabilized gimbal - ignoring vehicle intrinsic pitch and roll for camera RPY.')
        self.get_logger().debug('Assuming zero roll for camera RPY.')  # TODO remove zero roll assumption

        yaw = heading + gimbal_yaw
        yaw = yaw % 360
        if abs(yaw) > 180:  # Important: >, not >= (because we are using mod 180 operation below)
            yaw = yaw % 180 if yaw < 0 else yaw % -180  # Make the compound yaw between -180 and 180 degrees
        roll = 0  # TODO remove zero roll assumption
        rpy = RPY(roll, pitch, yaw)

        return rpy

    def _camera_normal(self) -> Optional[np.ndarray]:
        """Returns camera normal unit vector.

        :return: Camera normal unit vector, or None if not available
        """
        nadir = np.array([0, 0, 1])
        rpy = self._get_camera_rpy()
        if rpy is None:
            self.get_logger().warn('Could not get RPY - cannot compute camera normal.')
            return None
        assert_type(RPY, rpy)

        r = Rotation.from_euler(self.EULER_SEQUENCE, list(rpy), degrees=True)
        camera_normal = r.apply(nadir)
        assert_shape(camera_normal, nadir.shape)

        camera_normal_length = np.linalg.norm(camera_normal)
        # TODO: is this arbitrary check needed?
        if abs(camera_normal_length - 1) >= 0.001:
            self.get_logger().warn(f'Camera normal length: {camera_normal_length}, does not look like a unit vector?.')

        return camera_normal

    def _pitch_index(self) -> int:
        """Returns the pitch index for used euler vectors.

        :return: Pitch index
        """
        return self.EULER_SEQUENCE.lower().find('y')

    def _yaw_index(self) -> int:
        """Returns the yaw index for used euler vectors.

        :return: Yaw index
        """
        return self.EULER_SEQUENCE.lower().find('x')

    def camera_info_callback(self, msg: CameraInfo) -> None:
        """Handles latest camera info message.

        :param msg: CameraInfo message from the PX4-ROS 2 bridge
        :return:
        """
        self.get_logger().debug(f'Camera info received:\n{msg}.')
        self._camera_info = msg
        camera_info_topic = self._topics.get(self.SUBSCRIBE_KEY, {}).get('camera_info', None)
        if camera_info_topic is not None:
            self.get_logger().warn('Assuming camera_info is static - destroying the subscription.')
            camera_info_topic.destroy()

    def vehiclelocalposition_pubsubtopic_callback(self, msg: VehicleLocalPosition) -> None:
        """Handles latest VehicleLocalPosition message.

        :param msg: VehicleLocalPosition from the PX4-ROS 2 bridge
        :return:
        """
        self._vehicle_local_position = msg
        self.get_logger().debug(f'VehicleLocalPosition: {msg}.')

    def _get_dynamic_map_radius(self, altitude: Union[int, float]) -> int:
        """Returns map radius that adjusts for camera altitude.

        :param altitude: Altitude of camera in meters
        :return: Suitable map radius in meters
        """
        assert_type(get_args(Union[int, float]), altitude)
        max_map_radius = self.get_parameter('map_update.max_map_radius').get_parameter_value().integer_value

        camera_info = self._camera_info
        if camera_info is not None:
            assert hasattr(camera_info, 'k')
            assert hasattr(camera_info, 'width')
            w = camera_info.width
            f = camera_info.k[0]
            assert camera_info.k[0] == camera_info.k[4]  # Assert assumption that fx = fy
            hfov = 2 * math.atan(w / (2 * f))
            map_radius = 1.5*hfov*altitude  # Arbitrary padding of 50%
        else:
            # TODO: does this happen? Trying to update map before camera info has been received?
            self.get_logger().warn(f'Could not get camera info, using best guess for map width.')
            map_radius = 3*altitude  # Arbitrary guess

        if map_radius > max_map_radius:
            self.get_logger().warn(f'Dynamic map radius {map_radius} exceeds max map radius {max_map_radius}, using '
                                   f'max_map_radius instead.')
            map_radius = max_map_radius

        return map_radius

    def vehicleglobalposition_pubsubtopic_callback(self, msg: VehicleGlobalPosition) -> None:
        """Handles latest VehicleGlobalPosition message.

        :param msg: VehicleGlobalPosition from the PX4-ROS 2 bridge
        :return:
        """
        self._vehicle_global_position = msg

    def _wms_results_pending(self) -> bool:
        """Checks whether there are pending wms_results.

        :return: True if there are pending results.
        """
        if self._wms_results is not None:
            if not self._wms_results.ready():
                # Previous request still running
                return True

        return False

    def _previous_map_frame_too_close(self, center: Union[LatLon, LatLonAlt], radius: Union[int, float]) -> bool:
        """Checks if previous map frame is too close to new requested one.

        :param center: WGS84 coordinates of new map candidate center
        :param radius: Radius in meters of new map candidate
        :return: True if previous map frame is too close.
        """
        assert_type(get_args(Union[int, float]), radius)
        assert_type(get_args(Union[LatLon, LatLonAlt]), center)
        if self._previous_map_frame is not None:
            if not (abs(self._distance(center, self._previous_map_frame.center)) >
                    self.get_parameter('map_update.update_map_center_threshold').get_parameter_value().integer_value or
                    abs(radius - self._previous_map_frame.radius) >
                    self.get_parameter('map_update.update_map_radius_threshold').get_parameter_value().integer_value):
                return True

        return False

    def _should_update_map(self, center: Union[LatLon, LatLonAlt], radius: Union[int, float]) -> bool:
        """Checks if a new WMS map request should be made to update old map.

        Map is updated unless (1) there is a previous map frame that is close enough to provided center and has radius
        that is close enough to new request, (2) previous WMS request is still processing, or (3) camera pitch is too
        large and gimbal projection is used so that map center would be too far or even beyond the horizon.

        :param center: WGS84 coordinates of new map candidate center
        :param radius: Radius in meters of new map candidate
        :return: True if map should be updated
        """
        assert_type(get_args(Union[int, float]), radius)
        assert_type(get_args(Union[LatLon, LatLonAlt]), center)

        # Check conditions (1) and (2) - previous results pending or requested new map too close to old one
        if self._wms_results_pending() or self._previous_map_frame_too_close(center, radius):
            return False

        # Check condition (3) - whether camera pitch is too large if using gimbal projection
        # TODO: do not even attempt to compute center arg in this case? Would have to be checked earlier?
        use_gimbal_projection = self.get_parameter('map_update.gimbal_projection').get_parameter_value().bool_value
        if use_gimbal_projection:
            max_pitch = self.get_parameter('map_update.max_pitch').get_parameter_value().integer_value
            if self._camera_pitch_too_high(max_pitch):
                self.get_logger().warn(f'Camera pitch not available or above maximum {max_pitch}. Will not update map.')
                return False

        return True

    def gimbaldeviceattitudestatus_pubsubtopic_callback(self, msg: GimbalDeviceAttitudeStatus) -> None:
        """Handles latest GimbalDeviceAttitudeStatus message.

        :param msg: GimbalDeviceAttitudeStatus from the PX4-ROS 2 bridge
        :return:
        """
        self._gimbal_device_attitude_status = msg

    def gimbaldevicesetattitude_pubsubtopic_callback(self, msg: GimbalDeviceSetAttitude) -> None:
        """Handles latest GimbalDeviceSetAttitude message.

        :param msg: GimbalDeviceSetAttitude from the PX4-ROS 2 bridge
        :return:
        """
        """Handles latest GimbalDeviceSetAttitude message."""
        self._gimbal_device_set_attitude = msg

    def vehicleattitude_pubsubtopic_callback(self, msg: VehicleAttitude) -> None:
        """Handles latest VehicleAttitude message.

        :param msg: VehicleAttitude from the PX4-ROS 2 bridge
        :return:
        """
        self._vehicle_attitude = msg

    def _create_vehicle_visual_odometry_msg(self, timestamp: int, position: tuple, rotation: tuple) \
            -> None:
        """Creates a VehicleVisualOdometry and saves it to self._vehicle_visual_odometry.

        See https://docs.px4.io/v1.12/en/advanced_config/tuning_the_ecl_ekf.html#external-vision-system for supported
        EKF2_AID_MASK values when using an external vision system.

        :param timestamp: Timestamp to be included in the outgoing message
        :param position: Position tuple (x, y, z) to be published
        :param rotation: Rotation quaternion to be published
        :return:
        """
        assert VehicleVisualOdometry is not None, 'VehicleVisualOdometry definition not found (was None).'
        msg = VehicleVisualOdometry()

        # Timestamp
        #now = int(time.time() * 1e6)  # uint64 time in microseconds  # TODO: should be time since system start?
        msg.timestamp = timestamp  # now
        msg.timestamp_sample = timestamp  # now  # uint64

        # Position and linear velocity local frame of reference
        msg.local_frame = self.LOCAL_FRAME_NED  # uint8

        # Position
        if position is not None:
            assert len(
                position) == 3, f'Unexpected length for position estimate: {len(position)} (3 expected).'  # TODO: can also be length 2 if altitude is not published, handle that
            assert all(isinstance(x, float) for x in position), f'Position contained non-float elements.'
            msg.x, msg.y, msg.z = position  # float32 North, East, Down
        else:
            self.get_logger().warn('Position tuple was None - publishing NaN as position.')
            msg.x, msg.y, msg.z = (float('nan'),) * 3  # float32 North, East, Down

        # Attitude quaternions
        # Rotation is currently computed with assumed NED frame so it is asserted here just in case
        assert msg.local_frame is self.LOCAL_FRAME_NED, f'Published rotation logic requires that NED frame is used.'
        if rotation is not None:
            msg.q = rotation
            msg.q_offset = (0.0, ) * 4
        else:
            msg.q = (float('nan'),) * 4  # float32
            msg.q_offset = (float('nan'),) * 4
        msg.pose_covariance = (float('nan'),) * 21

        # Velocity frame of reference
        msg.velocity_frame = self.LOCAL_FRAME_NED  # uint8

        # Velocity
        msg.vx, msg.vy, msg.vz = (float('nan'),) * 3  # float32 North, East, Down

        # Angular velocity - not used
        msg.rollspeed, msg.pitchspeed, msg.yawspeed = (float('nan'),) * 3  # float32
        msg.velocity_covariance = (float('nan'),) * 21  # float32 North, East, Down

        self.get_logger().debug(f'Setting outgoing vehicle visual odometry message as:\n{msg}.')
        self._vehicle_visual_odometry = msg

    def _camera_pitch(self) -> Optional[Union[int, float]]:
        """Returns camera pitch in degrees relative to vehicle frame.

        :return: Camera pitch in degrees, or None if not available
        """
        rpy = self._get_camera_rpy()
        if rpy is None:
            self.get_logger().warn('Gimbal RPY not available, cannot compute camera pitch.')
            return None
        assert_type(RPY, rpy)
        return rpy.pitch

    def _gimbal_attitude(self) -> Optional[Union[GimbalDeviceAttitudeStatus, GimbalDeviceSetAttitude]]:
        """Returns 1. GimbalDeviceAttitudeStatus, or 2. GimbalDeviceSetAttitude if 1. is not available.

        :return: GimbalDeviceAttitudeStatus or GimbalDeviceSetAttitude message
        """
        gimbal_attitude = self._gimbal_device_attitude_status
        if gimbal_attitude is None:
            self.get_logger().debug('GimbalDeviceAttitudeStatus not available. Trying GimbalDeviceSetAttitude instead.')
            gimbal_attitude = self._gimbal_device_set_attitude
            if gimbal_attitude is None:
                self.get_logger().debug('GimbalDeviceSetAttitude not available. Gimbal attitude status not available.')
        return gimbal_attitude

    @staticmethod
    def _find_and_decompose_homography(mkp_img: np.ndarray, mkp_map: np.ndarray, k: np.ndarray,
                                       camera_normal: np.ndarray, reproj_threshold: float = 1.0,
                                       restrict_affine: bool = False) -> Tuple[np.ndarray, np.ndarray, np.ndarray,
                                                                               np.ndarray]:
        """Processes matching keypoints from img and map and returns homography matrix, mask, translation and rotation.

        :param mkp_img: Matching keypoints from image
        :param mkp_map: Matching keypoints from map
        :param k: Camera intrinsics matrix
        :param camera_normal: Camera normal unit vector
        :param reproj_threshold: RANSAC reprojection threshold parameter
        :param restrict_affine: Flag indicating whether homography should be restricted to 2D affine transformation
        :return: Tuple containing homography matrix, RANSAC inlier mask, translation and rotation
        """
        min_points = 4  # TODO: use self.MINIMIUM_MATCHES?
        assert_type(np.ndarray, mkp_img)
        assert_type(np.ndarray, mkp_map)
        assert len(mkp_img) >= min_points and len(mkp_map) >= min_points, 'Four points needed to estimate homography.'

        assert_type(bool, restrict_affine)
        assert_type(float, reproj_threshold)
        if not restrict_affine:
            h, h_mask = cv2.findHomography(mkp_img, mkp_map, cv2.RANSAC, reproj_threshold)
        else:
            h, h_mask = cv2.estimateAffinePartial2D(mkp_img, mkp_map)
            h = np.vstack((h, np.array([0, 0, 1])))  # Make it into a homography matrix

        assert_type(np.ndarray, k)
        assert_shape(k, (3, 3))
        num, Rs, Ts, Ns = cv2.decomposeHomographyMat(h, k)

        # Get the one where angle between plane normal and inverse of camera normal is smallest
        # Plane is defined by Z=0 and "up" is in the negative direction on the z-axis in this case
        get_angle_partial = partial(get_angle, -camera_normal)
        angles = list(map(get_angle_partial, Ns))
        index_of_smallest_angle = angles.index(min(angles))
        rotation, translation = Rs[index_of_smallest_angle], Ts[index_of_smallest_angle]

        return h, h_mask, translation, rotation

    def _match_inputs(self, image_frame: ImageFrame) -> dict:
        """Returns a dictionary snapshot of the input data required to perform and process a match.

        Processing of matches is asynchronous, so this method provides a way of taking a snapshot of the input arguments
        to :meth:`_process_matches` from the the time image used for the matching was taken.

        The dictionary has the following data:
            map_frame - np.ndarray map_frame to match
            camera_info - CameraInfo
            camera_normal - np.ndarray Camera normal unit vector
            camera_yaw - float
            camera_pitch - float
            map_dim_with_padding - Dim map dimensions including padding for rotation
            img_dim - Dim image dimensions
            restrict_affine - bool flag indicating whether homography matrix should be restricted to 2D affine tform
            local_frame_origin_position - LatLonAlt origin of local frame global frame WGS84
            timestamp - Local position message timestamp (to sync vehicle visual odom messages)
            map_cropped - np.ndarray Rotated and cropped map raster from map_frame.image

        :param image_frame: The image frame from the drone video
        :return: Dictionary with matching input data (give as **kwargs to _process_matches)
        """
        data = {
            'image_frame': image_frame,
            'map_frame': self._map_frame,
            # Camera information
            'camera_info': self._camera_info,
            'camera_normal': self._camera_normal(),
            'camera_yaw': self._camera_yaw(),
            'camera_pitch': self._camera_pitch(),
            # Image and map raster dimensions
            'map_dim_with_padding': self._map_dim_with_padding(),
            'img_dim': self._img_dim(),
            # Should homography be restricted to 2D affine transformation
            'restrict_affine': self._restrict_affine(),
            # Vehicle local frame global reference position
            'local_frame_origin_position': (self._vehicle_local_position_ref_latlonalt_timestamp())[0],
            'timestamp': (self._vehicle_local_position_ref_latlonalt_timestamp())[1],
        }

        # Get cropped and rotated map
        camera_yaw = data.get('camera_yaw', None)
        map_frame = data.get('map_frame', None)
        img_dim = data.get('img_dim', None)
        if all((camera_yaw, map_frame, img_dim)):
            assert hasattr(map_frame, 'image'), 'Map frame unexpectedly did not contain the image data.'
            assert -180 <= camera_yaw <= 180, f'Unexpected gimbal yaw value: {camera_yaw} ([-180, 180] expected).'
            camera_yaw = math.radians(camera_yaw)
            data['map_cropped'] = rotate_and_crop_map(map_frame.image, camera_yaw, img_dim)
        else:
            data['map_cropped'] = None

        return data

    def _compute_camera_position(self, t: np.ndarray, center: LatLon, scaling: float) -> LatLon:
        """Returns camera position based on translation vector and map raster center coordinates.

        NOTE:
        The map center coordinates are the coordinates of the center of map_cropped. The cropping and rotation of
        map_cropped is implemented so that its center coordinates match the uncropped, unrotated map's center
        coordinates. Input arg center can therefore be given as the center of the original map's bounding box as long
        as the cropping and rotation implementation remains unchanged.

        :param t: Camera translation vector
        :param center: Map center WGS84 coordinates
        :param scaling: Scaling factor for translation vector (i.e. positive altitude in meters)
        :return: WGS84 coordinates of camera
        """
        assert_type(np.ndarray, t)
        assert_shape(t, (2,))
        azmth = self._get_azimuth(t[0], t[1])
        dist = math.sqrt(t[0] ** 2 + t[1] ** 2)
        scaled_dist = scaling*dist
        assert scaling > 0
        position = self._move_distance(center, (-azmth, scaled_dist))  # Invert azimuth, going the other way
        return position

    def _compute_camera_distance(self, fov_wgs84: np.ndarray, focal_length: float, img_dim: Dim) -> float:
        """Computes camera distance from projected principal point in meters.

        :param fov_wgs84: Field of view corners in WGS84 coordinates
        :param focal_length: Camera focal length
        :param img_dim: Image dimensions
        :return: Distance to principal point on surface in meters
        """
        assert_type(np.ndarray, fov_wgs84)
        assert_type(float, focal_length)
        assert_type(Dim, img_dim)
        fov_center_line_length = self._get_distance_of_fov_center(fov_wgs84)
        camera_distance = fov_center_line_length * focal_length / img_dim.width
        assert_type(float, camera_distance)
        return camera_distance

    @staticmethod
    def _compute_camera_altitude(camera_distance: float, camera_pitch: Union[int, float]) -> float:
        """Computes camera altitude based on distance to principal point and pitch in degrees.

        :param camera_distance: Camera distance to projected principal point
        :param camera_pitch: Camera pitch in degrees
        :return:
        """
        # TODO: use rotation from decomposeHomography for getting the pitch in this case (use visual info, not from sensors)
        camera_altitude = math.cos(math.radians(camera_pitch)) * camera_distance
        return camera_altitude

    def _camera_pitch_too_high(self, max_pitch: Union[int, float]) -> bool:
        """Returns True if camera pitch exceeds given limit.

        :param max_pitch: The limit for the pitch over which it will be considered too high
        :return: True if pitch is too high
        """
        assert_type(get_args(Union[int, float]), max_pitch)
        camera_pitch = self._camera_pitch()
        if camera_pitch is not None:
            if abs(camera_pitch) > max_pitch:
                self.get_logger().debug(f'Camera pitch {camera_pitch} is above limit {max_pitch}.')
                return True
            if camera_pitch < 0:
                self.get_logger().warn(f'Camera pitch {camera_pitch} is negative.')
        else:
            self.get_logger().warn(f'Could not determine camera pitch.')
            return True

        return False

    def _should_match(self):
        """Determines whether _match should be called based on whether previous match is still being processed.

        Match should be attempted if (1) there are no pending match results, and (2) camera pitch is not too high (e.g.
        facing horizon instead of nadir).

        :return:
        """
        # Check condition (1) - that a request is not already running
        if not (self._superglue_results is None or self._superglue_results.ready()):  # TODO: handle timeouts, failures for _superglue_results
            return False

        # Check condition (2) - whether camera pitch is too large
        max_pitch = self.get_parameter('misc.max_pitch').get_parameter_value().integer_value
        if self._camera_pitch_too_high(max_pitch):
            self.get_logger().warn(f'Camera pitch is not available or above limit {max_pitch}. Skipping matching.')
            return False

        return True

    def superglue_worker_error_callback(self, e: BaseException) -> None:
        """Error callback for SuperGlue worker.

        :return:
        """
        self.get_logger().error(f'SuperGlue process returned and error:\n{e}\n{traceback.print_exc()}')

    def superglue_worker_callback(self, results) -> None:
        """Callback for SuperGlue worker.

        :return:
        """
        mkp_img, mkp_map = results[0]
        assert_len(mkp_img, len(mkp_map))

        visualize_homography_ = self.get_parameter('misc.visualize_homography').get_parameter_value().bool_value
        if not visualize_homography_:
            # _process_matches will not visualize homography if map_cropped is None
            self._stored_inputs['map_cropped'] = None
        self._process_matches(mkp_img, mkp_map, **self._stored_inputs)

    def _compute_local_frame_origin(self, position: Union[LatLon, LatLonAlt]):
        """Computes local frame global coordinates from local frame coordinates and their known global coordinates.

        VehicleLocalPosition does not include the reference coordinates for the local frame if e.g. GPS is turned off.
        This function computes the reference coordinates by translating the inverse of current local coordinates from
        current estimated global position.

        :param position: WGS84 coordinates of current vehicle local position
        :return:
        """
        assert self._vehicle_local_position is not None
        assert np.isnan(self._vehicle_local_position.ref_lat), \
            f'_vehicle_local_position.ref_lat was {self._vehicle_local_position.ref_lat}. You should not try to ' \
            f'compute the local origin unless it is not provided in VehicleLocalPosition'
        assert np.isnan(self._vehicle_local_position.ref_lon), \
            f'_vehicle_local_position.ref_lon was {self._vehicle_local_position.ref_lon}. You should not try to ' \
            f'compute the local origin unless it is not provided in VehicleLocalPosition'
        assert self._local_origin is None, f'self._local_origin was {self.__local_origin}. You should not try to ' \
                                           f'recompute the local origin if it has already been done.'
        x, y = -self._vehicle_local_position.x, -self._vehicle_local_position.y
        assert_type(float, x)
        assert_type(float, y)
        azmth = self._get_azimuth(x, y)
        dist = math.sqrt(x ** 2 + y ** 2)
        local_origin = LatLonAlt(*(self._move_distance(position, (azmth, dist)) + (0,)))
        self.get_logger().info(f'Local frame origin set at {local_origin}, this should happen only once.')
        return local_origin

    def _compute_local_frame_position(self, position: LatLonAlt, origin: LatLonAlt) -> Tuple[float, float, float]:
        """Computes position of WGS84 coordinates in local frame coordinates

        :param origin: WGS84 coordiantes of local frame origin
        :param position: WGS84 coordinates and altitude in meters of estimated camera position
        :return: Tuple containing x, y and z coordinates (meters) in local frame
        """
        assert_type(LatLonAlt, position)
        assert_type(LatLonAlt, origin)

        lats_orig = (origin.lat, origin.lat)
        lons_orig = (origin.lon, origin.lon)
        lats_term = (origin.lat, position.lat)
        lons_term = (position.lon, origin.lon)
        _, __, dist = self._geod.inv(lons_orig, lats_orig, lons_term, lats_term)

        lat_diff = math.copysign(dist[1], position.lat - origin.lat)
        lon_diff = math.copysign(dist[0], position.lon - origin.lon)

        alt = position.alt - origin.alt
        assert alt >= 0

        return lat_diff, lon_diff, -alt

    def _compute_attitude_quaternion(self, ll: np.ndarray, lr: np.ndarray) -> Optional[Tuple[float]]:
        """Computes attitude quaternion against NED frame for outgoing VehicleVisualOdometry message.

        Attitude estimate adjusts vehicle heading (used to rotate the map raster together with gimbal yaw) by the angle
        of the estimated field of view bottom side to the x-axis vector. This adjustment angle is expected to be very
        small in most cases as the FOV should align well with the rotated map raster.

        :param ll: Lower left corner pixel coordinates of estimated field of view
        :param lr: Lower right corner pixel coordinates of estimated field of view
        :return: Quaternion tuple, or None if attitude information is not available
        """
        # TODO: Seems like when gimbal (camera) is nadir facing, PX4 automatically rotates it back to face vehicle
        #  heading. However, GimbalDeviceSetAttitude stays the same so this will give an estimate for vehicle heading
        #  that is off by camera_yaw in this case. No problems with nadir-facing camera only if there is no gimbal yaw.
        #  Need to log a warning or even an error in these cases!
        assert_type(np.ndarray, ll)
        assert_type(np.ndarray, lr)
        if self._vehicle_attitude is not None:
            fov_adjustment_angle = math.degrees(get_angle(np.float32([1, 0]), (lr - ll).squeeze(), normalize=True))
            assert hasattr(self._vehicle_attitude, 'q')
            euler = Rotation.from_quat(self._vehicle_attitude.q).as_euler(self.EULER_SEQUENCE, degrees=True)
            vehicle_yaw = euler[self._yaw_index()] + fov_adjustment_angle
            euler[self._yaw_index()] = vehicle_yaw
            quaternion = tuple(Rotation.from_euler(self.EULER_SEQUENCE, euler, degrees=True).as_quat())
            assert_len(quaternion, 4)
            self.get_logger().debug(f'Heading adjustment angle: {fov_adjustment_angle}.')
            self.get_logger().debug(f'Vehicle yaw: {vehicle_yaw}.')
            return quaternion
        else:
            # TODO: when does this happen? When do we not have this info? Should have it always, at least pitch and roll?
            return None

    def _process_matches(self, mkp_img: np.ndarray, mkp_map: np.ndarray, image_frame: ImageFrame, map_frame: MapFrame,
                         camera_info: CameraInfo, camera_normal: np.ndarray, camera_yaw: float, camera_pitch: float,
                         map_dim_with_padding: Dim, img_dim: Dim, restrict_affine: bool,
                         local_frame_origin_position: Optional[LatLonAlt], timestamp: Optional[int],
                         map_cropped: Optional[np.ndarray] = None) -> None:
        """Process the matching image and map keypoints into an outgoing VehicleVisualOdometry message.

        :param mkp_img: Matching keypoints in drone image
        :param mkp_map: Matching keypoints in map raster
        :param image_frame: The drone image
        :param map_frame: The map raster
        :param camera_info: CameraInfo from time of match (from _match_inputs)
        :param camera_normal: Camera normal unit vector from time of match (from _match_inputs)
        :param camera_yaw: Camera yaw in degrees from time of match (from _match_inputs)
        :param camera_pitch: Camera pitch in degrees from time of match (from _match_inputs)
        :param map_dim_with_padding: Map dimensions with padding from time of match (from _match_inputs)
        :param img_dim: Drone image dimensions from time of match (from _match_inputs)
        :param restrict_affine: Restrict affine flag from time of match (from _match_inputs)
        :param local_frame_origin_position: Local frame origin coordinates from time of match (from _match_inputs)
        :param timestamp: Local position message timestamp from time of match (from _match_inputs)
        :param map_cropped: Optional map cropped image, visualizes matches if provided

        :return:
        """
        if len(mkp_img) < self.MINIMUM_MATCHES:
            self.get_logger().warn(f'Found {len(mkp_img)} matches, {self.MINIMUM_MATCHES} required. Skip frame.')
            return None

        # Find and decompose homography matrix, do some sanity checks
        k = camera_info.k.reshape([3, 3])
        h, h_mask, t, r = self._find_and_decompose_homography(mkp_img, mkp_map, k, camera_normal,
                                                              restrict_affine=restrict_affine)
        assert_shape(h, (3, 3))
        assert_shape(t, (3, 1))
        assert_shape(r, (3, 3))

        # This block 1. computes fov in WGS84 and attaches it to image_frame, and 3. visualizes homography
        # Convert pixel field of view into WGS84 coordinates, save it to the image frame, visualize the pixels
        fov_pix = get_fov(image_frame.image, h)
        if map_cropped is not None:
            visualize_homography('Matches and FoV', image_frame.image, map_cropped, mkp_img, mkp_map, fov_pix)
        fov_wgs84, fov_uncropped, fov_unrotated = convert_from_pix_to_wgs84(
            fov_pix, map_dim_with_padding, map_frame.bbox, camera_yaw, img_dim)
        image_frame.fov = fov_wgs84

        # Compute camera altitude, and distance to principal point using triangle similarity
        # TODO: _update_map or _project_gimbal_fov_center has similar logic used in gimbal fov projection, try to combine
        assert k[0][0] == k[1][1]
        camera_distance = self._compute_camera_distance(fov_wgs84, k[0][0], img_dim)
        camera_altitude = self._compute_camera_altitude(camera_distance, camera_pitch)
        self.get_logger().debug(f'Computed camera distance {camera_distance}, altitude {camera_altitude}.')

        t_rotated = rotate_point(math.radians(camera_yaw), Dim(0, 0), t)  # TODO: do checks on inputs that they are not none # Ugly API call, rotate around 0,0
        t_rotated = np.array((t_rotated[0][0], t_rotated[1][0])) # TODO: clean up rotate_point API!
        map_center = get_bbox_center(map_frame.bbox)
        position = self._compute_camera_position(t_rotated, map_center, camera_altitude)  # TODO: calls convert_fov_from_pix_to_wgs84 which is also called above, remove redundant use
        image_frame.position = LatLonAlt(position.lat, position.lon, camera_altitude)

        assert_type(LatLonAlt, image_frame.position)
        assert all(image_frame.position)

        # If no GPS, local frame origin must be computed or retrieved from memory if it was computed earlier
        if local_frame_origin_position is None:
            if self._local_origin is None:
                self._local_origin = self._compute_local_frame_origin(image_frame.position)
            local_frame_origin_position = self._local_origin

        local_position = self._compute_local_frame_position(image_frame.position, local_frame_origin_position)
        self.get_logger().debug(f'Local frame position: {local_position}, origin ref position '
                                f'{local_frame_origin_position}.')

        assert camera_altitude is not None

        quaternion = self._compute_attitude_quaternion(fov_pix[1], fov_pix[2])  # TODO: do not use hard-coded indices, prone to breaking

        if timestamp is None:
            # Initialize timestamp for vvo
            self.get_logger().debug('No ekf2 timestamp provided, using current time as timestamp.')
            timestamp = int(time.time_ns() / 1000)  # time "since system start" in microseconds
        self._create_vehicle_visual_odometry_msg(timestamp, local_position, quaternion)

        export_geojson = self.get_parameter('misc.export_position').get_parameter_value().string_value
        if export_geojson is not None:
            self._export_position(image_frame.position, image_frame.fov, export_geojson)

    def _export_position(self, position: Union[LatLon, LatLonAlt], fov: np.ndarray, filename: str) -> None:
        """Exports the computed position and field of view (FOV) into a geojson file.

        :param position: Computed camera position
        :param: fov: Field of view of camera
        :param filename: Name of file to write into
        :return:
        """
        assert_type(get_args(Union[LatLon, LatLonAlt]), position)
        assert_type(np.ndarray, fov)
        assert_type(str, filename)
        point = Feature(geometry=Point((position.lon, position.lat)))  # TODO: add name/description properties
        corners = np.flip(fov.squeeze()).tolist()
        corners = [tuple(x) for x in corners]
        corners = Feature(geometry=Polygon([corners]))  # TODO: add name/description properties
        features = [point, corners]
        feature_collection = FeatureCollection(features)
        try:
            with open(filename, 'w') as f:
                dump(feature_collection, f)
        except Exception as e:
            self.get_logger().error(f'Could not write file {filename} because of exception:'
                                    f'\n{e}\n{traceback.print_exc()}')

    def _match(self, image_frame: ImageFrame, map_cropped: np.ndarray) -> None:
        """Matches camera image to map image and computes camera position and field of view.

        :param image_frame: The image frame to match
        :param map_cropped: Cropped and rotated map raster (aligned with image)
        :return:
        """
        assert self._superglue_results is None or self._superglue_results.ready()
        self._superglue_results = self._superglue_pool.starmap_async(
            self._superglue_pool_worker,
            [(image_frame.image, map_cropped)],
            callback=self.superglue_worker_callback,
            error_callback=self.superglue_worker_error_callback
        )

    def terminate_wms_pool(self):
        """Terminates the WMS Pool.

        :return:
        """
        if self._wms_pool is not None:
            self.get_logger().info('Terminating WMS pool.')
            self._wms_pool.terminate()

    def destroy_timers(self):
        """Destroys the vehicle visual odometry publish and map update timers.

        :return:
        """
        if self._publish_timer is not None:
            self.get_logger().info('Destroying publish timer.')
            assert_type(rclpy.timer.Timer, self._publish_timer)
            self._publish_timer.destroy()

        if self._publish_timer is not None:
            self.get_logger().info('Destroying map update timer.')
            assert_type(rclpy.timer.Timer, self._map_update_timer)
            self._map_update_timer.destroy()


def main(args=None):
    """Starts and terminates the ROS 2 node.

    Also starts cProfile profiling in debugging mode.

    :param args: Any args for initializing the rclpy node
    :return:
    """
    if __debug__:
        pr = cProfile.Profile()
        pr.enable()
    else:
        pr = None
    try:
        rclpy.init(args=args)
        matcher = MapNavNode('map_nav_node', share_dir, superglue_dir)
        rclpy.spin(matcher)
    except KeyboardInterrupt as e:
        print(f'Keyboard interrupt received:\n{e}')
        if pr is not None:
            # Print out profiling stats
            pr.disable()
            s = io.StringIO()
            ps = pstats.Stats(pr, stream=s).sort_stats(pstats.SortKey.CUMULATIVE)
            ps.print_stats()
            print(s.getvalue())
    finally:
        matcher.destroy_timers()
        matcher.terminate_wms_pool()
        matcher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
