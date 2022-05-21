"""Module that contains the MapNavNode ROS 2 node."""
import sys
import rclpy
import traceback
import math
import numpy as np
import cv2
import time
import importlib
import os
import yaml
import copy

from ament_index_python.packages import get_package_share_directory
PACKAGE_NAME = 'python_px4_ros2_map_nav'  # TODO: try to read from somewhere (e.g. package.xml)

# Import and configure torch for multiprocessing
import torch
try:
    torch.multiprocessing.set_start_method('spawn', force=True)
except RuntimeError:
    pass
torch.set_num_threads(1)

from abc import ABC, abstractmethod
from multiprocessing.pool import Pool, AsyncResult  # Used for WMS client process, not for torch
from pyproj import Geod
from typing import Optional, Union, Tuple, get_args, List
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from geojson import Point, Polygon, Feature, FeatureCollection, dump
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation
from functools import partial
from px4_msgs.msg import VehicleLocalPosition, VehicleGlobalPosition, GimbalDeviceAttitudeStatus, \
    GimbalDeviceSetAttitude, VehicleGpsPosition
from sensor_msgs.msg import CameraInfo, Image


from python_px4_ros2_map_nav.data import BBox, Dim, LatLon, TimePair, RPY, LatLonAlt, ImageData, MapData, Match,\
    InputData, OutputData, ImagePair, AsyncQuery, ContextualMapData, FixedCamera, FOV, Img, Pose
from python_px4_ros2_map_nav.transform import get_fov_and_c, \
    inv_homography_from_k_and_e, get_azimuth, make_keypoint, is_convex_isosceles_trapezoid, \
    relative_area_of_intersection
from python_px4_ros2_map_nav.assertions import assert_type, assert_ndim, assert_len, assert_shape
from python_px4_ros2_map_nav.ros_param_defaults import Defaults
from python_px4_ros2_map_nav.matchers.matcher import Matcher
from python_px4_ros2_map_nav.matchers.orb import ORBMatcher
from python_px4_ros2_map_nav.wms import WMSClient
from python_px4_ros2_map_nav.visualization import Visualization


class MapNavNode(Node, ABC):
    """ROS 2 Node that publishes position estimate based on visual match of drone video to map of same location."""
    # Encoding of input video (input to CvBridge)
    IMAGE_ENCODING = 'bgr8'  # E.g. gscam2 only supports bgr8 so this is used to override encoding in image header

    # Ellipsoid model used by pyproj
    PYPROJ_ELLIPSOID = 'WGS84'

    # Keys for topics dictionary that map microRTPS bridge topics to subscribers and message definitions
    TOPICS_MSG_KEY = 'message'
    TOPICS_SUBSCRIBER_KEY = 'subscriber'

    # Process counts for multiprocessing pools
    WMS_PROCESS_COUNT = 1  # should be 1
    MATCHER_PROCESS_COUNT = 1  # should be 1, same for both map and vo matching pools

    def __init__(self, node_name: str) -> None:
        """Initializes the ROS 2 node.

        :param node_name: Name of the node
        """
        assert_type(node_name, str)
        super().__init__(node_name)
        # TODO: try this if loading param values from YAML file does not work
        #super().__init__(node_name, allow_undeclared_parameters=True,
        #                 automatically_declare_parameters_from_overrides=True)
        self.name = node_name

        # Setup config and declare ROS parameters
        self.__declare_ros_params()

        # WMS client and requests in a separate process
        self._wms_results = None  # Must check for None when using this
        url = self.get_parameter('wms.url').get_parameter_value().string_value
        version = self.get_parameter('wms.version').get_parameter_value().string_value
        timeout = self.get_parameter('wms.request_timeout').get_parameter_value().integer_value
        assert_type(url, str)
        assert_type(version, str)
        assert_type(timeout, int)
        self._wms_pool = Pool(self.WMS_PROCESS_COUNT, initializer=WMSClient.initializer,
                              initargs=(url, version, timeout))

        # Setup map update timer
        self._map_update_timer = self._setup_map_update_timer()

        # Dict for storing all microRTPS bridge subscribers
        self._topics = {
            'VehicleLocalPosition_PubSubTopic': {self.TOPICS_MSG_KEY: VehicleLocalPosition},
            'VehicleGlobalPosition_PubSubTopic': {self.TOPICS_MSG_KEY: VehicleGlobalPosition},
            'GimbalDeviceAttitudeStatus_PubSubTopic': {self.TOPICS_MSG_KEY: GimbalDeviceAttitudeStatus},
            'GimbalDeviceSetAttitude_PubSubTopic': {self.TOPICS_MSG_KEY: GimbalDeviceSetAttitude},
            'camera_info': {self.TOPICS_MSG_KEY: CameraInfo},
            'image_raw': {self.TOPICS_MSG_KEY: Image},
        }
        self._setup_subscribers()

        # Converts image_raw to cv2 compatible image
        self._cv_bridge = CvBridge()

        # Setup map matching pool
        map_matcher_params_file = self.get_parameter('map_matcher.params_file').get_parameter_value().string_value
        self._map_matcher, self._map_matching_pool = self._setup_matching_pool(map_matcher_params_file)
        self._map_matching_query = None  # Must check for None when using this

        # Setup visual odometry matching pool
        vo_enabled = self.get_parameter('misc.visual_odometry').get_parameter_value().bool_value
        self._vo_matching_query = None
        if vo_enabled:
            vo_matcher_params_file = self.get_parameter('vo_matcher.params_file').get_parameter_value().string_value
            self._vo_matcher, self._vo_matching_pool = self._setup_matching_pool(vo_matcher_params_file)
        else:
            self._vo_matcher = None
            self._vo_matching_pool = None

        # Used for pyproj transformations
        self._geod = Geod(ellps=self.PYPROJ_ELLIPSOID)

        # Stored blur values for blur detection
        self._blurs = None

        self._visualization = Visualization('Keypoint matches and homography') if __debug__ else None

        # Must check for None when using these
        self._vo_input_data = None
        self._vo_input_data_prev = None
        self._vo_output_data_prev = None
        self._vo_output_data_fix = None
        self._map_input_data = None
        self._map_input_data_prev = None
        self._map_output_data_prev = None

        # self._image_data = None  # Not currently used / needed
        self._map_data = None

        # Windowed estimates for computing estimate SD and variance
        self._map_estimation_history = None
        self._vo_estimation_history = None

        # Stored solution for the PnP problem (map matching and visual odometry separately)
        self._pose_map_guess = None
        self._pose_vo_guess = None

        self._time_sync = None  # For storing local and foreign (EKF2) timestamps

        # Properties that are mapped to microRTPS bridge topics, must check for None when using them
        self._camera_info = None
        self._vehicle_local_position = None
        self._vehicle_global_position = None
        self._gimbal_device_attitude_status = None
        self._gimbal_device_set_attitude = None

    #region Properties
    @property
    def name(self) -> str:
        """Node name."""
        return self._name

    @name.setter
    def name(self, value: str) -> None:
        assert_type(value, str)
        self._name = value

    @property
    def _visualization(self) -> str:
        """Visualization of keypoint matches for debugging"""
        return self.__visualization

    @_visualization.setter
    def _visualization(self, value: Visualization) -> None:
        assert_type(value, Visualization)
        self.__visualization = value

    @property
    def _blurs(self) -> Optional[np.ndarray]:
        """Array of image blur values for filtering images based on blur."""
        return self.__blurs

    @_blurs.setter
    def _blurs(self, value: Optional[np.ndarray]) -> None:
        assert_type(value, get_args(Optional[np.ndarray]))
        self.__blurs = value

    @property
    def _pose_map_guess(self) -> Optional[Pose]:
        """Solution to the PnP problem in :meth:`~_process_matches` for map matching."""
        return self.__pose_map_guess

    @_pose_map_guess.setter
    def _pose_map_guess(self, value: Optional[Pose]) -> None:
        assert_type(value, get_args(Optional[Pose]))
        self.__pose_map_guess = value

    @property
    def _pose_vo_guess(self) -> Optional[Pose]:
        """Solution to the PnP problem in :meth:`~_process_matches` for visual odometry."""
        return self.__pose_vo_guess

    @_pose_vo_guess.setter
    def _pose_vo_guess(self, value: Optional[Pose]) -> None:
        assert_type(value, get_args(Optional[Pose]))
        self.__pose_vo_guess = value

    @property
    def _map_matcher(self) -> Matcher:
        """Dynamically loaded map matcher"""
        return self.__map_matcher

    @_map_matcher.setter
    def _map_matcher(self, value: Matcher) -> None:
        #assert_type(value, Matcher)  # TODO: fix this
        self.__map_matcher = value

    @property
    def _vo_matcher(self) -> Matcher:
        """Dynamically loaded visual odometry matcher"""
        return self.__vo_matcher

    @_vo_matcher.setter
    def _vo_matcher(self, value: Matcher) -> None:
        #assert_type(value, Matcher)  # TODO: fix this
        self.__vo_matcher = value

    @property
    def _time_sync(self) -> Optional[TimePair]:
        """A :class:`python_px4_ros2_map_nav.data.TimePair` with local and foreign (EKF2) timestamps in microseconds

        The pair will contain the local system time and the EKF2 time received via the PX4-ROS 2 bridge. The pair can
        then at any time be used to locally estimate the EKF2 system time.
        """
        return self.__time_sync

    @_time_sync.setter
    def _time_sync(self, value: Optional[TimePair]) -> None:
        assert_type(value, get_args(Optional[TimePair]))
        self.__time_sync = value

    @property
    def _wms_pool(self) -> Pool:
        """Web Map Service client for fetching map rasters."""
        return self.__wms_pool

    @_wms_pool.setter
    def _wms_pool(self, value: Pool) -> None:
        assert_type(value, Pool)
        self.__wms_pool = value

    @property
    def _wms_results(self) -> Optional[AsyncResult]:
        """Asynchronous results from a WMS client request."""
        return self.__wms_results

    @_wms_results.setter
    def _wms_results(self, value: Optional[AsyncResult]) -> None:
        assert_type(value, get_args(Optional[AsyncResult]))
        self.__wms_results = value

    @property
    def _map_update_timer(self) -> rclpy.timer.Timer:
        """Timer for throttling map update WMS requests."""
        return self.__map_update_timer

    @_map_update_timer.setter
    def _map_update_timer(self, value: rclpy.timer.Timer) -> None:
        assert_type(value, rclpy.timer.Timer)
        self.__map_update_timer = value

    @property
    def _map_matching_pool(self) -> torch.multiprocessing.Pool:
        """Pool for running a :class:`~keypoint_matcher.KeypointMatcher` in dedicated process"""
        return self.__map_matching_pool

    @_map_matching_pool.setter
    def _map_matching_pool(self, value: torch.multiprocessing.Pool) -> None:
        # TODO assert type
        #assert_type(torch.multiprocessing.Pool, value)
        self.__map_matching_pool = value

    @property
    def _vo_matching_pool(self) -> Optional[Pool]:
        """Pool for running a :class:`~keypoint_matcher.ORBMatcher` in dedicated process

        None if visual odometry is not enabled.
        """
        return self.__vo_matching_pool

    @_vo_matching_pool.setter
    def _vo_matching_pool(self, value: Optional[Pool]) -> None:
        assert_type(value, get_args(Optional[Pool]))
        self.__vo_matching_pool = value

    @ property
    def _map_data(self) -> Optional[MapData]:
        """The map raster from the WMS server response along with supporting metadata."""
        return self.__map_data

    @_map_data.setter
    def _map_data(self, value: Optional[MapData]) -> None:
        assert_type(value, get_args(Optional[MapData]))
        self.__map_data = value

    @property
    def _map_input_data(self) -> InputData:
        """Inputs stored at time of launching a new asynchronous match that are needed for processing its results."""
        return self.__map_input_data

    @_map_input_data.setter
    def _map_input_data(self, value: Optional[InputData]) -> None:
        assert_type(value, get_args(Optional[InputData]))
        self.__map_input_data = value

    @property
    def _vo_input_data(self) -> InputData:
        """Inputs stored at time of launching a new asynchronous match that are needed for processing its results.

        This is used for visual odometry matches as opposed to :py:attr:`~stored_inputs` which is used for map matches.
        """
        return self.__vo_input_data

    @_vo_input_data.setter
    def _vo_input_data(self, value: Optional[InputData]) -> None:
        assert_type(value, get_args(Optional[InputData]))
        self.__vo_input_data = value

    @property
    def _map_matching_query(self) -> Optional[AsyncQuery]:
        """Asynchronous results and input from a matching process."""
        return self.__map_matching_query

    @_map_matching_query.setter
    def _map_matching_query(self, value: Optional[AsyncQuery]) -> None:
        assert_type(value, get_args(Optional[AsyncQuery]))
        self.__map_matching_query = value

    @property
    def _vo_matching_query(self) -> Optional[AsyncQuery]:
        """Asynchronous results and input from a visual odometry matching process."""
        return self.__vo_matching_query

    @_vo_matching_query.setter
    def _vo_matching_query(self, value: Optional[AsyncQuery]) -> None:
        assert_type(value, get_args(Optional[AsyncQuery]))
        self.__vo_matching_query = value

    @property
    def _map_estimation_history(self) -> Optional[np.ndarray]:
        """Rolling window of past map position estimates for computing estimate variance"""
        return self.__map_estimation_history

    @_map_estimation_history.setter
    def _map_estimation_history(self, value: Optional[np.ndarray]) -> None:
        assert_type(value, get_args(Optional[np.ndarray]))
        self.__map_estimation_history = value

    @property
    def _vo_estimation_history(self) -> Optional[np.ndarray]:
        """Rolling window of past vo position estimates for computing estimate variance"""
        return self.__vo_estimation_history

    @_vo_estimation_history.setter
    def _vo_estimation_history(self, value: Optional[np.ndarray]) -> None:
        assert_type(value, get_args(Optional[np.ndarray]))
        self.__vo_estimation_history = value

    @property
    def _topics(self) -> dict:
        """Dictionary that stores all rclpy publishers and subscribers."""
        return self.__topics

    @_topics.setter
    def _topics(self, value: dict) -> None:
        assert_type(value, dict)
        self.__topics = value

    @property
    def _geod(self) -> Geod:
        """Stored pyproj Geod instance for performing geodetic computations."""
        return self.__geod

    @_geod.setter
    def _geod(self, value: Geod) -> None:
        assert_type(value, Geod)
        self.__geod = value

    @property
    def _cv_bridge(self) -> CvBridge:
        """CvBridge that decodes incoming PX4-ROS 2 bridge images to cv2 images."""
        return self.__cv_bridge

    @_cv_bridge.setter
    def _cv_bridge(self, value: CvBridge) -> None:
        assert_type(value, CvBridge)
        self.__cv_bridge = value

    @property
    def _map_input_data_prev(self) -> Optional[InputData]:
        """Previous map input data"""
        return self.__map_input_data_prev

    @_map_input_data_prev.setter
    def _map_input_data_prev(self, value: Optional[InputData]) -> None:
        assert_type(value, get_args(Optional[InputData]))
        self.__map_input_data_prev = value

    @property
    def _vo_input_data_prev(self) -> Optional[InputData]:
        """Previous visual odometry input data"""
        return self.__vo_input_data_prev

    @_vo_input_data_prev.setter
    def _vo_input_data_prev(self, value: Optional[InputData]) -> None:
        assert_type(value, get_args(Optional[InputData]))
        self.__vo_input_data_prev = value

    @property
    def _camera_info(self) -> Optional[CameraInfo]:
        """CameraInfo received via the PX4-ROS 2 bridge."""
        return self.__camera_info

    @_camera_info.setter
    def _camera_info(self, value: Optional[CameraInfo]) -> None:
        assert_type(value, get_args(Optional[CameraInfo]))
        self.__camera_info = value

    @property
    def _vehicle_local_position(self) -> Optional[VehicleLocalPosition]:
        """VehicleLocalPosition received via the PX4-ROS 2 bridge."""
        return self.__vehicle_local_position

    @_vehicle_local_position.setter
    def _vehicle_local_position(self, value: Optional[VehicleLocalPosition]) -> None:
        assert_type(value, get_args(Optional[VehicleLocalPosition]))
        self.__vehicle_local_position = value

    @property
    def _vehicle_global_position(self) -> Optional[VehicleGlobalPosition]:
        """VehicleGlobalPosition received via the PX4-ROS 2 bridge."""
        return self.__vehicle_global_position

    @_vehicle_global_position.setter
    def _vehicle_global_position(self, value: Optional[VehicleGlobalPosition]) -> None:
        assert_type(value, get_args(Optional[VehicleGlobalPosition]))
        self.__vehicle_global_position = value

    @property
    def _gimbal_device_attitude_status(self) -> Optional[GimbalDeviceAttitudeStatus]:
        """GimbalDeviceAttitudeStatus received via the PX4-ROS 2 bridge."""
        return self.__gimbal_device_attitude_status

    @_gimbal_device_attitude_status.setter
    def _gimbal_device_attitude_status(self, value: Optional[GimbalDeviceAttitudeStatus]) -> None:
        assert_type(value, get_args(Optional[GimbalDeviceAttitudeStatus]))
        self.__gimbal_device_attitude_status = value

    @property
    def _gimbal_device_set_attitude(self) -> Optional[GimbalDeviceSetAttitude]:
        """GimbalDeviceSetAttitude received via the PX4-ROS 2 bridge."""
        return self.__gimbal_device_set_attitude

    @_gimbal_device_set_attitude.setter
    def _gimbal_device_set_attitude(self, value: Optional[GimbalDeviceSetAttitude]) -> None:
        assert_type(value, get_args(Optional[GimbalDeviceSetAttitude]))
        self.__gimbal_device_set_attitude = value
    #endregion

    #region Setup
    def _setup_matching_pool(self, params_file: str) -> Tuple[str, torch.multiprocessing.Pool]:
        """Imports a matcher from given params file and returns a matching pool

        :param params_file: Parameter file with matcher class name and initializer arguments
        :return: Tuple containing the class_name and matching pool
        """
        matcher_params = self._load_config(params_file)
        module_name, class_name = matcher_params.get('class_name', '').rsplit('.', 1)
        matcher = self._import_class(class_name, module_name)
        matching_pool = torch.multiprocessing.Pool(self.MATCHER_PROCESS_COUNT,
                                                   initializer=matcher.initializer,
                                                   initargs=(matcher, *matcher_params.get('args', []),))  # TODO: handle missing args, do not use default value

        return matcher, matching_pool

    def _load_config(self, yaml_file: str) -> dict:
        """Loads config from the provided YAML file.

        :param yaml_file: Path to the yaml file
        :return: The loaded yaml file as dictionary
        """
        assert_type(yaml_file, str)
        with open(os.path.join(get_package_share_directory(PACKAGE_NAME), yaml_file), 'r') as f:
            try:
                config = yaml.safe_load(f)
                self.get_logger().info(f'Loaded config:\n{config}.')
                return config
            except Exception as e:
                self.get_logger().error(f'Could not load config file {yaml_file} because of exception:'
                                        f'\n{e}\n{traceback.print_exc()}')

    def _setup_map_update_timer(self) -> rclpy.timer.Timer:
        """Sets up a timer to throttle map update requests.

        Initially map updates were triggered in VehicleGlobalPosition message callbacks, but were moved to a separate
        timer since map updates may be needed even if the EKF2 filter does not publish a global position reference (e.g.
        when GPS fusion is turned off in the EKF2_AID_MASK).

        :return: The timer instance
        """
        timer_period = self.get_parameter('map_update.update_delay').get_parameter_value().integer_value
        assert_type(timer_period, int)
        if not 0 <= timer_period:
            error_msg = f'Map update delay must be >0 seconds ({timer_period} provided).'
            self.get_logger().error(error_msg)
            raise ValueError(error_msg)
        timer = self.create_timer(timer_period, self._map_update_timer_callback)
        return timer

    def _map_update_timer_callback(self) -> None:
        """Attempts to update the stored map at regular intervals.

        Calls :meth:`~_update_map` if the center and altitude coordinates for the new map raster are available and the
        :meth:`~_should_update_map` check passes.

        New map is retrieved based on rough guess of the vehicle's global position. If projection is enabled, the
        center of the projected camera field of view is used instead of vehicle position to ensure the field of view is
        best contained in the new map raster.

        :return:
        """
        # Try to get lat, lon, alt from VehicleGlobalPosition if available
        latlonalt = self._latlonalt_from_vehicle_global_position()
        assert_type(latlonalt, LatLonAlt)

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

    def _setup_subscribers(self) -> None:
        """Creates and stores subscribers for microRTPS bridge topics.

        :return:
        """
        for topic_name, d in self._topics.items():
            assert topic_name is not None, f'Topic name not provided in topic: {topic_name}, {d}.'
            assert d is not None, f'Dictionary not provided for topic: {topic_name}.'
            class_ = d.get(self.TOPICS_MSG_KEY, None)
            assert class_ is not None, f'Message definition not provided for {topic_name}.'
            self._topics.update({topic_name: {self.TOPICS_SUBSCRIBER_KEY: self._create_subscriber(topic_name, class_)}})

        self.get_logger().info(f'Subscribers setup complete:\n{self._topics}.')

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

    def __declare_ros_params(self) -> None:
        """Declares ROS parameters

        Uses defaults from :py:mod:`python_px4_ros2_map_nav.ros_param_defaults`. Note that some parameters are declared
        as read_only and cannot be changed at runtime.

        :return:
        """
        read_only = ParameterDescriptor(read_only=True)
        namespace = 'wms'
        self.declare_parameters(namespace, [
            ('url', Defaults.WMS_URL, read_only),
            ('version', Defaults.WMS_VERSION, read_only),
            ('layer', Defaults.WMS_LAYER),
            ('srs', Defaults.WMS_SRS),
            ('request_timeout', Defaults.WMS_REQUEST_TIMEOUT)
        ])

        namespace = 'misc'
        self.declare_parameters(namespace, [
            ('max_pitch', Defaults.MISC_MAX_PITCH),
            ('variance_estimation_length', Defaults.MISC_VARIANCE_ESTIMATION_LENGTH),
            ('min_match_altitude', Defaults.MISC_MIN_MATCH_ALTITUDE),
            ('blur_threshold', Defaults.MISC_BLUR_THRESHOLD),
            ('blur_window_length', Defaults.MISC_BLUR_WINDOW_LENGTH),
            ('visual_odometry', Defaults.MISC_VISUAL_ODOMETRY),
            ('visual_odometry_update_t_threshold', Defaults.MISC_VISUAL_ODOMETRY_UPDATE_T_THRESHOLD),
            ('visual_odometry_update_r_threshold', Defaults.MISC_VISUAL_ODOMETRY_UPDATE_R_THRESHOLD)
        ])

        namespace = 'map_update'
        self.declare_parameters(namespace, [
            ('initial_guess', None),
            ('update_delay', Defaults.MAP_UPDATE_UPDATE_DELAY, read_only),
            ('default_altitude', Defaults.MAP_UPDATE_DEFAULT_ALTITUDE),
            ('gimbal_projection', Defaults.MAP_UPDATE_GIMBAL_PROJECTION),
            ('max_map_radius', Defaults.MAP_UPDATE_MAP_RADIUS_METERS_DEFAULT),
            ('update_map_center_threshold', Defaults.MAP_UPDATE_UPDATE_MAP_CENTER_THRESHOLD),
            ('update_map_radius_threshold', Defaults.MAP_UPDATE_UPDATE_MAP_RADIUS_THRESHOLD),
            ('max_pitch', Defaults.MAP_UPDATE_MAX_PITCH)
        ])

        namespace = 'map_matcher'
        self.declare_parameters(namespace, [
            ('class', Defaults.MAP_MATCHER_CLASS, read_only),
            ('params_file', Defaults.MAP_MATCHER_PARAMS_FILE, read_only)
        ])

        namespace = 'vo_matcher'
        self.declare_parameters(namespace, [
            ('class', Defaults.VO_MATCHER_CLASS, read_only),
            ('params_file', Defaults.VO_MATCHER_PARAMS_FILE, read_only)
        ])

    def _import_class(self, class_name: str, module_name: str) -> object:
        """Dynamically imports class from given module if not yet imported

        :param class_name: Name of the class to import
        :param module_name: Name of module that contains the class
        :return: Imported class
        """
        if module_name not in sys.modules:
            self.get_logger().info(f'Importing module {module_name}.')
            importlib.import_module(module_name)
        imported_class = getattr(sys.modules[module_name], class_name, None)
        assert imported_class is not None, f'{class_name} was not found in module {module_name}.'
        return imported_class

    def _import_matcher(self, matcher_params_file: str) -> Tuple[str, str]:
        """Imports the matcher class based on configuration

        :param matcher_params_file: Matcher parameter file name
        """
        class_path = self.get_parameter('matcher.class').get_parameter_value().string_value
        if class_path is None or matcher_params_file is None:
            msg = f'Class path {class_path} or init args {matcher_params_file} for matcher was None.'
            self.get_logger.error(msg)
            raise ValueError(msg)
        module_name, class_name = class_path.rsplit('.', 1)

        return module_name, class_name
    #endregion

    def _latlonalt_from_vehicle_global_position(self) -> LatLonAlt:
        """Returns lat, lon in WGS84 coordinates and alt in meters from VehicleGlobalPosition.

        The individual values of the LatLonAlt tuple may be None if vehicle global position is not available but a
        LatLonAlt tuple is returned nevertheless.

        :return: LatLonAlt tuple"""
        lat, lon, alt = None, None, None
        if self._vehicle_global_position is not None:
            assert hasattr(self._vehicle_global_position, 'lat') and hasattr(self._vehicle_global_position, 'lon') and \
                   hasattr(self._vehicle_global_position, 'alt')
            lat, lon, alt = self._vehicle_global_position.lat, self._vehicle_global_position.lon, \
                            self._vehicle_global_position.alt
            assert_type(lat, get_args(Union[int, float]))
            assert_type(lon, get_args(Union[int, float]))
            assert_type(alt, get_args(Union[int, float]))
        return LatLonAlt(lat, lon, alt)

    def _alt_from_vehicle_local_position(self) -> Optional[float]:
        """Returns altitude from vehicle local position or None if not available.

        This method tries to return the 'z' value first, and 'dist_bottom' second from the VehicleLocalPosition
        message. If neither are valid, a None is returned.

        :return: Altitude in meters or None if information is not available"""
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
        """Returns lat, lon (WGS84) and altitude (meters) from provided values, or None if not available.

        If some of the initial guess values are not provided, a None is returned in their place within the tuple.

        :return: A lat, lon, alt tuple"""
        initial_guess = self.get_parameter('map_update.initial_guess').get_parameter_value().double_array_value
        if not (len(initial_guess) == 2 and all(isinstance(x, float) for x in initial_guess)):
            lat, lon = None, None
        else:
            lat, lon = initial_guess[0], initial_guess[1]

        return lat, lon, self.get_parameter('map_update.default_altitude').get_parameter_value().double_value

    def _use_gimbal_projection(self) -> bool:
        """Checks if map rasters should be retrieved for projected field of view instead of vehicle position.

        If this is set to false, map rasters are retrieved for the vehicle's global position instead. This is typically
        fine as long as the camera is not aimed too far in to the horizon and has a relatively wide field of view. For
        best results, this should be on to ensure the field of view is fully contained within the area of the retrieved
        map raster.

        :return: True if field of view projection should be used for updating map rasters
        """
        gimbal_projection_flag = self.get_parameter('map_update.gimbal_projection').get_parameter_value().bool_value
        if type(gimbal_projection_flag) is bool:
            return gimbal_projection_flag
        else:
            self.get_logger().warn(f'Could not read gimbal projection flag: {gimbal_projection_flag}. Assume False.')
            return False

    def _sync_timestamps(self, ekf2_timestamp_usec: int) -> None:
        """Synchronizes local timestamp with EKF2's system time.

        This synchronization is done in the :meth:`~vehicle_local_position_callback` and therefore expected to be done
        at high frequency. See :py:attr:`~_time_sync` for more information.

        :param ekf2_timestamp_usec: The time since the EKF2 system start in microseconds
        :return:
        """
        assert_type(ekf2_timestamp_usec, int)
        now_usec = time.time() * 1e6
        self._time_sync = TimePair(now_usec, ekf2_timestamp_usec)

    def _get_ekf2_time(self) -> Optional[int]:
        """Returns current (estimated) EKF2 timestamp in microseconds

        See :py:attr:`~_time_sync` for more information.

        :return: Estimated EKF2 system time in microseconds or None if not available"""
        if self._time_sync is None:
            self.get_logger().warn('Could not estimate EKF2 timestamp.')
            return None
        else:
            now_usec = time.time() * 1e6
            assert now_usec > self._time_sync.local, f'Current timestamp {now_usec} was unexpectedly smaller than ' \
                                                     f'timestamp stored earlier for synchronization ' \
                                                     f'{self._time_sync.local}.'
            ekf2_timestamp_usec = int(self._time_sync.foreign + (now_usec - self._time_sync.local))
            return ekf2_timestamp_usec

    def _get_bbox(self, latlon: Union[LatLon, LatLonAlt], radius_meters: Optional[Union[int, float]] = None) -> BBox:
        """Gets the bounding box containing a circle with given radius centered at given lat-lon fix.

        If the map radius is not provided, a default value is used.

        :param latlon: Center of the bounding box
        :param radius_meters: Radius of the circle in meters enclosed by the bounding box
        :return: The bounding box
        """
        if radius_meters is None:
            radius_meters = self.get_parameter('map_update.map_radius_meters_default')\
                .get_parameter_value().integer_value
        assert_type(latlon, get_args(Union[LatLon, LatLonAlt]))
        assert_type(radius_meters, get_args(Union[int, float]))
        corner_distance = math.sqrt(2) * radius_meters  # Distance to corner of square enclosing circle of radius
        ul = self._move_distance(latlon, (-45, corner_distance))
        lr = self._move_distance(latlon, (135, corner_distance))
        return BBox(ul.lon, lr.lat, lr.lon, ul.lat)

    def _distance(self, latlon1: Union[LatLon, LatLonAlt], latlon2: Union[LatLon, LatLonAlt]) -> float:
        """Returns distance between two points in meters.

        The distance computation is based on latitude and longitude only and ignores altitude.

        :param latlon1: The first point
        :param latlon2: The second point
        :return: The ground distance in meters between the two points
        """
        assert_type(latlon1, get_args(Union[LatLon, LatLonAlt]))
        assert_type(latlon2, get_args(Union[LatLon, LatLonAlt]))
        _, __, dist = self._geod.inv(latlon1.lon, latlon1.lat, latlon2.lon, latlon2.lat)
        return dist

    def _move_distance(self, latlon: Union[LatLon, LatLonAlt], azmth_dist: Tuple[Union[int, float], Union[int, float]])\
            -> LatLon:
        """Returns the point that is a given distance in the direction of azimuth from the origin point.

        :param latlon: Origin point
        :param azmth_dist: Tuple containing azimuth in degrees and distance in meters: (azimuth, distance)
        :return: The point that is given meters away in the azimuth direction from origin
        """
        assert_type(azmth_dist, tuple)
        assert_type(latlon, get_args(Union[LatLon, LatLonAlt]))
        azmth, dist = azmth_dist  # TODO: silly way of providing these args just to map over a zipped list in _update_map, fix it
        assert_type(azmth, get_args(Union[int, float]))
        assert_type(dist, get_args(Union[int, float]))
        lon, lat, azmth = self._geod.fwd(latlon.lon, latlon.lat, azmth, dist)
        return LatLon(lat, lon)

    def _map_size_with_padding(self) -> Optional[Tuple[int, int]]:
        """Returns map size with padding for rotation without clipping corners.

        Because the deep learning models used for predicting matching keypoints between camera image frames and
        retrieved map rasters are not assumed to be rotation invariant, the map rasters are rotated based on camera yaw
        so that they align with the camera images. To keep the scale of the map after rotation the same, black corners
        would appear unless padding is used. Retrieved maps therefore have to squares with the side lengths matching the
        diagonal of the camera frames so that scale is preserved and no black corners appear in the map rasters after
        rotation.

        :return: Padded map size tuple (height, width) or None if the info is not available. The height and width will
        both be equal to the diagonal of the declared (:py:attr:`~_camera_info`) camera frame dimensions.
        """
        dim = self._img_dim()
        if dim is None:
            self.get_logger().warn(f'Dimensions not available - returning None as map size.')
            return None
        assert_type(dim, Dim)
        diagonal = math.ceil(math.sqrt(dim.width ** 2 + dim.height ** 2))
        assert_type(diagonal, int)  # TODO: What if this is float?
        return diagonal, diagonal

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

        This method is a wrapper for :meth:`~declared_img_size`.

        :return: Image dimensions or None if not available
        """
        declared_size = self._declared_img_size()
        if declared_size is None:
            self.get_logger().warn('CDeclared size not available - returning None as image dimensions.')
            return None
        assert_type(declared_size, tuple)
        assert_len(declared_size, 2)
        return Dim(*declared_size)

    def _project_gimbal_fov(self, translation: np.ndarray) -> Optional[np.ndarray]:
        """Returns field of view (FOV) meter coordinates projected using gimbal attitude and camera intrinsics.

        The returned fov coordinates are meters from the origin of projection of the FOV on ground. This method is used
        by :meth:`~_projected_field_of_view_center` when new coordinates for an outgoing WMS GetMap request are needed.

        :param translation: Translation vector (cx, cy, altitude) in meter coordinates
        :return: Projected FOV bounding box in pixel coordinates or None if not available
        """
        assert_shape(translation, (3,))
        rpy = self._get_camera_set_rpy()
        if rpy is None:
            self.get_logger().warn('Could not get RPY - cannot project gimbal FOV.')
            return None

        # Need coordinates in image frame
        rpy = rpy.axes_ned_to_image()

        r = Rotation.from_euler('XYZ', [rpy.roll, rpy.pitch, rpy.yaw], degrees=True).as_matrix()
        e = np.hstack((r, np.expand_dims(translation, axis=1)))
        assert_shape(e, (3, 4))

        if self._camera_info is None:
            self.get_logger().warn('Could not get camera info - cannot project gimbal FOV.')
            return None

        # Intrinsic matrix
        k = np.array(self._camera_info.k).reshape([3, 3])

        # Project image corners to z=0 plane (ground)
        h = inv_homography_from_k_and_e(k, e)
        if h is None:
            self.get_logger().warn('Could not invert homography matrix - cannot project gimbal FOV.')
            return None

        img_dim = self._img_dim()
        if img_dim is None:
            self.get_logger().warn('Could determine image dimensions- cannot project gimbal FOV.')
            return None
        else:
            assert_type(h, np.ndarray)
            # noinspection PyTypeChecker
            dst_corners, _ = get_fov_and_c(img_dim, h)
            dst_corners = dst_corners.squeeze()

        return dst_corners

    def _projected_field_of_view_center(self, origin: LatLonAlt) -> Optional[LatLon]:
        """Returns WGS84 coordinates of projected camera field of view (FOV).

        Used in :meth:`~_map_update_timer_callback` when gimbal projection is enabled to determine center coordinates
        for next WMS GetMap request.

        :param origin: Camera position  # TODO: why is this an argument but all else is not?
        :return: Center of the FOV or None if not available
        """
        if self._camera_info is not None:
            pitch = self._camera_set_pitch()  # TODO: _project_gimbal_fov uses _get_camera_rpy - redundant calls  # TODO: this logic uses old pitch origin (nadir=0)
            if pitch is None:
                self.get_logger().warn('Camera pitch not available, cannot project gimbal field of view.')
                return None
            pitch_from_nadir = 90 + pitch

            #assert 0 <= abs(pitch) <= 90, f'Pitch {pitch} was outside of expected bounds [0, 90].' # TODO: need to handle outside of bounds, cannot assert
            pitch_rad = math.radians(pitch_from_nadir)
            assert origin.alt is not None
            assert hasattr(origin, 'alt')
            hypotenuse = origin.alt * math.tan(pitch_rad)  # Distance from camera origin to projected principal point
            cx = hypotenuse*math.sin(pitch_rad)
            cy = hypotenuse*math.cos(pitch_rad)
            translation = np.array([cx, cy, origin.alt])
            gimbal_fov_pix = self._project_gimbal_fov(translation)

            # Convert gimbal field of view from pixels to WGS84 coordinates
            if gimbal_fov_pix is not None:
                azmths = list(map(lambda x: get_azimuth(x[0], x[1]), gimbal_fov_pix))
                dists = list(map(lambda x: math.sqrt(x[0] ** 2 + x[1] ** 2), gimbal_fov_pix))
                zipped = list(zip(azmths, dists))
                to_wgs84 = partial(self._move_distance, origin)
                gimbal_fov_wgs84 = np.array(list(map(to_wgs84, zipped)))
                ### TODO: add some sort of assertion hat projected FoV is contained in size and makes sense

                # TODO: make this a method of FOV after refactoring this function to use FOV
                # Use projected field of view center instead of global position as map center
                #map_center_latlon = fov_center(gimbal_fov_wgs84)
                left, bottom, right, top = 180, 90, -180, -90
                for pt in gimbal_fov_wgs84:
                    right = pt[1] if pt[1] >= right else right
                    left = pt[1] if pt[1] <= left else left
                    top = pt[0] if pt[0] >= top else top
                    bottom = pt[0] if pt[0] <= bottom else bottom
                map_center_latlon = LatLon((top + bottom) / 2, (left + right) / 2)

                self.publish_projected_fov(gimbal_fov_wgs84, map_center_latlon)  # Note: map center, not principal point
            else:
                self.get_logger().warn('Could not project camera FoV, getting map raster assuming nadir-facing camera.')
                return None
        else:
            self.get_logger().debug('Camera info not available, cannot project FoV, defaulting to global position.')
            return None

        return map_center_latlon  # TODO: using principal point for updating map no good, too close to bottom fov. Principal point still needed but not for updating map.

    def _update_map(self, center: Union[LatLon, LatLonAlt], radius: Union[int, float]) -> None:
        """Instructs the WMS client to get a new map from the WMS server.

        :param center: WGS84 coordinates of map to be retrieved
        :param radius: Radius in meters of circle to be enclosed by the map raster
        :return:
        """
        self.get_logger().info(f'Updating map at {center}, radius {radius} meters.')
        assert_type(center, get_args(Union[LatLon, LatLonAlt]))
        assert_type(radius, get_args(Union[int, float]))
        max_radius = self.get_parameter('map_update.max_map_radius').get_parameter_value().integer_value
        # TODO: need to recover from this, e.g. if its more than max_radius, warn and use max instead. Users could crash this by setting radius to above max radius
        assert 0 < radius <= max_radius, f'Radius should be between 0 and {max_radius}.'

        bbox = self._get_bbox(center, radius)  # TODO: should these things be moved to args? Move state related stuff up the call stack all in the same place. And isnt this a static function anyway?
        assert_type(bbox, BBox)

        map_size = self._map_size_with_padding()
        if map_size is None:
            self.get_logger().warn('Map size not yet available - skipping WMS request.')
            return None

        # Build and send WMS request
        layer_str = self.get_parameter('wms.layer').get_parameter_value().string_value
        srs_str = self.get_parameter('wms.srs').get_parameter_value().string_value
        assert_type(layer_str, str)
        assert_type(srs_str, str)
        try:
            self.get_logger().info(f'Getting map for bbox: {bbox}, layer: {layer_str}, srs: {srs_str}.')
            if self._wms_results is not None:
                assert self._wms_results.ready(), f'Update map was called while previous results were not yet ready.'  # Should not happen - check _should_update_map conditions
            self._wms_results = self._wms_pool.starmap_async(
                WMSClient.worker, [(center, radius, bbox, map_size, layer_str, srs_str)],
                callback=self.wms_pool_worker_callback, error_callback=self.wms_pool_worker_error_callback)
        except Exception as e:
            self.get_logger().error(f'Something went wrong with WMS worker:\n{e},\n{traceback.print_exc()}.')
            return None

    def _local_position_ref_alt(self) -> Optional[float]:
        """Returns local position reference altitude (AMSL)

        :return: Assumed altitude of ground surface in meters above mean sea level
        """
        if self._vehicle_local_position is not None:
            if hasattr(self._vehicle_local_position, 'ref_alt') and \
                    isinstance(self._vehicle_local_position.ref_alt, float):
                return self._vehicle_local_position.ref_alt
            else:
                self.get_logger().error('Vehicle local position did not contain a valid ref_alt value.')
        else:
            self.get_logger().warn('Vehicle local position not available, local position ref_alt unknown.')

        return None

    def _camera_yaw(self) -> Optional[Union[int, float]]:
        """Returns camera yaw in degrees.

        :return: Camera yaw in degrees, or None if not available
        """
        rpy = self._get_camera_set_rpy()
        if rpy is None:
            self.get_logger().warn(f'Could not get camera RPY - cannot return yaw.')
            return None
        assert_type(rpy, RPY)
        camera_yaw = rpy.yaw
        return camera_yaw

    def _get_gimbal_set_attitude(self) -> Optional[Rotation]:
        """Returns gimbal set attitude from :class:`px4_msgs.msg.GimbalDeviceSetAttitude` or None if not available

        :return: Vehicle attitude or None if not available
        """
        if self._gimbal_device_set_attitude is None:
            self.get_logger().warn('No VehicleAttitude message has been received yet.')
            return None
        else:
            gimbal_set_attitude = Rotation.from_quat(self._gimbal_device_set_attitude.q)
            return gimbal_set_attitude

    def _get_camera_set_rpy(self) -> Optional[RPY]:
        """Returns roll-pitch-yaw tuple in NED frame of camera attitude setting.

        True camera attitude may be different if gimbal has not yet stabilized.

        :return: An :class:`util.RPY` tuple
        """
        gimbal_set_attitude = self._get_gimbal_set_attitude()
        if gimbal_set_attitude is None:
            self.get_logger().warn('Gimbal attitude not available, cannot return RPY.')
            return None
        assert_type(gimbal_set_attitude, Rotation)
        gimbal_euler = gimbal_set_attitude.as_euler('xyz', degrees=True)

        if self._vehicle_local_position is None:
            self.get_logger().warn('VehicleLocalPosition is unknown, cannot get heading. Cannot return RPY.')
            return None
        elif not hasattr(self._vehicle_local_position, 'heading'):
            self.get_logger().error('VehicleLocalPosition unexpectedly did contain a heading field.')
            return None
        else:
            heading = self._vehicle_local_position.heading
            if abs(heading) > 180:
                self.get_logger().error(f'VehicleLocalPosition did not have a valid heading value: {heading}, '
                                        f'([-180, 180] expected).')
                return None
            heading = math.degrees(heading)

        self.get_logger().debug('Assuming stabilized gimbal - ignoring vehicle intrinsic pitch and roll for camera RPY.')
        self.get_logger().debug('Assuming zero roll for camera RPY.')  # TODO remove zero roll assumption

        gimbal_yaw = gimbal_euler[0] - 180
        yaw = heading - gimbal_yaw
        yaw = yaw % 360
        if abs(yaw) > 180:  # Important: >, not >= (because we are using mod 180 operation below)
            yaw = yaw % 180 if yaw < 0 else yaw % -180  # Make the compound yaw between -180 and 180 degrees
        roll = 0  # TODO remove zero roll assumption
        pitch = -gimbal_euler[1]
        rpy = RPY(roll, pitch, yaw)

        return rpy

    #region microRTPSBridgeCallbacks
    def image_raw_callback(self, msg: Image) -> None:
        """Handles latest image frame from camera.

        For every image frame, uses :meth:`~_should_match` to determine whether a new :meth:`_match` call needs to be
        made to the neural network. Inputs for the :meth:`_match` call are collected with :meth:`~_match_inputs` and
        saved into :py:attr:`~_stored_inputs` for later use. When the match call returns,
        the :meth:`~_matching_worker_callback` will use the stored inputs for post-processing the matches based on
        the same input_data of data that was used to make the call. It is assumed that the latest stored inputs are the
        same ones that were used for making the :meth:`_match` call, no additional checking or verification is used.

        :param msg: The Image message from the PX4-ROS 2 bridge to decode
        :return:
        """
        # Estimate EKF2 timestamp first to get best estimate
        timestamp = self._get_ekf2_time()
        if timestamp is None:
            self.get_logger().warn('Image frame received but could not estimate EKF2 system time, skipping frame.')
            return None

        self.get_logger().debug('Camera image callback triggered.')
        assert_type(msg, Image)

        cv_image = self._cv_bridge.imgmsg_to_cv2(msg, self.IMAGE_ENCODING)

        # Check that image dimensions match declared dimensions
        img_size = self._declared_img_size()
        if img_size is not None:
            cv_img_shape = cv_image.shape[0:2]
            assert cv_img_shape == img_size, f'Converted cv_image shape {cv_img_shape} did not match declared image ' \
                                             f'shape {img_size}.'

        # Process image frame
        # TODO: save previous image frame and check that new timestamp is greater
        assert self._camera_info is not None
        assert hasattr(self._camera_info, 'k')
        img_dim = self._img_dim()
        assert isinstance(img_dim, Dim)
        image_data = ImageData(image=Img(cv_image), frame_id=msg.header.frame_id, timestamp=timestamp, k=self._camera_info.k.reshape([3, 3]))

        inputs = None  # TODO: the odom flag should be disabled when called for map!

        # Do visual odometry if enabled
        if self._should_vo_match(image_data.image.arr):
            assert self._vo_matching_query is None or self._vo_matching_query.result.ready()
            assert self._vo_matching_pool is not None
            assert self._map_input_data_prev is not None
            try:
                inputs, contextual_map_data = self._match_inputs()
            except TypeError as e:
                # TODO: handle invalid/unavailable inputs with a warning, not error
                self.get_logger().error(f'Data class initialization type error:\n{e}\n{traceback.print_exc()}. '
                                        f'Skipping visual odometry matching.')
                return
            self._vo_input_data = inputs
            vo_reference = self._vo_reference()._match.image_pair.qry  # Access _match intentional, need the raw vo pose, not map pose
            if vo_reference is not None and inputs.vo_fix is not None:  # Need both previous frame and a map fix  # TODO: move this check to should_vo_match!
                image_pair = ImagePair(image_data, vo_reference)
                self._match(image_pair, inputs)
            else:
                self.get_logger().error(f'No visual odometry reference data, returning None.')
                return

        # TODO: store image_data as self._image_data and move the stuff below into a dedicated self._matching_timer?
        if self._should_map_match(image_data.image.arr):  # TODO: possibly redundant checking with _odom_should_match?
            assert self._map_matching_query is None or self._map_matching_query.result.ready()
            if inputs is None:
                # Get inputs if did not yet get them earlier for viz odom
                try:
                    inputs, contextual_map_data = self._match_inputs()
                except TypeError as e:
                    # TODO: handle invalid/unavailable inputs with a warning, not error
                    self.get_logger().error(f'Data class initialization type error:\n{e}\n{traceback.print_exc()}. '
                                            f'Skipping map matching.')
                    return

            self._map_input_data = inputs
            self.get_logger().debug(f'Matching image with timestamp {image_data.timestamp} to map.')
            assert self._map_data is not None  # TODO: check in should_map_match
            assert hasattr(self._map_data, 'image'), 'Map data unexpectedly did not contain the image data.'

            image_pair = ImagePair(image_data, contextual_map_data)
            self._match(image_pair, inputs)

    def camera_info_callback(self, msg: CameraInfo) -> None:
        """Handles latest camera info message.

        :param msg: CameraInfo message from the PX4-ROS 2 bridge
        :return:
        """
        self.get_logger().debug(f'Camera info received:\n{msg}.')
        self._camera_info = msg
        camera_info_topic = self._topics.get('camera_info', {}).get(self.TOPICS_SUBSCRIBER_KEY, None)
        if camera_info_topic is not None:
            self.get_logger().warn('Assuming camera_info is static - destroying the subscription.')
            camera_info_topic.destroy()

    def vehiclelocalposition_pubsubtopic_callback(self, msg: VehicleLocalPosition) -> None:
        """Handles latest VehicleLocalPosition message.

        Uses the EKF2 system time in the message to synchronize local system time.

        :param msg: VehicleLocalPosition from the PX4-ROS 2 bridge
        :return:
        """
        assert_type(msg.timestamp, int)
        self._vehicle_local_position = msg
        self._sync_timestamps(self._vehicle_local_position.timestamp)

    def _get_dynamic_map_radius(self, altitude: Union[int, float]) -> int:
        """Returns map radius that adjusts for camera altitude.

        :param altitude: Altitude of camera in meters
        :return: Suitable map radius in meters
        """
        assert_type(altitude, get_args(Union[int, float]))
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
        return not self._wms_results.ready() if self._wms_results is not None else False

    def _previous_map_too_close(self, center: Union[LatLon, LatLonAlt], radius: Union[int, float]) -> bool:
        """Checks if previous map is too close to new requested one.

        This check is made to avoid retrieving a new map that is almost the same as the previous map. Increasing map
        update interval should not improve accuracy of position estimation unless the map is so old that the field of
        view either no longer completely fits inside (vehicle has moved away or camera is looking in other direction)
        or is too small compared to the size of the map (vehicle altitude has significantly decreased).

        :param center: WGS84 coordinates of new map candidate center
        :param radius: Radius in meters of new map candidate
        :return: True if previous map is too close.
        """
        assert_type(radius, get_args(Union[int, float]))
        assert_type(center, get_args(Union[LatLon, LatLonAlt]))
        if self._map_output_data_prev is not None:
            previous_map_data = self._map_output_data_prev._match.image_pair.ref.map_data
            center_threshold = self.get_parameter('map_update.update_map_center_threshold').get_parameter_value().integer_value
            radius_threshold = self.get_parameter('map_update.update_map_radius_threshold').get_parameter_value().integer_value
            if abs(self._distance(center, previous_map_data.center)) <= center_threshold and \
                    abs(radius - previous_map_data.radius) <= radius_threshold:
                return True

        return False

    def _should_update_map(self, center: Union[LatLon, LatLonAlt], radius: Union[int, float]) -> bool:
        """Checks if a new WMS map request should be made to update old map.

        Map is updated unless (1) there is a previous map that is close enough to provided center and has radius
        that is close enough to new request, (2) previous WMS request is still processing, or (3) camera pitch is too
        large and gimbal projection is used so that map center would be too far or even beyond the horizon.

        :param center: WGS84 coordinates of new map candidate center
        :param radius: Radius in meters of new map candidate
        :return: True if map should be updated
        """
        assert_type(radius, get_args(Union[int, float]))
        assert_type(center, get_args(Union[LatLon, LatLonAlt]))

        # Check conditions (1) and (2) - previous results pending or requested new map too close to old one
        if self._wms_results_pending() or self._previous_map_too_close(center, radius):
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
    #endregion

    #region WMSWorkerCallbacks
    def wms_pool_worker_callback(self, result: List[MapData]) -> None:
        """Handles result from :meth:`python_px4_ros2_map_nav.wms.worker`.

        Saves received result to :py:attr:`~_map_data. The result should be a collection containing a single
        :class:`~data.MapData`.

        :param result: Results from the asynchronous call
        :return:
        """
        assert_len(result, 1)
        result = result[0]
        assert_type(result, MapData)
        assert result.image.arr.shape[0:2] == self._map_size_with_padding(), 'Decoded map is not of specified size.'
        self.get_logger().info(f'Map received for bbox: {result.bbox}.')
        self._map_data = result

    def wms_pool_worker_error_callback(self, e: BaseException) -> None:
        """Handles errors from WMS pool worker.

        :param e: Exception returned by the worker
        :return:
        """
        self.get_logger().error(f'Something went wrong with WMS process:\n{e},\n{traceback.print_exc()}.')
    #endregion

    #region MatchingWorkerCallbacks
    def _matching_worker_callback_common(self, results: list, visual_odometry: bool) -> Optional[OutputData]:
        """Common logic for both vo and map matching worker callbacks

        :param results: Results from the matching worker
        :param visual_odometry: True if results are from visual odometry worker
        :return: Parsed output data
        """
        pose = results[0]
        if pose is not None:
            self._store_extrinsic_guess(pose, odom=visual_odometry)
        else:
            self.get_logger().warn(f'Could not compute _match, returning None.')
            return None
        query = self._vo_matching_query if visual_odometry else self._map_matching_query
        match = Match(image_pair=query.image_pair, pose=pose)
        output_data = self._compute_output(match, query.input_data)

        # noinspection PyUnreachableCode
        if __debug__ and output_data is not None:
            # TODO: vo_enabled argument should not be needed (see Visualization.update() method)
            vo_enabled = self.get_parameter('misc.visual_odometry').get_parameter_value().bool_value
            self._visualization.update(output_data, vo_enabled)

        return output_data

    def map_matching_worker_error_callback(self, e: BaseException) -> None:
        """Error callback for matching worker.

        :return:
        """
        self.get_logger().error(f'Matching process returned and error:\n{e}\n{traceback.print_exc()}')

    def map_matching_worker_callback(self, results: list) -> None:
        """Callback for matching worker.

        Retrieves latest :py:attr:`~_stored_inputs` and uses them to call :meth:`~_process_matches`. The stored inputs
        are needed so that the post-processing is done using the same state information that was used for initiating
        the match in the first place. For example, camera pitch may have changed since then (e.g. if match takes 100ms)
        and current camera pitch should therefore not be used for processing the matches.

        :return:
        """
        output_data = self._matching_worker_callback_common(results, False)
        if output_data is None:
            self.get_logger().debug('Position estimate was not good or could not be obtained, skipping this map match.')
        else:
            self._push_estimates(np.array(output_data.position), False)
            if self._variance_window_full(False):
                output_data.sd = np.std(self._map_estimation_history, axis=0)
                self.publish(output_data)
            else:
                self.get_logger().debug('Waiting to get more data to estimate position error, not publishing yet.')

            self._vo_reset()
            self._map_input_data_prev = self._map_input_data
            self._map_output_data_prev = output_data

    def vo_matching_worker_error_callback(self, e: BaseException) -> None:
        """Error callback for visual odometry matching worker.

        :return:
        """
        self.get_logger().error(f'Visual odometry matching process returned an error:\n{e}\n{traceback.print_exc()}')

    def vo_matching_worker_callback(self, results: list) -> None:
        """Callback for visual odometry matching worker.

        Retrieves latest :py:attr:`~_odom_stored_inputs` and uses them to call :meth:`~_process_matches`. The stored
        inputs are needed so that the post-processing is done using the same state information that was used for
        initiating the match. For example, camera pitch may have changed since then (e.g. if match takes 100ms) and
        current camera pitch should therefore not be used for processing the matches.

        :return:
        """
        if self._vo_input_data is None:
            # VO reset happened while matching
            self.get_logger().warn('VO match received but VO reset happend in the meantime.')
            return

        output_data = self._matching_worker_callback_common(results, True)
        if output_data is None:
            self.get_logger().warn('Bad visual odometry match. Resetting visual odometry and map match.')
            self._vo_reset()
        else:
            if self._should_fix_vo(output_data):
                self._vo_output_data_fix = output_data

            self._push_estimates(np.array(output_data.position), True)
            if self._variance_window_full(True):
                output_data.sd = np.std(self._vo_estimation_history, axis=0)
                self.publish(output_data)
            else:
                self.get_logger().debug('Waiting to get more data to estimate position error, not publishing yet.')

            self._vo_input_data_prev = self._vo_input_data
            self._vo_output_data_prev = output_data
    #endregion

    def _camera_set_pitch(self) -> Optional[Union[int, float]]:
        """Returns camera pitch setting in degrees relative to nadir.

        Pitch of 0 degrees is a nadir facing camera, while a positive pitch of 90 degrees means the camera is facing
        the direction the vehicle is heading (facing horizon).

        Note: this is the pitch setting, true pitch may be different if gimbal has not yet stabilized.

        :return: Camera pitch in degrees, or None if not available
        """
        rpy = self._get_camera_set_rpy()
        if rpy is None:
            self.get_logger().warn('Gimbal RPY not available, cannot compute camera pitch.')
            return None
        assert_type(rpy, RPY)
        return rpy.pitch

    def _vo_reference(self) -> Optional[OutputData]:
        """Returns previous image data that should be used for visual odometry matching.

        First priority is the vo fixed frame reference. If no fixed frame exists, try latest map frame.
        
        :return: Previous OutputData from either map or vo matching, or None if not available
        """
        if self._vo_output_data_fix is not None:
            return self._vo_output_data_fix
        elif self._map_output_data_prev is not None:
            return self._map_output_data_prev
        else:
            self.get_logger().debug('No previous frame available, returning None.')
            return None

    def _camera_pitch_too_high(self, max_pitch: Union[int, float]) -> bool:
        """Returns True if (set) camera pitch exceeds given limit.

        Used to determine whether camera pitch setting is too high up from nadir to make matching against a map
        worthwhile.

        :param max_pitch: The limit for the pitch over which it will be considered too high
        :return: True if pitch is too high
        """
        assert_type(max_pitch, get_args(Union[int, float]))
        camera_pitch = self._camera_set_pitch()
        if camera_pitch is not None:
            if camera_pitch + 90 > max_pitch:
                self.get_logger().debug(f'Camera pitch {camera_pitch} is above limit {max_pitch}.')
                return True
        else:
            self.get_logger().warn(f'Could not determine camera pitch.')
            return True

        return False

    # TODO: this is a check, should not push blur here? Easy to call this multiple times for the same frame
    def _image_too_blurry(self, img: np.ndarray) -> bool:
        """Returns True if image is deemed too blurry for matching

        Also pushes the blur value to a stack using :meth:`~_push_blur`.

        :param img: Image to match
        :return: True if image is too blurry
        """
        blur_threshold = self.get_parameter('misc.blur_threshold').get_parameter_value().double_value
        blur = cv2.Laplacian(img, cv2.CV_64F).var()
        self._push_blur(blur)
        sd = np.std(self._blurs)
        mn = np.mean(self._blurs)
        threshold = mn - blur_threshold * sd
        if blur < threshold:
            # Expected to reject a fixed proportion of images so debug message more appropriate than warning
            self.get_logger().debug(f'Image too blurry (blur: {blur}, mean: {mn}, sd: {sd}, threshold: {threshold}). '
                                    f'Skipping matching.')
            return True
        else:
            return False

    def _push_blur(self, blur: float) -> None:
        """Pushes blur estimates to :py:attr:`~_blurs`

        Pops the oldest estimate from the stack if needed.

        :param blur: Blur value
        :return:
        """
        if self._blurs is None:
            self._blurs = np.array([blur])
        else:
            window_length = self.get_parameter('misc.blur_window_length').get_parameter_value().integer_value
            assert window_length > 0, f'Window length for storing blur should be >0 ({window_length} provided).'
            obs_count = len(self._blurs)
            assert 0 <= obs_count <= window_length
            if obs_count == window_length:
                # Pop oldest values
                self._blurs = np.delete(self._blurs, 0, 0)

            # Add newest values
            self._blurs = np.append(self._blurs, blur)

    def _store_extrinsic_guess(self, pose: Pose, odom: bool = False) -> None:
        """Stores rotation and translation vectors for use by :func:`cv2.solvePnPRansac` in :meth:`~_process_matches`.

        Assumes previous solution to the PnP problem will be close to the new solution. See also
        :meth:`~_retrieve_extrinsic_guess`.

        :param pose: Pose to store
        :param odom: Set to True to store for visual odometry, otherwise map matching is assumed
        :return:
        """
        if odom:
            self._pose_vo_guess = pose
        else:
            self._pose_map_guess = pose

    def _retrieve_extrinsic_guess(self, odom: bool = False) -> Optional[Pose]:
        """Retrieves stored rotation and translation vectors for use by :func:`cv2.solvePnPRansac` in
         :meth:`~_process_matches`.

        Assumes previous solution to the PnP problem will be close to the new solution. See also
        :meth:`~_store_extrinsic_guess`.

        # TODO: require that timestamp of previous solution is not too old

        :param odom: Set to true to retrieve extrinsic guess for visual odometry, otherwise map matching is assumed
        :return: Requested _pose, or None if not available
        """
        if odom:
            return self._pose_vo_guess
        else:
            return self._pose_map_guess

    def _should_fix_vo(self, output_data: OutputData) -> bool:
        """Returns True if previous visual odometry fixed reference frame should be updated

        Assumes fx == fy (focal lengths in x and y dimensions are the approximately same).
g
        :param output_data: Output data from the visual odometry matching
        """
        # TODO: turn the info messages to debugging messages
        # If has not been fixed yet
        if self._vo_output_data_fix is None:
            self.get_logger().info(f'Visual odometry fixed frame missing. Fixing it now.')
            return True

        # Check whether camera translation is over threshold
        t_threshold = self.get_parameter('misc.visual_odometry_update_t_threshold').get_parameter_value().double_value
        camera_translation = np.linalg.norm(output_data._match.camera_position_difference.squeeze())
        threshold = t_threshold * output_data._match.image_pair.qry.fx
        if camera_translation > threshold:
            self.get_logger().info(f'Camera translation {camera_translation} over threshold {threshold}, fixing vo '
                                   f'frame.')
            return True

        # Check whether camera rotation is over threshold
        r_threshold = self.get_parameter('misc.visual_odometry_update_r_threshold').get_parameter_value().double_value
        rotvec = Rotation.from_matrix(output_data._match.pose.r).as_rotvec()
        camera_rotation = np.linalg.norm(rotvec)
        threshold = r_threshold
        if camera_rotation > threshold:
            self.get_logger().info(f'Camera rotation {camera_rotation} over threshold {threshold}, fixing vo '
                                   f'frame.')
            return True

        return False

    def _vo_reset(self) -> None:
        """Resets accumulated _match

        Used when a new map match is found or visual odometry has lost track (bad match with visual odometry).

        :param k: Camera intrinsics matrix
        """
        # Reset accumulated position differential
        self._vo_input_data = None
        self._vo_input_data_prev = None
        self._vo_output_data_prev = None
        self._vo_output_data_fix = None

    # TODO: how to estimate if fov_wgs84 is zero (cannot be projected because camera pitch too high)?
    def _estimate_altitude_scaling(self, fov_pix: np.ndarray, fov_wgs84: np.ndarray) -> float:
        """Estimates altitude scaling factor

        Altitude in t is in rotated and cropped map raster pixel coordinates. We can use fov_pix and fov_wgs84 to
        find out the right scale in meters. Distance in pixels is computed from lower left and lower right corners
        of the field of view (bottom of fov assumed more stable than top), while distance in meters is computed from
        the corresponding WGS84 latitude and latitude coordinates.

        :param fov_pix: Field of view in pixel coordinates
        :param fov_wgs84: Field of view in WGS84 coordinates
        :return: Altitude scaling factor
        """
        distance_in_pixels = np.linalg.norm(fov_pix[1]-fov_pix[2])  # fov_pix[1]: lower left, fov_pix[2]: lower right
        distance_in_meters = self._distance(LatLon(*fov_wgs84[1].squeeze().tolist()),
                                            LatLon(*fov_wgs84[2].squeeze().tolist()))
        altitude_scaling = abs(distance_in_meters / distance_in_pixels)

        return altitude_scaling

    @staticmethod
    def _estimate_attitude(map_match: Match) -> np.ndarray:
        """Estimates gimbal attitude from _match and camera yaw in global NED frame

        Estimates attitude in NED frame so need map match (uses ContextualMapData.rotation), not just any match

        :param map_match: Map match
        """
        # TODO: Estimate vehicle attitude from estimated camera attitude
        #  Problem is gimbal relative attitude to vehicle body not known if gimbal not yet stabilized to set attitude,
        #  at least when using GimbalDeviceSetAttitude provided quaternion
        # Convert estimated rotation to attitude quaternion for publishing
        gimbal_estimated_attitude = Rotation.from_matrix(map_match.pose.r.T)  # in rotated map pixel frame
        gimbal_estimated_attitude *= Rotation.from_rotvec(-(np.pi/2) * np.array([1, 0, 0]))  # camera body _match
        assert (map_match.image_pair.mapful())
        assert (map_match.image_pair.ref, ContextualMapData)  # Alternative way to assert mapful()
        gimbal_estimated_attitude *= Rotation.from_rotvec(map_match.image_pair.ref.rotation * np.array([0, 0, 1]))  # unrotated map pixel frame

        # Re-arrange axes from unrotated (original) map pixel frame to NED frame
        rotvec = gimbal_estimated_attitude.as_rotvec()
        gimbal_estimated_attitude = Rotation.from_rotvec([-rotvec[1], rotvec[0], rotvec[2]])

        return gimbal_estimated_attitude

    @staticmethod
    def _estimate_fov(img_dim: Dim, h_pose: np.ndarray, map_match: Match) -> FOV:
        """Estimates field of view and principal point in both pixel and WGS84 coordinates

        :param img_dim: Image dimensions
        :param h_pose: Homography matrix against reference image (map or previous frame)
        :param map_match: Fixed map_match against map
        :return: Field of view and principal point in pixel and WGS84 coordinates, respectively
        """
        # TODO: what if wgs84 coordinates are not valid? H projects FOV to horizon?
        assert_type(map_match.image_pair.ref, ContextualMapData)  # Need pix_to_wgs84
        h_wgs84 = map_match.image_pair.ref.pix_to_wgs84 @ map_match.inv_h
        fov_pix, c_pix = get_fov_and_c(img_dim, h_pose)
        fov_wgs84, c_wgs84 = get_fov_and_c(img_dim, h_wgs84)

        fov = FOV(fov_pix=fov_pix,
                  fov=fov_wgs84,
                  c_pix=c_pix,
                  c=c_wgs84)

        return fov

    def _estimate_position(self, map_match: Match, fov: FOV, ground_elevation: Optional[float]) -> Tuple[LatLonAlt, float]:
        """Estimates camera position (WGS84 coordinates + altitude in meters above mean sea level (AMSL)) as well as
        terrain altitude in meters.

        :param map_match: Camera map map_match in pixel (world) space
        :param fov: Camera field of view estimate
        :return: Camera position LatLonAlt, and altitude from ground in meters
        """
        altitude_scaling = self._estimate_altitude_scaling(fov.fov_pix, fov.fov)
        # Translation in WGS84 (and altitude or z-axis translation in meters above ground)
        assert_type(map_match.image_pair.ref, ContextualMapData)  # need pix_to_wgs84
        t_wgs84 = map_match.image_pair.ref.pix_to_wgs84 @ np.append(map_match.camera_position[0:2], 1)
        t_wgs84[2] = -altitude_scaling * map_match.camera_position[2]  # In NED frame z-coordinate is negative above ground, make altitude >0
        position = t_wgs84.squeeze().tolist()
        position = LatLonAlt(*position)

        # Check that we have all the values needed for a global position
        # if not all(position) or any(map(np.isnan, position)):
        if not all([(isinstance(x, float) or np.isnan(x)) for x in position]):
            self.get_logger().warn(f'Could not determine global position, some fields were empty: {position}.')
            return None

        # Get altitude above mean sea level (AMSL)
        terrain_altitude = position.alt
        if ground_elevation is None:
            self.get_logger().debug('Ground plane elevation (AMSL) unavailable. Setting position.alt as None.')  # TODO: or return LatLon instead?
            position = LatLonAlt(*position[0:2], None)
        else:
            position = LatLonAlt(*position[0:2], position.alt + ground_elevation)

        return position, terrain_altitude

    @staticmethod
    def _estimate_map_match(match: Match, input_data: InputData) -> Match:
        """Estimates _match against the latest map frame

        :param match: Match for the match
        :param input_data: Input data context
        :return: Estimated _match against map
        """
        if not match.image_pair.mapful():
            # Combine with latest map match
            assert input_data.vo_fix is not None  # Should be checked in image_raw_callback and/or _should_vo_match
            map_match = match @ input_data.vo_fix.map_match
        else:
            # TODO: just return Match or need a copy()
            map_match = Match(
                image_pair=match.image_pair,
                pose=Pose(match.pose.r, match.pose.t)
            )

        return map_match

    #region Match
    def _compute_output(self, match: Match, input_data: InputData) -> Optional[OutputData]:
        """Process the estimated camera _match into OutputData

        :param match: Estimated _match between images
        :param input_data: InputData of vehicle state variables from the time the image was taken
        :return: Computed output_data if a valid estimate was obtained
        """
        map_match = self._estimate_map_match(match, input_data)
        fov = self._estimate_fov(match.image_pair.qry.image.dim, match.inv_h, map_match)
        position, terrain_altitude = self._estimate_position(map_match, fov, input_data.ground_elevation)  # TODO: make a dataclass out of position too
        attitude = self._estimate_attitude(map_match)  # TODO Make a dataclass out of attitude?

        # Init output
        # TODO: make OutputData immutable and assign all values in constructor here
        output_data = OutputData(input=input_data,
                                 _match=match,
                                 fixed_camera=FixedCamera(fov=fov, map_match=map_match),
                                 position=position,
                                 terrain_altitude=terrain_altitude,
                                 attitude=attitude,
                                 sd=None)

        if self._good_match(output_data):
            return output_data
        else:
            self.get_logger().debug(f'Bad match computed, returning None for this frame (mapful: {match.image_pair.mapful()}.')
            return None

    def _match_inputs(self) -> Tuple[InputData, ContextualMapData]:
        """Returns a dictionary input_data of the input data required to perform and process a match.

        Processing of matches is asynchronous, so this method provides a way of taking a input_data of the input arguments
        to :meth:`_process_matches` from the time image used for the matching was taken.

        :return: The input data
        """
        input_data = InputData(
            vo_fix=self._vo_reference().fixed_camera if self._vo_reference() is not None else None,
            ground_elevation=self._local_position_ref_alt()
        )

        img_dim = self._img_dim()
        assert_type(img_dim, Dim)  # TODO: handle None?

        # Get cropped and rotated map
        camera_yaw_deg = self._camera_yaw()  # TODO: handle None?
        camera_yaw = math.radians(camera_yaw_deg) if camera_yaw_deg is not None else None
        assert -np.pi <= camera_yaw <= np.pi, f'Unexpected gimbal yaw value: {camera_yaw} ([-pi, pi] expected).'

        contextual_map_data = ContextualMapData(rotation=camera_yaw, map_data=self._map_data, crop=img_dim)
        return copy.deepcopy(input_data), contextual_map_data

    def _good_match(self, output_data: OutputData) -> bool:
        """Uses heuristics for determining whether position estimate is good or not.

        :param output_data: Computed output
        :return: True if match is good
        """
        # Altitude below startin altitude?
        if output_data.terrain_altitude < 0:  # TODO: or is nan
            self.get_logger().warn(f'Match terrain altitude {output_data.terrain_altitude} was negative, assume bad '
                                   f'match.')
            return False

        # Estimated field of view has unexpected shape?
        if not is_convex_isosceles_trapezoid(output_data.fixed_camera.fov.fov_pix):
            self.get_logger().warn(f'Match fov_pix {output_data.fixed_camera.fov.fov_pix.squeeze().tolist()} was not a convex isosceles '
                                   f'trapezoid, assume bad match.')
            return False

        # Estimated translation vector blows up?
        reference = np.array([output_data.fixed_camera.map_match.image_pair.qry.k[0][2], output_data.fixed_camera.map_match.image_pair.qry.k[1][2], output_data.fixed_camera.map_match.image_pair.qry.k[0][0]])  # TODO: refactor this line
        if (np.abs(output_data._match.pose.t).squeeze() >= 3 * reference).any() or \
                (np.abs(output_data.fixed_camera.map_match.pose.t).squeeze() >= 6 * reference).any():  # TODO: The 3 and 6 are an arbitrary threshold, make configurable
            self.get_logger().error(f'Match.pose.t {output_data._match.pose.t} & map_match.pose.t {output_data.fixed_camera.map_match.pose.t} have values '
                                    f'too large compared to (cx, cy, fx): {reference}.')
            return False

        return True

    def _have_map_match(self) -> None:
        """Returns True if an existing map match is in store

        :return: True if a map match has been made earlier
        """
        #assert self._map_input_data_prev is not None  # TODO: why was this here?
        return self._map_output_data_prev is not None

    def _should_vo_match(self, img: np.ndarray) -> bool:
        """Determines whether _odom_match should be called based on whether previous match is still being processed.

        Match should be attempted if (1) visual odometry is enabled, (2) previous image frame is available, (3) there
        are no pending visual odometry match results and (4) image is not too blurry. Unlike :meth:`~should_match`, the
        visual odometry ignores the drone altitude and camera pitch checks since they are not assumed to be relevant
        for comparing successive frames against each other.

        :param img: Image to match
        :return: True if matching should be attempted
        """
        # Check whether visual odometry matching is enabled
        visual_odometry = self.get_parameter('misc.visual_odometry').get_parameter_value().bool_value
        if not visual_odometry:
            return False

        # Check whether previous image frame data is available
        if not self._have_map_match():  # or self._vo_input_data_prev is None:
            #assert self._map_input_data_prev is None  # TODO: why was this originally here? Need to check for output, not input
            assert self._map_output_data_prev is None
            assert self._vo_input_data_prev is None  # If no map match, reset odom should have been called
            return False

        # Check that a request is not already running
        if not (self._vo_matching_query is None or self._vo_matching_query.result.ready()):
            return False

        # Check if is image too blurry
        # if self._image_too_blurry(qry):
        #    self.get_logger().warn('ODOM TOO BLURRY.')
        #    return False

        return True

    def _should_map_match(self, img: np.ndarray) -> bool:
        """Determines whether _match should be called based on whether previous match is still being processed.

        Match should be attempted if (1) there are no pending match results, (2) camera pitch is not too high (e.g.
        facing horizon instead of nadir), (3) drone is not flying too low, and (4) image is not too blurry.

        :param img: Image to match
        :return: True if matching should be attempted
        """
        # Check that _map_data exists (map has been retrieved)
        if self._map_data is None:
            return False

        # Check condition (1) - that a request is not already running
        if not (self._map_matching_query is None or self._map_matching_query.result.ready()):  # TODO: confirm that hasattr result
            return False

        # Check condition (2) - whether camera pitch is too large
        max_pitch = self.get_parameter('misc.max_pitch').get_parameter_value().integer_value
        if self._camera_pitch_too_high(max_pitch):
            self.get_logger().warn(f'Camera pitch is not available or above limit {max_pitch}. Skipping matching.')
            return False

        # Check condition (3) - whether vehicle altitude is too low
        min_alt = self.get_parameter('misc.min_match_altitude').get_parameter_value().integer_value
        altitude = self._alt_from_vehicle_local_position()  # assume this is distance to ground
        if not isinstance(min_alt, int) or altitude < min_alt:
            self.get_logger().warn(f'Altitude {altitude} was lower than minimum threshold for matching ({min_alt}) or '
                                   f'could not be determined. Skipping matching.')
            return False

        # Check condition (4) - is image too blurry?
        if self._image_too_blurry(img):
            return False

        return True

    def _match(self, image_pair: ImagePair, input_data: InputData) -> None:
        """Instructs the _match estimator to estimate the camera _match for the image pair

        :param image_pair: The image pair to match
        :param input_data: Input data context
        :return:
        """
        if image_pair.mapful():
            assert self._map_matching_query is None or self._map_matching_query.result.ready()
            self._map_matching_query = AsyncQuery(
                result=self._map_matching_pool.starmap_async(
                    self._map_matcher.worker,
                    [(image_pair, self._retrieve_extrinsic_guess(False))],  # TODO: have k in input data after all? or somehow inside image_pair?
                    callback=self.map_matching_worker_callback,
                    error_callback=self.map_matching_worker_error_callback
                ),
                image_pair=image_pair,
                input_data=input_data  # TODO: no longer passed to matching, this is "context", not input
            )
        else:
            assert self._vo_matching_query is None or self._vo_matching_query.result.ready()
            self._vo_matching_query = AsyncQuery(
                result=self._vo_matching_pool.starmap_async(
                    self._vo_matcher.worker,
                    [(image_pair, self._retrieve_extrinsic_guess(True))],
                    callback=self.vo_matching_worker_callback,
                    error_callback=self.vo_matching_worker_callback
                ),
                image_pair=image_pair,
                input_data=input_data  # TODO: no longer passed to matching, this is "context", not input
            )
    #endregion

    #region Variance
    @staticmethod
    def _window_full(stack: np.ndarray, window_length: int) -> bool:
        """Return True if stack is full

        :param stack: Map or vo estimation history stack
        :param window_length: Configured max length for the estimation history (stack max height)
        :return: True if stack is full
        """
        if stack is not None:
            obs_count = len(stack)
            if obs_count >= window_length:
                assert obs_count == window_length
                return True
            else:
                assert 0 <= obs_count < window_length
                return False
        else:
            return False

    def _variance_window_full(self, visual_odometry: bool) -> bool:
        """Returns true if the variance estimation window is full.

        :param visual_odometry: True if need to check vo variance windo
        :return: True if :py:attr:`~_map_estimation_history` or :py:attr:`~_vo_estimation_history` is full
        """
        window_length = self.get_parameter('misc.variance_estimation_length').get_parameter_value().integer_value
        if visual_odometry:
            return self._window_full(self._vo_estimation_history, window_length)
        else:
            return self._window_full(self._map_estimation_history, window_length)

    def _update_estimation_history(self, stack: np.ndarray, position: np.ndarray) -> np.ndarray:
        """Pushes a position estimate to the stack and pops an older one if needed

        :param stack: Map or visual odometry estimation history stack
        :param: position: Position estimate
        :return: Returns the new stack
        """
        if stack is None:
            # Compute rotations in radians around x, y, z axes (get RPY and convert to radians?)
            assert_len(position, 3)
            stack = position.reshape(-1, len(position))
        else:
            window_length = self.get_parameter(
                'misc.variance_estimation_length').get_parameter_value().integer_value
            assert window_length > 0, f'Window length for estimating variances should be >0 ({window_length} ' \
                                      f'provided).'
            obs_count = len(stack)
            assert 0 <= obs_count <= window_length
            if obs_count == window_length:
                # Pop oldest values
                stack = np.delete(stack, 0, 0)

            # Add newest values
            stack = np.vstack((stack, position))

        return stack

    # Refactor redundant logic!
    def _push_estimates(self, position: np.ndarray, visual_odometry: bool) -> None:
        """Pushes position estimates to :py:attr:`~_map_estimation_history`

        Pops the oldest estimate from the window if needed.

        :param position: Match translation (x, y, z) in WGS84
        :param visual_odometry: True input is from visual odometry, map is assumed otherwise
        :return:
        """
        if visual_odometry:
            self._vo_estimation_history = self._update_estimation_history(self._vo_estimation_history, position)
        else:
            self._map_estimation_history = self._update_estimation_history(self._map_estimation_history, position)
    #endregion

    #region PublicAPI
    @abstractmethod
    def publish(self, output_data: OutputData) -> None:
        """Publishes or exports computed output

        This method should be implemented by an extending class to adapt for any given use case.
        """
        pass

    @abstractmethod
    def publish_projected_fov(self, fov: np.ndarray, c: np.ndarray) -> None:
        """Publishes projected field of view (FOV) and principal point

        This method should be implemented by an extending class to adapt for any given use case.
        """
        pass
    #endregion

    def terminate_wms_pool(self):
        """Terminates the WMS Pool.

        :return:
        """
        if self._wms_pool is not None:
            self.get_logger().info('Terminating WMS pool.')
            self._wms_pool.terminate()

    def destroy_timers(self):
        """Destroys the map update timer.

        :return:
        """
        if self._map_update_timer is not None:
            self.get_logger().info('Destroying map update timer.')
            assert_type(self._map_update_timer, rclpy.timer.Timer)
            self._map_update_timer.destroy()
