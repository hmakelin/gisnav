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

from multiprocessing.pool import Pool, AsyncResult
from pyproj import Geod
from typing import Optional, Union, Tuple, get_args, List
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from owslib.wms import WebMapService

from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation
from functools import partial, lru_cache
from python_px4_ros2_map_nav.util import setup_sys_path, convert_fov_from_pix_to_wgs84, get_bbox_center, BBox, Dim,\
    rotate_and_crop_map, visualize_homography, get_fov, LatLon, fov_to_bbox, get_angle,\
    create_src_corners, RPY, LatLonAlt, ImageFrame, assert_type, assert_ndim, assert_len, assert_shape,\
    assert_first_stamp_greater, MapFrame

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
        raise e  # TODO: anything here?


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

        # Dict for storing all microRTPS bridge subscribers and publishers
        self._topics = {self.PUBLISH_KEY: {}, self.SUBSCRIBE_KEY: {}}
        self._setup_topics()

        # Setup vehicle visual odometry publisher timer
        self._timer = self._setup_timer()
        self._publish_timestamp = 0

        # Converts image_raw to cv2 compatible image
        self._cv_bridge = CvBridge()

        # Setup SuperGlue
        self._superglue = self._setup_superglue()

        # Used for pyproj transformations
        self._geod = Geod(ellps=self.PYPROJ_ELLIPSOID)

        # Must check for None when using these
        # self._image_frame = None  # Not currently used / needed
        self._previous_image_frame = None
        self._map_frame = None
        self._previous_map_frame = None

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
    def _superglue(self) -> SuperGlue:
        """SuperGlue graph neural network (GNN) estimator for matching keypoints between images."""
        return self.__superglue

    @_superglue.setter
    def _superglue(self, value: SuperGlue) -> None:
        assert_type(SuperGlue, value)
        self.__superglue = value

    @property
    def _timer(self) -> rclpy.timer.Timer:
        """Timer for controlling publish frequency of outgoing VehicleVisualOdometry messages."""
        return self.__timer

    @_timer.setter
    def _timer(self, value: rclpy.timer.Timer) -> None:
        assert_type(rclpy.timer.Timer, value)
        self.__timer = value

    @property
    def _publish_timestamp(self) -> int:
        """Timestamp in of when last VehicleVisualOdometry message was published."""
        return self.__publish_timestamp

    @_publish_timestamp.setter
    def _publish_timestamp(self, value: int) -> None:
        assert_type(int, value)
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
    def _previous_image_frame(self) -> Optional[ImageFrame]:
        """Previous image frame and supporting metadata which is compared to current frame for a velocity estimate."""
        return self.__previous_image_frame

    @_previous_image_frame.setter
    def _previous_image_frame(self, value: Optional[ImageFrame]) -> None:
        assert_type(get_args(Optional[ImageFrame]), value)
        self.__previous_image_frame = value

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

    def _setup_timer(self) -> rclpy.timer.Timer:
        """Sets up a timer to control the publish rate of vehicle visual odometry.

        :return: The timer instance
        """
        # Setup publish timer
        frequency = self.get_parameter('misc.publish_frequency').get_parameter_value().integer_value
        assert_type(int, frequency)
        if not 0 <= frequency:
            error_msg = f'Publish frequency must be >0 Hz ({frequency} provided).'
            self.get_logger().error(error_msg)
            raise ValueError(error_msg)
        if not 30 <= frequency <= 50:
            warn_msg = f'Publish frequency should be between 30 and 50 Hz ({frequency} provided) for EKF2 filter.'
            self.get_logger().warn(warn_msg)
        timer_period = 1.0 / frequency
        timer = self.create_timer(timer_period, self._vehicle_visual_odometry_timer_callback)
        return timer

    def _vehicle_visual_odometry_timer_callback(self) -> None:
        """Publishes the vehicle visual odometry message at given intervals.

        :return:
        """
        if self._vehicle_visual_odometry is not None:
            assert_type(VehicleVisualOdometry, self._vehicle_visual_odometry)
            now = time.time_ns()
            frequency = None
            if self._publish_timestamp is not None:
                assert now > self._publish_timestamp  # TODO: Is it possible that they are the same?
                frequency = 1e9 * 1/(now - self._publish_timestamp)
                self._publish_timestamp = now
            self.get_logger().debug(f'Publishing vehicle visual odometry message:\n{self._vehicle_visual_odometry}.'
                                    f'Publish frequency {frequency}.')
            self._topics.get(self.PUBLISH_KEY).get(self.VEHICLE_VISUAL_ODOMETRY_TOPIC_NAME)\
                .publish(self._vehicle_visual_odometry)

    def _declare_ros_params(self, config: dict) -> None:
        """Declares ROS parameters from a config file.

        :param config: The value of the ros__parameters key from the parsed configuration file.
        :return:
        """
        # TODO: add defaults here instead of Nones and do not use .yaml file for defaults
        # TODO: log warning if config has a param that is not declared here!
        namespace = 'wms'
        self.declare_parameters(namespace, [
            ('url', config.get(namespace, {}).get('url', None), ParameterDescriptor(read_only=True)),
            ('version', config.get(namespace, {}).get('version', None), ParameterDescriptor(read_only=True)),
            ('layer', config.get(namespace, {}).get('layer', None)),
            ('srs', config.get(namespace, {}).get('srs', None)),
            ('request_timeout', config.get(namespace, {}).get('request_timeout', None))
        ])

        namespace = 'misc'
        self.declare_parameters(namespace, [
            ('affine_threshold', config.get(namespace, {}).get('affine_threshold', None)),
            ('gimbal_projection', config.get(namespace, {}).get('gimbal_projection', None)),
            ('max_map_radius', config.get(namespace, {}).get('max_map_radius', None)),
            ('map_radius_meters_default', config.get(namespace, {}).get('map_radius_meters_default', None)),
            ('update_map_center_threshold', config.get(namespace, {}).get('update_map_center_threshold', None)),
            ('update_map_radius_threshold', config.get(namespace, {}).get('update_map_radius_threshold', None)),
            ('publish_frequency', config.get(namespace, {}).get('publish_frequency', None), ParameterDescriptor(read_only=True))
        ])

    def _setup_superglue(self) -> SuperGlue:
        """Sets up SuperGlue estimator.

        :return: The SuperGlue instance
        """
        superglue_conf = self._config.get(self.name, {}).get('superglue', None)
        assert_type(dict, superglue_conf)
        superglue = SuperGlue(superglue_conf, self.get_logger())
        return superglue

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
        """Checks if map rasters should be retrieved for projected principal point instead of vehicle position.

        :return: True if projected principal point should be used.
        """
        gimbal_projection_flag = self.get_parameter('misc.gimbal_projection').get_parameter_value().bool_value
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
        # TODO: check that abs(camera_pitch) <= 90? (0 is nadir facing)
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
            radius_meters = self.get_parameter('misc.map_radius_meters_default').get_parameter_value().integer_value
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

    def _distances(self, latlon1: Union[LatLon, LatLonAlt], latlon2: Union[LatLon, LatLonAlt]) -> Tuple[float, float]:
        """Calculate distance in meters in x and y dimensions between two points.

        :param latlon1: The first point
        :param latlon2: The second point
        :return: The x and y distances in meters as a tuple: (x-distance, y-distance)
        """
        assert_type(get_args(Union[LatLon, LatLonAlt]), latlon1)
        assert_type(get_args(Union[LatLon, LatLonAlt]), latlon2)
        lats1 = (latlon1.lat, latlon1.lat)
        lons1 = (latlon1.lon, latlon1.lon)
        lats2 = (latlon1.lat, latlon2.lat)  # Lon difference for first, lat difference for second --> y, x
        lons2 = (latlon2.lon, latlon1.lon)
        _, __, dist = self._geod.inv(lons1, lats1, lons2, lats2)

        # invert order to x, y (lat diff, lon diff in meters) in NED frame dimensions,
        # also invert X axis so that it points north
        dist = (-dist[1], dist[0])
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

    def _project_gimbal_fov(self, altitude_meters: float) -> Optional[np.ndarray]:
        """Returns field of view (FOV) BBox projected using gimbal attitude and camera intrinsics information.

        :param altitude_meters: Altitude of camera in meters  # TODO: why is altitude an arg but other info are retrieved inside the function?
        :return: Projected FOV bounding box in pixel coordinates or None if not available
        """
        rpy = self._get_camera_rpy()
        if rpy is None:
            self.get_logger().warn('Could not get RPY - cannot project gimbal fov.')
            return None

        r = Rotation.from_euler(self.EULER_SEQUENCE, list(rpy), degrees=True).as_matrix()
        e = np.hstack((r, np.expand_dims(np.array([0, 0, altitude_meters]), axis=1)))
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

    def _projected_field_of_view_center(self, origin: LatLonAlt) -> Optional[LatLon]:
        """Returns WGS84 coordinates of projected camera field of view (FOV).

        :param origin: Camera position  # TODO: why is this an argument but all else is not?
        :return: Center of the FOV or None if not available
        """
        if self._camera_info is not None:
            gimbal_fov_pix = self._project_gimbal_fov(origin.alt)

            # Convert gimbal field of view from pixels to WGS84 coordinates
            if gimbal_fov_pix is not None:
                azmths = list(map(lambda x: math.degrees(math.atan2(x[0], x[1])), gimbal_fov_pix))
                dists = list(map(lambda x: math.sqrt(x[0] ** 2 + x[1] ** 2), gimbal_fov_pix))
                zipped = list(zip(azmths, dists))
                to_wgs84 = partial(self._move_distance, origin)
                gimbal_fov_wgs84 = np.array(list(map(to_wgs84, zipped)))
                ### TODO: add some sort of assertion hat projected FoV is contained in size and makes sense

                # Use projected field of view center instead of global position as map center
                map_center_latlon = get_bbox_center(fov_to_bbox(gimbal_fov_wgs84))
            else:
                self.get_logger().warn('Could not project camera FoV, getting map raster assuming nadir-facing camera.')
                return None
        else:
            self.get_logger().debug('Camera info not available, cannot project FoV, defaulting to global position.')
            return None

        return map_center_latlon

    def _update_map(self, center: LatLon, radius: Union[int, float]) -> None:
        """Gets latest map from WMS server for given location and radius and saves it.

        :param center: WGS84 coordinates of map to be retrieved
        :param radius: Radius in meters of circle to be enclosed by the map raster
        :return:
        """
        self.get_logger().info(f'Updating map at {center}, radius {radius} meters.')
        assert_type(LatLon, center)
        assert_type(get_args(Union[int, float]), radius)
        max_radius = self.get_parameter('misc.max_map_radius').get_parameter_value().integer_value
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
                self._wms_pool_worker, [(center, radius, bbox, map_size, url, version, layer_str, srs_str, timeout)],
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
    def _wms_pool_worker(center: LatLon, radius: Union[int, float], bbox: BBox, map_size: Tuple[int, int],
                         url: str, version: str, layer_str: str, srs_str: str, timeout: int) -> MapFrame:
        """Gets latest map from WMS server for given location and radius and returns it.

        :param center: Center of the map to be retrieved
        :param radius: Radius in meters of the circle to be enclosed by the map
        :param bbox: Bounding box of the map    # TODO: this seems to be redundant info with the _update_map call, maybe combine?
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
        wms_client = _cached_wms_client(url, version, timeout)
        assert wms_client is not None
        assert_type(BBox, bbox)
        assert(all(isinstance(x, int) for x in map_size))
        assert_type(str, layer_str)
        assert_type(str, srs_str)
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

    def image_raw_callback(self, msg: Image) -> None:
        """Handles latest image frame from camera.

        :param msg: The Image message from the PX4-ROS 2 bridge to decode
        :return:
        """
        self.get_logger().debug('Camera image callback triggered.')
        assert_type(Image, msg)

        cv_image = self._cv_bridge.imgmsg_to_cv2(msg, self.IMAGE_ENCODING)

        img_size = self._declared_img_size()
        if img_size is not None:
            cv_img_shape = cv_image.shape[0:2]
            assert cv_img_shape == img_size, f'Converted cv_image shape {cv_img_shape} did not match declared image ' \
                                             f'shape {img_size}.'

        image_frame = ImageFrame(cv_image, msg.header.frame_id, msg.header.stamp)

        # Check whether we can do matching
        pass_, inputs = self._match_inputs()
        if not pass_:
            self.get_logger().warn(f'_match_inputs check did not pass - skipping image frame matching.')
            return None
        self._match(image_frame, *inputs)

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

        self.get_logger().warn('Assuming stabilized gimbal - ignoring vehicle intrinsic pitch and roll for camera RPY.')
        self.get_logger().warn('Assuming zero roll for camera RPY.')  # TODO remove zero roll assumption

        yaw = heading + gimbal_yaw  # TODO: if over 180, make it negative instead
        assert abs(yaw) <= 360, f'Yaw was unexpectedly large: {abs(yaw)}, max 360 expected.'
        if abs(yaw) > 180:  # Important: >, not >= (because we are using mod 180 operation below)
            yaw = yaw % 180 if yaw < 0 else yaw % -180  # Make the compound yaw between -180 and 180 degrees
        pitch = -(90 + gimbal_euler[pitch_index])  # TODO: ensure abs(pitch) <= 90?
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

        # TODO: this assertion is arbitrary? how to handle unexpected camera normal length?
        # TODO: may have to raise error here - dont know what to do, this assertion could trigger an error
        camera_normal_length = np.linalg.norm(camera_normal)
        assert abs(camera_normal_length - 1) <= 0.001, f'Unexpected camera normal length {camera_normal_length}.'

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
        max_map_radius = self.get_parameter('misc.max_map_radius').get_parameter_value().integer_value
        map_radius = 3*altitude
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
        self._vehicle_global_position = msg  # TODO: seems like self.vehicle_global_position property is never accessed currently, info only needed here to trigger _update_map? Comment out this variabhle completely?
        center = LatLon(msg.lat, msg.lon)
        origin = LatLonAlt(*(center + (msg.alt,)))
        if self._use_gimbal_projection():
            projected_center = self._projected_field_of_view_center(origin)
            if projected_center is None:
                self.get_logger().warn('Could not project field of view center. Using global position for map instead.')
            else:
                center = projected_center
        map_radius = self._get_dynamic_map_radius(msg.alt)
        if self._should_update_map(center, map_radius):
            self._update_map(center, map_radius)
        else:
            self.get_logger().debug('Map center and radius not changed enough to update map yet, '
                                    'or previous results are not ready.')

    def _should_update_map(self, center: Union[LatLon, LatLonAlt], radius: Union[int, float]) -> bool:
        """Checks if a new WMS map request should be made to update old map.

        :param center: WGS84 coordinates of new map candidate center
        :param radius: Radius in meters of new map candidate
        :return: True if map should be updated
        """
        assert_type(get_args(Union[int, float]), radius)
        assert_type(get_args(Union[LatLon, LatLonAlt]), center)
        if self._previous_map_frame is not None:
            if not (abs(self._distance(center, self._previous_map_frame.center)) >
                    self.get_parameter('misc.update_map_center_threshold').get_parameter_value().integer_value or
                    abs(radius - self._previous_map_frame.radius) >
                    self.get_parameter('misc.update_map_center_threshold').get_parameter_value().integer_value):
                return False
        # No map yet, check whether old request is still processing
        if self._wms_results is not None:
            if not self._wms_results.ready():
                # Previous request still running
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

    def _create_vehicle_visual_odometry_msg(self, timestamp: int, position: tuple, velocity: tuple, rotation: tuple) \
            -> None:
        """Publishes a VehicleVisualOdometry message over the microRTPS bridge.

        See https://docs.px4.io/v1.12/en/advanced_config/tuning_the_ecl_ekf.html#external-vision-system for supported
        EKF2_AID_MASK values when using an external vision system.

        :param timestamp: Timestamp to be included in the outgoing message
        :param position: Position tuple (x, y, z) to be published
        :param velocity: Velocity tuple (vx, vy, vz) to be published
        :param rotation: Rotation quaternion to be published
        :return:
        """
        assert VehicleVisualOdometry is not None, 'VehicleVisualOdometry definition not found (was None).'
        msg = VehicleVisualOdometry()

        # TODO: could throw a warning if position and velocity BOTH are None - would publish a message full of NaN

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

        # Attitude quaternions - not used
        assert msg.local_frame is self.LOCAL_FRAME_NED  # TODO: this needed?
        msg.q = rotation  # (float('nan'),) * 4  # float32  # TODO: need vehicle yaw against NED frame here, need to assert self.LOCAL_FRAME_NED is used
        msg.q_offset = (0.0, ) * 4  # (float('nan'),) * 4      # TODO: make this zero and assert that self.LOCAL_FRAME_NED is used
        msg.pose_covariance = (float('nan'),) * 21

        # Velocity frame of reference
        msg.velocity_frame = self.LOCAL_FRAME_NED  # uint8

        # Velocity
        if velocity is not None:
            assert len(velocity) == 3, f'Unexpected length for velocity estimate: {len(velocity)} (3 expected).'
            assert all(isinstance(x, float) for x in velocity), f'Velocity contained non-float elements.'
            msg.vx, msg.vy, msg.vz = velocity  # float32 North, East, Down
        else:
            self.get_logger().warn('Velocity tuple was None - publishing NaN as velocity.')
            msg.vx, msg.vy, msg.vz = (float('nan'),) * 3  # float32 North, East, Down

        # Angular velocity - not used
        msg.rollspeed, msg.pitchspeed, msg.yawspeed = (float('nan'),) * 3  # float32
        msg.velocity_covariance = (float('nan'),) * 21  # float32 North, East, Down

        self.get_logger().debug(f'Setting outgoing vehicle visual odometry message as:\n{msg}.')
        self._vehicle_visual_odometry = msg
        #self.get_logger().debug(f'Publishing vehicle visual odometry message:\n{msg}.')
        #self._topics.get(self.PUBLISH_KEY).get(self.VEHICLE_VISUAL_ODOMETRY_TOPIC_NAME).publish(msg)

    def _camera_pitch(self) -> Optional[int]:  # TODO: float?
        """Returns camera pitch in degrees relative to vehicle frame.

        :return: Camera pitch in degrees, or None if not available
        """
        rpy = self._get_camera_rpy()
        if rpy is None:
            self.get_logger().warn('Gimbal RPY not available, cannot compute camera pitch.')
            return None
        assert_type(RPY, rpy)
        return rpy.pitch

    # TODO: just use one?
    def _gimbal_attitude(self) -> Optional[Union[GimbalDeviceAttitudeStatus, GimbalDeviceSetAttitude]]:
        """Returns 1. GimbalDeviceAttitudeStatus, or 2. GimbalDeviceSetAttitude if 1. is not available.

        :return: GimbalDeviceAttitudeStatus or GimbalDeviceSetAttitude message
        """
        gimbal_attitude = self._gimbal_device_attitude_status
        if gimbal_attitude is None:
            self.get_logger().warn('GimbalDeviceAttitudeStatus not available. Trying GimbalDeviceSetAttitude instead.')
            gimbal_attitude = self._gimbal_device_set_attitude
            if gimbal_attitude is None:
                self.get_logger().warn('GimbalDeviceSetAttitude not available. Gimbal attitude status not available.')
        return gimbal_attitude

    @staticmethod
    def _find_and_decompose_homography(mkp_img: np.ndarray, mkp_map: np.ndarray, k: np.ndarray,
                                       camera_normal: np.ndarray, reproj_threshold: float = 1.0, affine: bool = False) \
            -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Processes matching keypoints from img and map and returns homography matrix, mask, translation and rotation.

        :param mkp_img: Matching keypoints from image
        :param mkp_map: Matching keypoints from map
        :param k: Camera intrinsics matrix
        :param camera_normal: Camera normal unit vector
        :param reproj_threshold: RANSAC reprojection threshold parameter
        :param affine: Flag indicating whether homography should be restricted to 2D affine transformation
        :return: Tuple containing homography matrix, mask, translation and rotation
        """
        min_points = 4
        assert_type(np.ndarray, mkp_img)
        assert_type(np.ndarray, mkp_map)
        assert len(mkp_img) >= min_points and len(mkp_map) >= min_points, 'Four points needed to estimate homography.'

        assert_type(bool, affine)
        assert_type(float, reproj_threshold)
        if not affine:
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

    def _local_frame_position(self, local_frame_origin: LatLonAlt, camera_position: LatLon,
                              camera_altitude: Union[int, float]) -> Tuple[float, float, float]:
        """Returns camera position tuple (x, y) in meters in local frame.

        :param local_frame_origin: WGS84 coordinates of local frame origin  #TODO: can also be LonLat, not just LonLatAlt?
        :param camera_position: WGS84 coordinates of camera position
        :param camera_altitude: Camera altitude in meters
        :return:
        """
        assert_type(LatLonAlt, local_frame_origin)
        assert_type(LatLon, camera_position)
        assert_type(get_args(Union[int, float]), camera_altitude)
        return self._distances(local_frame_origin, camera_position) + (camera_altitude,)

    # TODO: why is this called local?
    def _local_frame_velocity(self, image_frame: ImageFrame, previous_image_frame: ImageFrame)\
            -> Tuple[float, float, Optional[float]]:
        """Computes velocity in meters per second for position between two image frames.

        :param image_frame: Latest image frame
        :param previous_image_frame: Previous image frame
        :return: Tuple containing x, y, and z (None if not available) axis velocities in meters per second
        """
        assert_type(ImageFrame, image_frame)
        assert_type(ImageFrame, previous_image_frame)
        assert previous_image_frame.position is not None, f'Previous camera position was unexpectedly None.'  # TODO: is it possible that this is None? Need to do warning instead of assert?
        assert image_frame.position is not None, f'Current camera position was unexpectedly None.'  # TODO: is it possible that this is None? Need to do warning instead of assert?
        assert_first_stamp_greater(image_frame.stamp, previous_image_frame.stamp)
        time_difference = image_frame.stamp.sec - previous_image_frame.stamp.sec
        if time_difference == 0:
            time_difference = (image_frame.stamp.nanosec - previous_image_frame.stamp.nanosec) / 1e9
        assert time_difference > 0, f'Time difference between frames was 0.'
        x_dist, y_dist = self._distances(image_frame.position, previous_image_frame.position)  # TODO: compute x,y,z components separately!
        z_dist = image_frame.position.alt - previous_image_frame.position.alt
        dist = (x_dist, y_dist, z_dist)
        assert all(isinstance(x, float) for x in
                   dist), f'Expected all float values for distance: {dist}.'  # TODO: z could be None/NaN - handle it!
        velocity = tuple(x / time_difference for x in dist)
        return velocity[0], velocity[1], velocity[2]  # Do this way to get rid of warning

    def _match_inputs(self) -> Tuple[bool, Tuple[np.ndarray, LatLonAlt, int, CameraInfo, np.ndarray, float, float, Dim,
                                                 Dim, bool, Optional[np.ndarray]]]:
        """Performs a check that all required data is available for performing a _match, and returns the input data.

        Returns (success, data) where success is False if there are any Nones in the data tuple.

        Data consists of:
            map_frame - np.darray map_frame to match
            local_frame_origin_position - LatLonAlt origin of local frame global frame WGS84
            timestamp - Local position message timestamp (to sync vehicle visual odom messages)
            camera_info - CameraInfo
            camera_normal - np.ndarray Camera normal unit vector
            camera_yaw - float  # TODO: degrees? If so, accept int also
            camera_pitch - float  # TODO: degrees? If so, accept int also
            map_dim_with_padding - Dim map dimensions including padding for rotation
            img_dim - Dim image dimensions
            restrict_affine - bool flag indicating whether homography matrix should be restricted to 2D affine tform
            previous_image_frame - Optional[np.ndarray], previous image frame, if available, None otherwise

        :return: Tuple containing success flag and data mentioned in the description
        """
        # Vehicle local frame global reference position
        local_frame_origin_position, timestamp = self._vehicle_local_position_ref_latlonalt_timestamp()  # TODO: also get timestamp?

        # Camera information
        camera_normal, camera_yaw, camera_pitch = self._camera_normal(), self._camera_yaw(), self._camera_pitch()

        # Image and map raster dimensions
        map_dim_with_padding, img_dim = self._map_dim_with_padding(), self._img_dim()

        # Should homography be restricted to 2D affine transformation
        restrict_affine = self._restrict_affine()

        # Make sure all info is available before attempting to match
        required_info = (self._map_frame, local_frame_origin_position, timestamp, self._camera_info,
                         camera_normal, camera_yaw, camera_pitch, map_dim_with_padding, img_dim, restrict_affine)
        optional_info = (self._previous_image_frame, )

        if local_frame_origin_position is None or timestamp is None:  # TODO: handle this better!
            return False, required_info + optional_info

        if not all(x is not None for x in required_info):
            self.get_logger().warn(f'At least one of following was None: {required_info}. Cannot do matching.')
            return False, required_info + optional_info
        else:
            assert -180 <= camera_yaw <= 180, f'Unexpected gimbal yaw value: {camera_yaw} ([-180, 180] expected).'
            return True, required_info + optional_info

    @staticmethod
    def _compute_camera_position(t: np.ndarray, map_dim_with_padding: Dim, bbox: BBox, camera_yaw: float, img_dim: Dim)\
            -> LatLon:  # TODO: Yaw is degrees or radians? If degrees, could also accept ints? Seems like radians so maybe only float is justified unless more refactoring is done
        """Returns camera position based on translation vector and metadata.

        :param t: Camera translation vector
        :param map_dim_with_padding: Map dimensions including padding for likely rotation
        :param bbox: Map bounding box
        :param camera_yaw: Camera yaw in radians
        :param img_dim: Image dimensions
        :return: WGS84 coordinates of camera
        """
        # Convert translation vector to WGS84 coordinates
        # Translate relative to top left corner, not principal point/center of map raster
        t[0] = (1 - t[0]) * img_dim.width / 2
        t[1] = (1 - t[1]) * img_dim.height / 2
        # TODO: break this func into an array and single version?
        cam_pos_wgs84, cam_pos_wgs84_uncropped, cam_pos_wgs84_unrotated = convert_fov_from_pix_to_wgs84(
            np.array(t[0:2].reshape((1, 1, 2))), map_dim_with_padding, bbox, camera_yaw, img_dim)
        cam_pos_wgs84 = cam_pos_wgs84.squeeze()  # TODO: eliminate need for this squeeze
        latlon = LatLon(*tuple(cam_pos_wgs84))
        return latlon

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

    # TODO Current tasks for _match too many:
    # 1. attach fov and position to image_frame
    # 2. Compute and publish position and velocity,
    # 3. Visualize homography,
    def _match(self, image_frame: ImageFrame, map_frame: MapFrame, local_frame_origin_position: LatLonAlt,
               local_position_timestamp: int,
               camera_info: CameraInfo, camera_normal: np.ndarray, camera_yaw: float, camera_pitch: float,
               map_dim_with_padding: Dim, img_dim: Dim, restrict_affine: bool,
               previous_image_frame: Optional[ImageFrame]) -> None:
        """Matches camera image to map image and computes camera position and field of view.

        :param image_frame: The image frame to match
        :param map_frame: The map frame against which to match the image
        :param local_frame_origin_position: WGS84 coordinates of local frame origin
        :param local_position_timestamp: Timestamp of latest VehicleLocalPosition message  # TODO: use some better way to sync timestamps
        :param camera_info: Latest CameraInfo message
        :param camera_normal: Camera normal unit vector
        :param camera_yaw: Camera yaw in degrees
        :param camera_pitch: Camera pitch in degrees
        :param map_dim_with_padding: Map dimensions including padding for rotations
        :param img_dim: Image dimensions
        :param restrict_affine: Flag indicating whether homography should be restricted to a 2D transformation
        :param previous_image_frame: Previous image frame
        :return:
        """
        try:
            self.get_logger().debug(f'Matching image with timestamp {image_frame.stamp} to map.')
            camera_yaw = math.radians(camera_yaw)

            # Get cropped and rotated map
            map_cropped = rotate_and_crop_map(map_frame.image, camera_yaw, img_dim)

            # Get matched keypoints and check that they seem valid
            mkp_img, mkp_map = self._superglue.match(image_frame.image, map_cropped)
            assert_len(mkp_img, len(mkp_map))
            if len(mkp_img) < self.MINIMUM_MATCHES:
                self.get_logger().warn(f'Found {len(mkp_img)} matches, {self.MINIMUM_MATCHES} required. Skip frame.')
                return None

            # Find and decompose homography matrix, do some sanity checks
            k = camera_info.k.reshape([3, 3])
            h, h_mask, t, r = self._find_and_decompose_homography(mkp_img, mkp_map, k, camera_normal,
                                                                  affine=restrict_affine)
            assert_shape(h, (3, 3))
            assert_shape(t, (3, 1))
            assert_shape(r, (3, 3))

            # This block 1. computes fov in WGS84 and attaches it to image_frame, and 3. visualizes homography
            # Convert pixel field of view into WGS84 coordinates, save it to the image frame, visualize the pixels
            fov_pix = get_fov(image_frame.image, h)
            visualize_homography('Matches and FoV', image_frame.image, map_cropped, mkp_img, mkp_map, fov_pix)
            fov_wgs84, fov_uncropped, fov_unrotated = convert_fov_from_pix_to_wgs84(
                fov_pix, map_dim_with_padding, map_frame.bbox, camera_yaw, img_dim)
            image_frame.fov = fov_wgs84

            # Compute camera altitude, and distance to principal point using triangle similarity
            # TODO: _update_map or _project_gimbal_fov_center has similar logic used in gimbal fov projection, try to combine
            camera_distance = self._compute_camera_distance(fov_wgs84, k[0][0], img_dim)
            camera_altitude = self._compute_camera_altitude(camera_distance, camera_pitch)
            self.get_logger().debug(f'Computed camera distance {camera_distance}, altitude {camera_altitude}.')

            position = self._compute_camera_position(t, map_dim_with_padding, map_frame.bbox, camera_yaw, img_dim)
            local_position = self._local_frame_position(local_frame_origin_position, position, camera_altitude)
            image_frame.position = LatLonAlt(*(position + (camera_altitude,)))  # TODO: alt should not be None? Use LatLon instead?  # TODO: move to _compute_camera_position?

            # Yaw against ned frame (quaternion)
            rotation = [0, 0, 0]
            rotation[self._yaw_index()] = camera_yaw
            rotation = tuple(Rotation.from_euler(self.EULER_SEQUENCE, rotation, degrees=True).as_quat())
            assert_len(rotation, 4)

            velocity = None
            if previous_image_frame is not None:
                velocity = self._local_frame_velocity(image_frame, previous_image_frame)
            else:
                self.get_logger().warning(f'Could not get previous image frame stamp - will not compute velocity.')

            self.get_logger().debug(f'Local frame position: {local_position}, velocity: {velocity}.')
            self.get_logger().debug(f'Local frame origin: {local_frame_origin_position}.')
            self._create_vehicle_visual_odometry_msg(local_position_timestamp, local_position, velocity, rotation)

            self._previous_image_frame = image_frame

        except Exception as e:
            self.get_logger().error('Matching returned exception: {}\n{}'.format(e, traceback.print_exc()))

    def terminate_wms_pool(self):
        """Terminates the WMS Pool.

        :return:
        """
        if self._wms_pool is not None:
            self.get_logger().info('Terminating WMS pool.')
            self._wms_pool.terminate()

    def destroy_publish_timer(self):
        """Destroys the vehicle visual odometry timer.

        :return:
        """
        if self._timer is not None:
            self.get_logger().info('Destroying publish timer.')
            assert_type(rclpy.timer.Timer, self._timer)
            self._timer.destroy()


def main(args=None):
    """Starts and terminates the ROS 2 node.

    Also starts cProfile profiling in debugging mode.

    :param args: Any args for initializing the rclpy node
    :return:
    """
    if __debug__:
        pr = cProfile.Profile()
        pr.enable()
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
        matcher.destroy_publish_timer()
        matcher.terminate_wms_pool()
        matcher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
