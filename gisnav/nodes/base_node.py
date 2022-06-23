"""Module that contains the BaseNode ROS 2 node."""
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

from abc import ABC, abstractmethod
from multiprocessing.pool import Pool
#from multiprocessing.pool import ThreadPool as Pool  # Rename 'Pool' to keep same interface
from typing import Optional, Union, Tuple, get_args, List
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition, VehicleGlobalPosition, GimbalDeviceAttitudeStatus, \
    GimbalDeviceSetAttitude
from sensor_msgs.msg import CameraInfo, Image

# TODO: for data at least may be cleaner to just import the module and use it as prefix?
#  almost everything is imported from data (except for abstract base classes)
from gisnav.data import Dim, TimePair, ImageData, MapData, CameraData, Attitude, DataValueError, \
    InputData, OutputData, ImagePair, AsyncPoseQuery, AsyncWMSQuery, ContextualMapData, FixedCamera, Img, Pose, Position
from gisnav.geo import GeoPoint, GeoSquare, GeoTrapezoid
from gisnav.assertions import assert_type, assert_len
from gisnav.pose_estimators.pose_estimator import PoseEstimator
from gisnav.wms import WMSClient
from gisnav.kalman import SimpleFilter


class BaseNode(Node, ABC):
    """ROS 2 node that publishes position estimate based on visual match of drone video to map of same location"""
    # Encoding of input video (input to CvBridge)
    # e.g. gscam2 only supports bgr8 so this is used to override encoding in image header
    _IMAGE_ENCODING = 'bgr8'

    # Keys for topics dictionary that map microRTPS bridge topics to subscribers and message definitions
    _TOPICS_MSG_KEY = 'message'
    _TOPICS_SUBSCRIBER_KEY = 'subscriber'

    # Process counts for multiprocessing pools
    _WMS_PROCESS_COUNT = 1      # should be 1
    _MATCHER_PROCESS_COUNT = 1  # should be 1

    # region ROS Parameter Defaults
    ROS_D_WMS_URL = 'http://localhost:8080/wms'
    """Default WMS server endpoint URL"""

    ROS_D_WMS_VERSION = '1.1.1'
    """Default WMS server version"""

    ROS_D_WMS_LAYERS = ['Imagery']
    """Default WMS request layers parameter

    .. note::
        The combined layers should cover the flight area of the vehicle at high resolution. Typically this list would 
        have just one layer for high resolution aerial or satellite imagery.
    """

    ROS_D_WMS_STYLES = ['']
    """Default WMS request styles parameter
    
    Must be same length as :py:attr:`.ROS_D_WMS_LAYERS`. Use empty strings for server default styles.
    """

    ROS_D_WMS_SRS = 'EPSG:4326'
    """Default WMS request SRS parameter"""

    ROS_D_WMS_REQUEST_TIMEOUT = 10
    """Default WMS client request timeout in seconds"""

    ROS_D_WMS_IMAGE_FORMAT = 'image/jpeg'
    """Default WMS client request image format"""

    ROS_D_MISC_MAX_PITCH = 30
    """Default maximum camera pitch from nadir in degrees for attempting to estimate pose against reference map

    .. seealso::
        :py:attr:`.ROS_D_MAP_UPDATE_MAX_PITCH` 
        :py:attr:`.ROS_D_MAP_UPDATE_GIMBAL_PROJECTION`
    """

    ROS_D_MISC_VARIANCE_ESTIMATION_LENGTH = 20
    """Default observation series length for smoothing and estimating the variance of the position estimate"""

    ROS_D_MISC_MIN_MATCH_ALTITUDE = 80
    """Default minimum ground altitude in meters under which matches against map will not be attempted"""

    ROS_D_MISC_BLUR_THRESHOLD = 2
    """Default threshold standard deviation for Laplacian image blur detector"""

    ROS_D_MISC_BLUR_WINDOW_LENGTH = 20
    """Default window length for rolling image blur filtering"""

    ROS_D_MAP_UPDATE_UPDATE_DELAY = 1
    """Default delay in seconds for throttling WMS GetMap requests

    When the camera is mounted on a gimbal and is not static, this delay should be set quite low to ensure that whenever
    camera field of view is moved to some other location, the map update request will follow very soon after. The field
    of view of the camera projected on ground generally moves *much faster* than the vehicle itself.
    
    .. note::
        This parameter provides a hard upper limit for WMS GetMap request frequency. Even if this parameter is set low, 
        WMS GetMap requests will likely be much less frequent because they will throttled by the conditions set in  
        :meth:`._should_update_map` (private method - see source code for reference).
    """

    ROS_D_MAP_UPDATE_GIMBAL_PROJECTION = True
    """Default flag to enable map updates based on expected center of field of view (FOV) projected onto ground

    When this flag is enabled, map rasters are retrieved for the expected center of the camera FOV instead of the
    expected position of the vehicle, which increases the chances that the FOV is fully contained in the map raster.
    This again increases the chances of getting a good pose estimate.
    
    .. seealso::
        :py:attr:`.ROS_D_MISC_MAX_PITCH`
        :py:attr:`.ROS_D_MAP_UPDATE_MAX_PITCH`
    """

    ROS_D_MAP_UPDATE_MAX_PITCH = 30
    """Default maximum camera pitch from nadir in degrees for attempting to update the stored map

    This limit only applies when camera field of view (FOV) projection is enabled. This value will prevent unnecessary 
    WMS GetMap requests when the camera is looking far into the horizon and it would be unrealistic to get a good pose 
    estimate against a map.

    .. seealso::
        :py:attr:`.ROS_D_MISC_MAX_PITCH`
        :py:attr:`.ROS_D_MAP_UPDATE_GIMBAL_PROJECTION`
    """

    ROS_D_MAP_UPDATE_MAX_MAP_WIDTH = 400
    """Default maximum map width in meters for WMS GetMap request bounding boxes

    This limit prevents unintentionally requesting very large maps if camera field of view is projected far into the
    horizon. This may happen e.g. if :py:attr:`.ROS_D_MAP_UPDATE_MAX_PITCH` is set too high relative to the camera's
    vertical view angle. If the WMS server back-end needs to e.g. piece the large map together from multiple files the 
    request might timeout in any case.
    """

    ROS_D_MAP_UPDATE_UPDATE_MAP_AREA_THRESHOLD = 0.85
    """Default map bounding box area intersection threshold that prevents a new map from being retrieved

    This prevents unnecessary WMS GetMap requests to replace an old map with a new map that covers almost the same area.
    """

    ROS_D_POSE_ESTIMATOR_CLASS = 'gisnav.pose_estimators.loftr.LoFTRPoseEstimator'
    """Default :class:`.PoseEstimator` to use for estimating pose camera pose against reference map"""

    ROS_D_POSE_ESTIMATOR_PARAMS_FILE = 'loftr_params.yml'
    """Default parameter file with args for the default :class:`.PoseEstimator`'s :meth:`.PoseEstimator.initializer`"""

    ROS_D_DEBUG_EXPORT_POSITION = '' # 'position.json'
    """Default filename for exporting GeoJSON containing estimated field of view and position

    Set to '' to disable
    """

    ROS_D_DEBUG_EXPORT_PROJECTION = '' # 'projection.json'
    """Default filename for exporting GeoJSON containing projected field of view (FOV) and FOV center
        
    Set to '' to disable
    """
    # endregion

    def __init__(self, name: str, package_share_dir: str) -> None:
        """Initializes the ROS 2 node.

        :param name: Name of the node
        :param package_share_dir: Package share directory file path
        """
        assert_type(name, str)
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.__declare_ros_params()

        self._package_share_dir = package_share_dir

        # WMS client and requests in a separate process
        self._wms_query = None  # Must check for None when using this
        url = self.get_parameter('wms.url').get_parameter_value().string_value
        version = self.get_parameter('wms.version').get_parameter_value().string_value
        timeout = self.get_parameter('wms.request_timeout').get_parameter_value().integer_value
        assert_type(url, str)
        assert_type(version, str)
        assert_type(timeout, int)
        self._wms_pool = Pool(self._WMS_PROCESS_COUNT, initializer=WMSClient.initializer,
                              initargs=(url, version, timeout))

        # Setup map update timer
        self._map_update_timer = self._setup_map_update_timer()

        # Dict for storing all microRTPS bridge subscribers
        self._topics = {
            'VehicleAttitude_PubSubTopic': {self._TOPICS_MSG_KEY: VehicleAttitude},
            'VehicleLocalPosition_PubSubTopic': {self._TOPICS_MSG_KEY: VehicleLocalPosition},
            'VehicleGlobalPosition_PubSubTopic': {self._TOPICS_MSG_KEY: VehicleGlobalPosition},
            'GimbalDeviceAttitudeStatus_PubSubTopic': {self._TOPICS_MSG_KEY: GimbalDeviceAttitudeStatus},
            'GimbalDeviceSetAttitude_PubSubTopic': {self._TOPICS_MSG_KEY: GimbalDeviceSetAttitude},
            'camera_info': {self._TOPICS_MSG_KEY: CameraInfo},
            'image_raw': {self._TOPICS_MSG_KEY: Image},
        }
        self._setup_subscribers()

        # Converts image_raw to cv2 compatible image
        self._cv_bridge = CvBridge()

        # Setup map matching pool
        pose_estimator_params_file = self.get_parameter('pose_estimator.params_file').get_parameter_value().string_value
        self._pose_estimator, self._pose_estimator_pool = self._setup_pose_estimation_pool(pose_estimator_params_file)
        self._pose_estimation_query = None  # Must check for None when using this

        # Stored blur values for blur detection
        self._blurs = None

        # Kalman filter (initialized once enough measurements available)
        window_length = self.get_parameter('misc.variance_estimation_length').get_parameter_value().integer_value
        self._kf = SimpleFilter(window_length)

        # Must check for None when using these
        self._map_input_data = None
        self._map_input_data_prev = None
        self._map_output_data_prev = None

        # self._image_data = None  # Not currently used / needed
        self._map_data = None

        # Stored solution for the PnP problem (map matching and visual odometry separately)
        self._pose_guess = None

        self._time_sync = None  # For storing local and foreign (EKF2) timestamps

        self._camera_data = None

        # Properties that are mapped to microRTPS bridge topics, must check for None when using them
        #self._camera_info = None  # Not used, CameraInfo subscription is destroyed once self._camera_data is assigned
        self._vehicle_local_position = None
        self._vehicle_global_position = None  # TODO: save directly as 'Position' class? Grab z_ground from local position?
        self._vehicle_attitude = None
        self._gimbal_device_attitude_status = None
        self._gimbal_device_set_attitude = None

    # region Properties
    @property
    def _package_share_dir(self) -> str:
        """ROS 2 package share directory"""
        return self.__package_share_dir

    @_package_share_dir.setter
    def _package_share_dir(self, value: str) -> None:
        assert_type(value, str)
        self.__package_share_dir = value

    @property
    def _kf(self) -> SimpleFilter:
        """Kalman filter for improved position and variance estimation"""
        return self.__kf

    @_kf.setter
    def _kf(self, value: SimpleFilter) -> None:
        assert_type(value, SimpleFilter)
        self.__kf = value

    @property
    def _blurs(self) -> Optional[np.ndarray]:
        """Array of image blur values for filtering images based on blur."""
        return self.__blurs

    @_blurs.setter
    def _blurs(self, value: Optional[np.ndarray]) -> None:
        assert_type(value, get_args(Optional[np.ndarray]))
        self.__blurs = value

    @property
    def _pose_guess(self) -> Optional[Pose]:
        """Stores rotation and translation vectors for use by :func:`cv2.solvePnPRansac`.

        Assumes previous solution to the PnP problem will be close to the new solution.
        """
        return self.__pose_guess

    @_pose_guess.setter
    def _pose_guess(self, value: Optional[Pose]) -> None:
        assert_type(value, get_args(Optional[Pose]))
        self.__pose_guess = value

    @property
    def _pose_estimator(self) -> PoseEstimator:
        """Dynamically loaded :class:`.PoseEstimator`"""
        return self.__pose_estimator

    @_pose_estimator.setter
    def _pose_estimator(self, value: Optional[PoseEstimator]) -> None:
        assert issubclass(value, get_args(Optional[PoseEstimator]))
        self.__pose_estimator = value

    @property
    def _time_sync(self) -> Optional[TimePair]:
        """A :class:`gisnav.data.TimePair` with local and foreign (EKF2) timestamps in microseconds

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
    def _wms_query(self) -> Optional[AsyncWMSQuery]:
        """Asynchronous results from a WMS client request."""
        return self.__wms_results

    @_wms_query.setter
    def _wms_query(self, value: Optional[AsyncWMSQuery]) -> None:
        assert_type(value, get_args(Optional[AsyncWMSQuery]))
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
    def _pose_estimator_pool(self) -> Pool:
        """Pool for running a :class:`.PoseEstimator` in dedicated thread (or process)"""
        return self.__pose_estimator_pool

    @_pose_estimator_pool.setter
    def _pose_estimator_pool(self, value: Pool) -> None:
        assert_type(value, Pool)
        self.__pose_estimator_pool = value

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
    def _pose_estimation_query(self) -> Optional[AsyncPoseQuery]:
        """Asynchronous results and input from a pose estimation thread or process."""
        return self.__pose_estimation_query

    @_pose_estimation_query.setter
    def _pose_estimation_query(self, value: Optional[AsyncPoseQuery]) -> None:
        assert_type(value, get_args(Optional[AsyncPoseQuery]))
        self.__pose_estimation_query = value

    @property
    def _topics(self) -> dict:
        """Dictionary that stores all rclpy publishers and subscribers."""
        return self.__topics

    @_topics.setter
    def _topics(self, value: dict) -> None:
        assert_type(value, dict)
        self.__topics = value

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
    def _camera_data(self) -> Optional[CameraData]:
        """CameraInfo received via the PX4-ROS 2 bridge."""
        return self.__camera_data

    @_camera_data.setter
    def _camera_data(self, value: Optional[CameraData]) -> None:
        assert_type(value, get_args(Optional[CameraData]))
        self.__camera_data = value

    @property
    def _vehicle_local_position(self) -> Optional[VehicleLocalPosition]:
        """VehicleLocalPosition received via the PX4-ROS 2 bridge."""
        return self.__vehicle_local_position

    @_vehicle_local_position.setter
    def _vehicle_local_position(self, value: Optional[VehicleLocalPosition]) -> None:
        assert_type(value, get_args(Optional[VehicleLocalPosition]))
        self.__vehicle_local_position = value

    @property
    def _vehicle_attitude(self) -> Optional[VehicleAttitude]:
        """VehicleAttitude received via the PX4-ROS 2 bridge."""
        return self.__vehicle_attitude

    @_vehicle_attitude.setter
    def _vehicle_attitude(self, value: Optional[VehicleAttitude]) -> None:
        assert_type(value, get_args(Optional[VehicleAttitude]))
        self.__vehicle_attitude = value

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
    # endregion

    # region Computed Properties
    @property
    def _wms_results_pending(self) -> bool:
        """True if there are pending results"""
        return not self._wms_query.result.ready() if self._wms_query is not None else False

    @property
    def _synchronized_time(self) -> Optional[int]:
        """Estimated EKF2 system reference time in microseconds or None if not available

        .. seealso:
            :py:attr:`._time_sync` and :meth:`._sync_timestamps`
        """
        if self._time_sync is None:
            self.get_logger().warn('Could not estimate EKF2 timestamp.')
            return None
        else:
            now_usec = time.time() * 1e6
            assert now_usec > self._time_sync.local, f'Current timestamp {now_usec} was unexpectedly smaller than '\
                                                     f'timestamp stored earlier for synchronization '\
                                                     f'{self._time_sync.local}.'
            ekf2_timestamp_usec = int(self._time_sync.foreign + (now_usec - self._time_sync.local))
            return ekf2_timestamp_usec

    @property
    def _is_gimbal_projection_enabled(self) -> bool:
        """True if map rasters should be retrieved for projected field of view instead of vehicle position

        If this is set to false, map rasters are retrieved for the vehicle's global position instead. This is typically
        fine as long as the camera is not aimed too far in to the horizon and has a relatively wide field of view. For
        best results, this should be on to ensure the field of view is fully contained within the area of the retrieved
        map raster.

        .. note::
            If you know your camera will be nadir-facing, disabling ``map_update.gimbal_projection`` may improve
            performance by a small amount.
        """
        gimbal_projection_flag = self.get_parameter('map_update.gimbal_projection').get_parameter_value().bool_value
        if type(gimbal_projection_flag) is bool:
            return gimbal_projection_flag
        else:
            # Default behavior (safer)
            self.get_logger().warn(f'Could not read gimbal projection flag: {gimbal_projection_flag}. Assume False.')
            return False

    @property
    def _gimbal_set_attitude(self) -> Optional[Attitude]:
        """Gimbal set attitude in NED frame or None if not available

        Gimbal set attitude is given in FRD frame. Combines it with vehicle yaw from VehicleAttitude to construct a
        gimbal set attitude in NED frame.

        .. note::
            This is only the set attitude, actual gimbal attitude may differ if gimbal has not yet stabilized.
        """
        if self._vehicle_attitude is None or self._gimbal_device_set_attitude is None:
            self.get_logger().debug('Vehicle or gimbal set attitude not yet available.')
            return None

        yaw_mask = np.array([1, 0, 0, 1])  # Gimbal roll & pitch is stabilized so we only need vehicle yaw (heading)
        vehicle_yaw = self._vehicle_attitude.q * yaw_mask

        gimbal_set_attitude_frd = self._gimbal_device_set_attitude.q

        # SciPy expects (x, y, z, w) while PX4 messages are (w, x, y, z)
        vehicle_yaw = Rotation.from_quat(np.append(vehicle_yaw[1:], vehicle_yaw[0]))
        gimbal_set_attitude_frd = Rotation.from_quat(np.append(gimbal_set_attitude_frd[1:], gimbal_set_attitude_frd[0]))

        gimbal_set_attitude_ned = vehicle_yaw * gimbal_set_attitude_frd

        return Attitude(gimbal_set_attitude_ned.as_quat())

    @property
    def _altitude_agl(self) -> Optional[float]:
        """Altitude above ground level in meters, or None if not available

        This property tries to use the 'dist_bottom' value from class:`px4_msgs.VehicleLocalPosition` first, and 'z'
        second. If neither are valid, the property value is 'None'.
        """
        if self._vehicle_local_position is not None:
            if self._vehicle_local_position.dist_bottom_valid:
                self.get_logger().debug('Using VehicleLocalPosition.dist_bottom for altitude AGL.')
                return abs(self._vehicle_local_position.dist_bottom)
            elif self._vehicle_local_position.z_valid:
                self.get_logger().warn('VehicleLocalPosition.dist_bottom was not valid, assuming '
                                       'VehicleLocalPosition.z for altitude AGL.')
                return abs(self._vehicle_local_position.z)
            else:
                return None
        else:
            self.get_logger().warn('Altitude AGL not yet available.')
            return None

    @property
    def _ground_elevation_amsl(self) -> Optional[float]:
        """Ground elevation in meters above mean sea level (AMSL) or None if information is not available

        It is assumed to be same as :class:`px4_msgs.VehicleLocalPosition` reference altitude.

        .. seealso::
            :py:attr:`._altitude_agl`
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

    @property
    def _map_size_with_padding(self) -> Optional[Tuple[int, int]]:
        """Padded map size tuple (height, width) or None if the information is not available.

        Because the deep learning models used for predicting matching keypoints or poses between camera image frames
        and map rasters are not assumed to be rotation invariant in general, the map rasters are rotated based on
        camera yaw so that they align with the camera images. To keep the scale of the map after rotation the same,
        black corners would appear unless padding is used. Retrieved maps therefore have to be squares with the side
        lengths matching the diagonal of the camera frames so that scale is preserved and no black corners appear in
        the map rasters after rotation. The height and width will both be equal to the diagonal of the declared
        (:py:attr:`._camera_data`) camera frame dimensions.
        """
        if self._img_dim is None:
            self.get_logger().warn(f'Dimensions not available - returning None as map size.')
            return None
        diagonal = math.ceil(math.sqrt(self._img_dim.width ** 2 + self._img_dim.height ** 2))
        assert_type(diagonal, int)
        return diagonal, diagonal

    @property
    def _img_dim(self) -> Optional[Dim]:
        """Image resolution from latest :class:`px4_msgs.msg.CameraInfo` message, None if not available"""
        if self._camera_data is not None:
            return self._camera_data.dim
        else:
            self.get_logger().warn('Camera data was not available, returning None as declared image size.')
            return None

    @property
    def _vehicle_position(self) -> Optional[Position]:
        """Vehicle position guess in WGS84 coordinates and altitude in meters above ground, None if not available"""
        if self._vehicle_global_position is not None:
            assert hasattr(self._vehicle_global_position, 'lat') and hasattr(self._vehicle_global_position, 'lon') and \
                   hasattr(self._vehicle_global_position, 'alt')
            if self._altitude_agl is None:
                # TODO: AMSL altitude can be used for map update requests, this should not be a strict requirement
                self.get_logger().warn('Could not get ground altitude, no reliable position guess for map update.')
                return None

            # TODO: make sure timestamp difference between altitude_agl (local position) and lon lat alt (global) is not too high
            crs = 'epsg:4326'
            position = Position(
                xy=GeoPoint(self._vehicle_global_position.lon, self._vehicle_global_position.lat, crs),  # lon-lat order
                z_ground=self._altitude_agl,  # not None
                z_amsl=self._vehicle_global_position.alt,
                x_sd=None,
                y_sd=None,
                z_sd=None
            )
            return position
        else:
            return None
    # endregion

    # region Initialization
    def __declare_ros_params(self) -> None:
        """Declares ROS parameters

        .. note::
            Some parameters are declared read_only and cannot be changed at runtime because there is currently no
            way to reinitialize the WMS client, pose estimator, Kalman filter, nor WMS map update timer.
        """
        read_only = ParameterDescriptor(read_only=True)
        try:
            namespace = 'wms'
            self.declare_parameters(namespace, [
                ('url', self.ROS_D_WMS_URL, read_only),
                ('version', self.ROS_D_WMS_VERSION, read_only),
                ('layers', self.ROS_D_WMS_LAYERS),
                ('styles', self.ROS_D_WMS_STYLES),
                ('srs', self.ROS_D_WMS_SRS),
                ('request_timeout', self.ROS_D_WMS_REQUEST_TIMEOUT)
            ])
        except rclpy.exceptions.ParameterAlreadyDeclaredException as e:
            self.get_logger().warn(str(e))

        try:
            namespace = 'misc'
            self.declare_parameters(namespace, [
                ('max_pitch', self.ROS_D_MISC_MAX_PITCH),
                ('variance_estimation_length', self.ROS_D_MISC_VARIANCE_ESTIMATION_LENGTH, read_only),
                ('min_match_altitude', self.ROS_D_MISC_MIN_MATCH_ALTITUDE),
                ('blur_threshold', self.ROS_D_MISC_BLUR_THRESHOLD),
                ('blur_window_length', self.ROS_D_MISC_BLUR_WINDOW_LENGTH),
            ])
        except rclpy.exceptions.ParameterAlreadyDeclaredException as e:
            self.get_logger().warn(str(e))

        try:
            namespace = 'map_update'
            self.declare_parameters(namespace, [
                ('update_delay', self.ROS_D_MAP_UPDATE_UPDATE_DELAY, read_only),
                ('gimbal_projection', self.ROS_D_MAP_UPDATE_GIMBAL_PROJECTION),
                ('max_map_radius', self.ROS_D_MAP_UPDATE_MAX_MAP_WIDTH),
                ('update_map_area_threshold', self.ROS_D_MAP_UPDATE_UPDATE_MAP_AREA_THRESHOLD),
                ('max_pitch', self.ROS_D_MAP_UPDATE_MAX_PITCH)
            ])
        except rclpy.exceptions.ParameterAlreadyDeclaredException as e:
            self.get_logger().warn(str(e))

        try:
            namespace = 'pose_estimator'
            self.declare_parameters(namespace, [
                ('class', self.ROS_D_POSE_ESTIMATOR_CLASS, read_only),
                ('params_file', self.ROS_D_POSE_ESTIMATOR_PARAMS_FILE, read_only)
            ])
        except rclpy.exceptions.ParameterAlreadyDeclaredException as e:
            self.get_logger().warn(str(e))

        try:
            namespace = 'debug'
            self.declare_parameters(namespace, [
                ('export_position', self.ROS_D_DEBUG_EXPORT_POSITION),
                ('export_projection', self.ROS_D_DEBUG_EXPORT_PROJECTION),
            ])
        except rclpy.exceptions.ParameterAlreadyDeclaredException as e:
            self.get_logger().warn(str(e))

    def _setup_pose_estimation_pool(self, params_file: str) -> Tuple[type, Pool]:
        """Returns the pose estimator type along with an initialized pool

        :param params_file: Parameter file with pose estimator full class path and initialization arguments
        :return: Tuple containing the class type and the initialized pose estimation pool
        """
        pose_estimator_params = self._load_config(params_file)
        module_name, class_name = pose_estimator_params.get('class_name', '').rsplit('.', 1)
        pose_estimator = self._import_class(class_name, module_name)
        pose_estimator_pool = Pool(self._MATCHER_PROCESS_COUNT, initializer=pose_estimator.initializer,
                                   initargs=(pose_estimator, *pose_estimator_params.get('args', []),))  # TODO: handle missing args, do not use default value

        return pose_estimator, pose_estimator_pool

    def _load_config(self, yaml_file: str) -> dict:
        """Loads config from the provided YAML file

        :param yaml_file: Path to the yaml file
        :return: The loaded yaml file as dictionary
        """
        assert_type(yaml_file, str)
        with open(os.path.join(self._package_share_dir, yaml_file), 'r') as f:
            # noinspection PyBroadException
            try:
                config = yaml.safe_load(f)
                self.get_logger().info(f'Loaded config:\n{config}.')
                return config
            except Exception as e:
                self.get_logger().error(f'Could not load config file {yaml_file} because of unexpected exception.')
                raise

    def _setup_map_update_timer(self) -> rclpy.timer.Timer:
        """Sets up a timer to throttle map update requests

        Initially map updates were triggered in VehicleGlobalPosition message callbacks, but were moved to a separate
        timer since map updates may be needed even if the EKF2 filter does not publish a global position reference (e.g.
        when GPS fusion is turned off in the EKF2_AID_MASK parameter).

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
        """Attempts to update the stored map at regular intervals

        Calls :meth:`._update_map` if the center and altitude coordinates for the new map raster are available and the
        :meth:`._should_update_map` check passes.

        New map is retrieved based on a guess of the vehicle's global position. If
        :py:attr:`._is_gimbal_projection_enabled`, the center of the projected camera field of view is used instead of
        vehicle position to ensure the field of view is best contained in the new map raster.
        """
        if self._vehicle_position is None:
            self.get_logger().warn(f'Could not determine vehicle approximate global position and therefore cannot '
                                   f'update map.')
            return

        if self._is_gimbal_projection_enabled:
            projected_center = self._guess_fov_center(self._vehicle_position)
            if projected_center is None:
                self.get_logger().warn('Could not project field of view center. Using vehicle position for map center '
                                       'instead.')

        assert position.z_ground is not None
        map_radius = self._get_dynamic_map_radius(self._vehicle_position.z_ground)
        map_candidate = GeoSquare(projected_center if projected_center is not None else self._vehicle_position.xy,
                                  map_radius)
        if self._should_request_new_map(map_candidate):
            self._request_new_map(map_candidate)
        else:
            self.get_logger().debug('Needed map not different enough to request new map yet, '
                                    'or previous results are not ready.')

    def _setup_subscribers(self) -> None:
        """Creates and stores subscribers for microRTPS bridge topics"""
        for topic_name, d in self._topics.items():
            assert topic_name is not None, f'Topic name not provided in topic: {topic_name}, {d}.'
            assert d is not None, f'Dictionary not provided for topic: {topic_name}.'
            class_ = d.get(self._TOPICS_MSG_KEY, None)
            assert class_ is not None, f'Message definition not provided for {topic_name}.'
            self._topics.update({topic_name: {self._TOPICS_SUBSCRIBER_KEY: self._create_subscriber(topic_name, class_)}})

        self.get_logger().info(f'Subscribers setup complete:\n{self._topics}.')

    def _create_subscriber(self, topic_name: str, class_: type) -> rclpy.subscription.Subscription:
        """Returns an rclpy subscription

        :param topic_name: Name of the microRTPS topic
        :param class_: Message definition class type (e.g. px4_msgs.msg.VehicleLocalPosition)
        :return: The subscriber instance
        """
        callback_name = f'_{topic_name.lower()}_callback'
        callback = getattr(self, callback_name, None)
        assert callback is not None, f'Missing callback implementation for {callback_name}.'
        return self.create_subscription(class_, topic_name, callback, 10)  # TODO: add explicit QoSProfile

    def _import_class(self, class_name: str, module_name: str) -> type:
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
    # endregion

    def _sync_timestamps(self, ekf2_timestamp_usec: int) -> None:
        """Synchronizes local timestamp with PX4 EKF2's reference time

        This synchronization is triggered in the :meth:`._vehicle_local_position_callback` and therefore expected to be
        done at high frequency.

        .. seealso:
            :py:attr:`._time_sync` and :py:attr:`._synchronized_time`

        :param ekf2_timestamp_usec: Time since PX4 EKF2 system start in microseconds
        """
        assert_type(ekf2_timestamp_usec, int)
        now_usec = time.time() * 1e6
        self._time_sync = TimePair(now_usec, ekf2_timestamp_usec)

    def _guess_fov_center(self, origin: Position) -> Optional[GeoPoint]:
        """Guesses WGS84 coordinates of camera field of view (FOV) projected on ground from given origin

        Triggered by :meth:`._map_update_timer_callback` when gimbal projection is enabled to determine center
        coordinates for next WMS GetMap request.

        :param origin: Camera position
        :return: Center of the projected FOV, or None if not available
        """
        if self._gimbal_set_attitude is None:
            self.get_logger().warn('Gimbal set attitude not available, cannot project gimbal FOV.')
            return None

        assert_type(self._gimbal_set_attitude, Attitude)
        gimbal_set_attitude = self._gimbal_set_attitude.to_esd()  # Need coordinates in image frame, not NED

        if self._camera_data is None:
            self.get_logger().warn('Camera data not available, could not create a mock pose to generate a FOV guess.')
            return None

        translation = -gimbal_set_attitude.r @ np.array([self._camera_data.cx, self._camera_data.cy, -self._camera_data.fx])
        mock_image_pair = self._mock_image_pair(origin)

        try:
            pose = Pose(gimbal_set_attitude.r, translation.reshape((3, 1)))
        except DataValueError as e:
            self.get_logger().warn(f'Pose input values: {gimbal_set_attitude.r}, {translation} were invalid: {e}.')
            return None

        altitude = self._altitude_agl
        if altitude is not None:
            try:
                mock_fixed_camera = FixedCamera(pose=pose, image_pair=mock_image_pair,
                                                ground_elevation=altitude)
            except DataValueError as _:
                self.get_logger().warn(f'Could not create a valid mock projection of FOV.')
                return None
        else:
            self.get_logger().warn(f'Could not create a valid mock projection of FOV because AGL altitude unknown.')
            return None

        if __debug__:
            export_projection = self.get_parameter('debug.export_projection').get_parameter_value().string_value
            if export_projection != '':
                self._export_position(mock_fixed_camera.fov.c, mock_fixed_camera.fov.fov, export_projection)

        return mock_fixed_camera.fov.fov.to_crs('epsg:4326').center

    def _export_position(self, position: GeoPoint, fov: GeoTrapezoid, filename: str) -> None:
        """Exports the computed position and field of view (FOV) into a geojson file

        The GeoJSON file is not used by the node but can be accessed by GIS software to visualize the data it contains.

        :param position: Computed camera position or projected principal point for gimbal projection
        :param: fov: Field of view of camera
        :param filename: Name of file to write into
        :return:
        """
        assert_type(position, GeoPoint)
        assert_type(fov, GeoTrapezoid)
        assert_type(filename, str)
        try:
            position._geoseries.append(fov._geoseries).to_file(filename)
        except Exception as e:
            self.get_logger().error(f'Could not write file {filename} because of exception:'
                                    f'\n{e}\n{traceback.print_exc()}')

    def _request_new_map(self, bbox: GeoSquare) -> None:
        """Instructs the WMS client to request a new map from the WMS server

        :param bbox: Bounding box of map requested map
        """
        if self._map_size_with_padding is None:
            self.get_logger().warn('Map size not yet available - skipping WMS request.')
            return

        # Build and send WMS request
        layers = self.get_parameter('wms.layers').get_parameter_value().string_array_value
        styles = self.get_parameter('wms.styles').get_parameter_value().string_array_value
        srs_str = self.get_parameter('wms.srs').get_parameter_value().string_value
        image_format = self.get_parameter('wms.image_format').get_parameter_value().string_value
        assert all(isinstance(x, str) for x in layers)
        assert all(isinstance(x, str) for x in styles)
        assert_len(styles, len(layers))
        assert_type(srs_str, str)
        assert_type(image_format, str)
        assert self._wms_query.result.ready(), f'New map was requested while previous results were not yet ready.'
        self.get_logger().info(f'Requesting map for bbox: {bbox.bounds}, layers: {layers}, srs: {srs_str}, format: '
                               f'{image_format}.')
        self._wms_query = AsyncWMSQuery(
            result=self._wms_pool.apply_async(
                WMSClient.worker,
                (layers, styles, bbox.bounds, self._map_size_with_padding, srs_str, image_format),
                callback=self._wms_pool_worker_callback,
                error_callback=self._wms_pool_worker_error_callback
            ),
            geobbox=bbox
        )

    # region microRTPSBridgeCallbacks
    def _image_raw_callback(self, msg: Image) -> None:
        """Handles latest :class:`px4_msgs.msg.Image` message

        :param msg: The :class:`px4_msgs.msg.Image` message from the PX4-ROS 2 bridge
        """
        # Estimate EKF2 timestamp first to get best estimate
        if self._synchronized_time is None:
            self.get_logger().warn('Image frame received but could not estimate EKF2 system time, skipping frame.')
            return None

        assert_type(msg, Image)
        cv_image = self._cv_bridge.imgmsg_to_cv2(msg, self._IMAGE_ENCODING)

        # Check that image dimensions match declared dimensions
        if self._img_dim is not None:
            cv_img_shape = cv_image.shape[0:2]
            assert cv_img_shape == self._img_dim, f'Converted cv_image shape {cv_img_shape} did not match '\
                                                  f'declared image shape {self._img_dim}.'

        if self._camera_data is None:
            self.get_logger().warn('Camera data not yet available, skipping frame.')
            return None

        image_data = ImageData(image=Img(cv_image), frame_id=msg.header.frame_id, timestamp=self._synchronized_time,
                               camera_data=self._camera_data)

        if self._should_map_match(image_data.image.arr):
            assert self._pose_estimation_query is None or self._pose_estimation_query.result.ready()
            assert self._map_data is not None
            assert hasattr(self._map_data, 'image'), 'Map data unexpectedly did not contain the image data.'

            inputs, contextual_map_data = self._match_inputs()
            self._map_input_data = inputs
            image_pair = ImagePair(image_data, contextual_map_data)
            self._estimate(image_pair, inputs)

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        """Handles latest :class:`px4_msgs.msg.CameraInfo` message

        :param msg: :class:`px4_msgs.msg.CameraInfo` message from the PX4-ROS 2 bridge
        """
        if not all(hasattr(msg, attr) for attr in ['k', 'height', 'width']):
            self.get_logger().warn(f'CameraInfo did not contain intrinsics or resolution information: {msg}.')
            return None
        else:
            self._camera_data = CameraData(k=msg.k.reshape((3, 3)), dim=Dim(msg.height, msg.width))
            camera_info_topic = self._topics.get('camera_info', {}).get(self._TOPICS_SUBSCRIBER_KEY, None)
            if camera_info_topic is not None:
                self.get_logger().warn('CameraInfo received. Assuming CameraInfo is static, destroying the '
                                       'subscription.')
                camera_info_topic.destroy()

    def _vehiclelocalposition_pubsubtopic_callback(self, msg: VehicleLocalPosition) -> None:
        """Handles latest :class:`px4_msgs.msg.VehicleLocalPosition` message

        Uses the EKF2 system time in the message to synchronize local system time.

        :param msg: :class:`px4_msgs.msg.VehicleLocalPosition` message from the PX4-ROS 2 bridge
        :return:
        """
        assert_type(msg.timestamp, int)
        self._vehicle_local_position = msg
        self._sync_timestamps(self._vehicle_local_position.timestamp)

    def _vehicleattitude_pubsubtopic_callback(self, msg: VehicleAttitude) -> None:
        """Handles latest :class:`px4_msgs.msg.VehicleAttitude` message

        :param msg: :class:`px4_msgs.msg.VehicleAttitude` message from the PX4-ROS 2 bridge
        :return:
        """
        self._vehicle_attitude = msg

    def _vehicleglobalposition_pubsubtopic_callback(self, msg: VehicleGlobalPosition) -> None:
        """Handles latest :class:`px4_msgs.msg.VehicleGlobalPosition` message

        :param msg: :class:`px4_msgs.msg.VehicleGlobalPosition` message from the PX4-ROS 2 bridge
        :return:
        """
        self._vehicle_global_position = msg

    def _gimbaldeviceattitudestatus_pubsubtopic_callback(self, msg: GimbalDeviceAttitudeStatus) -> None:
        """Handles latest :class:`px4_msgs.msg.GimbalDeviceAttitudeStatus` message

        :param msg: :class:`px4_msgs.msg.GimbalDeviceAttitudeStatus` message from the PX4-ROS 2 bridge
        :return:
        """
        self._gimbal_device_attitude_status = msg

    def _gimbaldevicesetattitude_pubsubtopic_callback(self, msg: GimbalDeviceSetAttitude) -> None:
        """Handles latest :class:`px4_msgs.msg.GimbalDeviceSetAttitude` message

        :param msg: :class:`px4_msgs.msg.GimbalDeviceSetAttitude` message from the PX4-ROS 2 bridge
        :return:
        """
        self._gimbal_device_set_attitude = msg
    # endregion

    #region WMSWorkerCallbacks
    def _wms_pool_worker_callback(self, result: List[MapData]) -> None:
        """Handles result from :meth:`gisnav.wms.worker`.

        Saves received result to :py:attr:`~_map_data. The result should be a collection containing a single
        :class:`~data.MapData`.

        :param result: Results from the asynchronous call
        :return:
        """
        #assert_len(result, 1)
        map_ = result #[0]
        # TODO: handle None
        assert_type(map_, np.ndarray)  # TODO: move this to assertions, do not hard code 4*float here and in WMSClient
        # TODO: create MapData again by retrieving self._wms_query
        assert map_.shape[0:2] == self._map_size_with_padding, 'Decoded map is not of specified size.'  # TODO: handle none/no size yet
        map_data = MapData(bbox=self._wms_query.geobbox, image=Img(map_))
        self.get_logger().info(f'Map received for bbox: {map_data.bbox}.')
        self._map_data = map_data

    def _wms_pool_worker_error_callback(self, e: BaseException) -> None:
        """Handles errors from WMS pool worker.

        :param e: Exception returned by the worker
        :return:
        """
        # TODO: handle IOError separately?
        # These are *probably* connection related exceptions from requests library. They do not seem to be part of
        # OWSLib public API so WMSClient does not handle them (in case OWSLib devs change it). Handling them would
        # require direct dependency to requests. Log exception as error here and move on.
        self.get_logger().error(f'Something went wrong with WMS process:\n{e},\n{traceback.print_exc()}.')
    #endregion

    # region MatchingWorkerCallbacks
    def _pose_estimation_worker_error_callback(self, e: BaseException) -> None:
        """Error callback for matching worker"""
        self.get_logger().error(f'Pose estimator encountered an unexpected exception:\n{e}\n{traceback.print_exc()}.')

    def _pose_estimation_worker_callback(self, result: Optional[Pose]) -> None:
        """Callback for :meth:`.PoseEstimator.worker`

        Retrieves latest :py:attr:`._pose_estimation_query.input_data` and uses it to call :meth:`._compute_output`.
        The input data is needed so that the post-processing is done using the same state information that was used for
        initiating the pose estimation in the first place. For example, camera pitch may have changed since then,
        and current camera pitch should therefore not be used for processing the matches.

        :param result: Pose result from WMS worker, or None if pose could not be estimated
        """
        if result is not None:
            try:
                pose = Pose(*result)
                self._pose_guess = pose
            except DataValueError as _:
                self.get_logger().warn(f'Estimated pose was not valid, skipping this frame.')
                return None
        else:
            self.get_logger().warn(f'Worker did not return a pose, skipping this frame.')
            return None

        output_data = self._compute_output(self._pose_estimation_query.image_pair, pose,
                                           self._pose_estimation_query.input_data)

        if output_data is not None:
            # noinspection PyUnreachableCode
            if __debug__:
                # Visualize projected FOV estimate
                fov_pix = output_data.fixed_camera.fov.fov_pix
                ref_img = output_data.fixed_camera.image_pair.ref.image.arr
                map_with_fov = cv2.polylines(ref_img.copy(),
                                             [np.int32(fov_pix)], True,
                                             255, 3, cv2.LINE_AA)

                img = np.vstack((map_with_fov, output_data.fixed_camera.image_pair.qry.image.arr))
                cv2.imshow("Projected FOV", img)
                cv2.waitKey(1)

            # Get output from Kalman filter
            orig_crs_str = output_data.fixed_camera.position.xy.crs
            filter_output = self._kf.update(output_data.fixed_camera.position.to_array())
            if filter_output is None:
                self.get_logger().warn('Waiting to get more data to estimate position error, not publishing yet.')
            else:
                filtered_position = Position.from_filtered_output(*filter_output, output_data.fixed_camera.position)
                filtered_position.xy.to_crs(orig_crs_str)
                output_data.filtered_position = filtered_position
                self.publish(output_data)

                if __debug__:
                    export_geojson = self.get_parameter('debug.export_position').get_parameter_value().string_value
                    if export_geojson != '':
                        self._export_position(output_data.filtered_position.xy, output_data.fixed_camera.fov.fov,
                                              export_geojson)

            self._map_input_data_prev = self._map_input_data
            self._map_output_data_prev = output_data
        else:
            self.get_logger().warn('Could not estimate position from pose, skipping this map match.')
    # endregion

    def _previous_map_too_close(self, bbox: GeoSquare) -> bool:
        """Checks if previous map is too close to new requested one.

        This check is made to avoid retrieving a new map that is almost the same as the previous map. Increasing map
        update interval should not improve accuracy of position estimation unless the map is so old that the field of
        view either no longer completely fits inside (vehicle has moved away or camera is looking in other direction)
        or is too small compared to the size of the map (vehicle altitude has significantly decreased).

        :param bbox: Bounding box of new map candidate
        :return: True if previous map is too close.
        """
        assert_type(bbox, GeoSquare)
        #if self._map_output_data_prev is not None:
        if self._map_data is not None:
            #previous_map_data = self._map_output_data_prev.fixed_camera.image_pair.ref.map_data  # TODO: use map_match not _match?
            previous_map_data = self._map_data
            area_threshold = self.get_parameter('map_update.update_map_area_threshold').get_parameter_value().double_value

            ratio = min(bbox.intersection(previous_map_data.bbox).area / bbox.area,
                        previous_map_data.bbox.intersection(previous_map_data.bbox).area / previous_map_data.bbox.area)

            if ratio > area_threshold:
                return True

        return False

    def _should_request_new_map(self, bbox: GeoSquare) -> bool:
        """Checks if a new WMS map request should be made to update old map.

        Map is updated unless (1) there is a previous map that is close enough to provided center and has radius
        that is close enough to new request, (2) previous WMS request is still processing, or (3) camera pitch is too
        large and gimbal projection is used so that map center would be too far or even beyond the horizon.

        :param bbox: Bounding box of new map candidate
        :return: True if map should be updated
        """
        assert_type(bbox, GeoSquare)

        if self._wms_results_pending:
            return False

        if self._previous_map_too_close(bbox):
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

    def _get_dynamic_map_radius(self, altitude: Union[int, float]) -> int:
        """Returns map radius that adjusts for camera altitude.

        :param altitude: Altitude of camera in meters
        :return: Suitable map radius in meters
        """
        assert_type(altitude, get_args(Union[int, float]))
        max_map_radius = self.get_parameter('map_update.max_map_radius').get_parameter_value().integer_value

        if self._camera_data is not None:
            #assert camera_info.k[0] == camera_info.k[4]  # Assert assumption that fx = fy  # TODO: with calibrated values this may not be exact, try something else with tolerance and warn if not within tolerance
            hfov = 2 * math.atan(self._camera_data.dim.width / (2 * self._camera_data.fx))
            map_radius = 1.5*hfov*altitude  # Arbitrary padding of 50%
        else:
            # TODO: does this happen? Trying to update map before camera info has been received?
            self.get_logger().warn(f'Could not get camera data, using best guess for map width.')
            map_radius = 3*altitude  # Arbitrary guess

        if map_radius > max_map_radius:
            self.get_logger().warn(f'Dynamic map radius {map_radius} exceeds max map radius {max_map_radius}, using '
                                   f'max_map_radius instead.')
            map_radius = max_map_radius

        return map_radius

    # TODO: no need to give max pitch arg, make property?
    def _camera_pitch_too_high(self, max_pitch: Union[int, float]) -> bool:
        """Returns True if (set) camera pitch exceeds given limit OR camera pitch is unknown.

        Used to determine whether camera pitch setting is too high up from nadir to make matching against a map
        worthwhile.

        :param max_pitch: The limit for the pitch over which it will be considered too high
        :return: True if pitch is too high
        """
        assert_type(max_pitch, get_args(Union[int, float]))
        if self._gimbal_set_attitude is not None:
            # +90 degrees to re-center from FRD frame to nadir-facing camera as origin for max pitch comparison
            pitch = np.degrees(self._gimbal_set_attitude.pitch) + 90
            if pitch > max_pitch:
                self.get_logger().warn(f'Camera pitch {pitch} is above limit {max_pitch}.')
                return True
        else:
            self.get_logger().warn(f'Gimbal attitude was not available, assuming camera pitch too high.')
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

    @staticmethod
    def _estimate_attitude(pose: Pose, rotation: float) -> np.ndarray:
        """Estimates gimbal attitude from _match and camera yaw in global NED frame

        Estimates attitude in NED frame so need map match (uses ContextualMapData.rotation), not just any match

        :param pose: Estimated pose
        :param rotation: The map data rotation
        """
        # TODO: Estimate vehicle attitude from estimated camera attitude
        #  Problem is gimbal relative attitude to vehicle body not known if gimbal not yet stabilized to set attitude,
        #  at least when using GimbalDeviceSetAttitude provided quaternion
        # Convert estimated rotation to attitude quaternion for publishing
        rT = pose.r.T
        assert not np.isnan(rT).any()
        gimbal_estimated_attitude = Rotation.from_matrix(rT)  # in rotated map pixel frame
        gimbal_estimated_attitude *= Rotation.from_rotvec(-(np.pi/2) * np.array([1, 0, 0]))  # camera body _match
        gimbal_estimated_attitude *= Rotation.from_rotvec(rotation * np.array([0, 0, 1]))  # unrotated map pixel frame

        # Re-arrange axes from unrotated (original) map pixel frame to NED frame
        rotvec = gimbal_estimated_attitude.as_rotvec()
        gimbal_estimated_attitude = Rotation.from_rotvec([-rotvec[1], rotvec[0], rotvec[2]])

        return gimbal_estimated_attitude

    def _mock_image_pair(self, origin: Position) -> Optional[ImagePair]:
        """Creates a mock image pair to be used for guessing the projected FOV needed for map updates

        The mock image pair which will be paired with a pose guess to compute the expected field of view. The expected
        field of view (or its principal point) is used to more accurately retrieve a new map from WMS that matches with
        where the camera is looking.

        :param origin: Vehicle position (altitude AMSL not required)
        :param terrain_altitude: Vehicle altitude from ground (assumed ground plane), required
        :return: Mock image pair that can be paired with a pose guess to generate a FOV guess, or None if required info not yet available
        """
        # TODO: handle none return values
        image_data = self._mock_image_data()
        map_data = self._mock_map_data(origin)
        contextual_map_data = ContextualMapData(rotation=0, crop=self._img_dim, map_data=map_data)  # TODO: redudnant img dim, check none
        image_pair = ImagePair(image_data, contextual_map_data)
        return image_pair

    def _mock_image_data(self) -> Optional[ImageData]:
        """Creates a mock ImageData to be used for guessing the projected FOV needed for map updates or None if required info not yet available"""
        # TODO: none checks
        image_data = ImageData(image=Img(np.zeros(self._img_dim)),  # TODO: if none?
                               frame_id='mock_image_data',  # TODO: have a proper frame_id
                               timestamp=self._synchronized_time,  # TODO: none?
                               camera_data=self._camera_data)  # TODO: if none?
        return image_data

    def _mock_map_data(self, origin: Position) -> Optional[MapData]:
        """Creates a mock MapData to be used for guessing the projected FOV needed for map updates

        The mock map data is used as part of mock image pair which will be paired with a pose guess to compute the
        expected field of view. The expected field of view (or its principal point) is used to more accurately retrieve
        a new map from WMS that matches with where the camera is looking.

        :param origin: Vehicle position (altitude AMSL not required)
        :return: MapData with mock images but expected bbox based on the image pixel size or None if required info not yet available
        """
        # TODO: none checks
        assert_type(origin, Position)
        assert_type(self._camera_data, CameraData)
        # TODO: make a new CameraData structure; k, cx, cy currently together with ImageData in flat structure
        # TODO: assumes fx == fy

        # Scaling factor of image pixels := terrain_altitude
        scaling = (self._map_size_with_padding[0]/2) / self._camera_data.fx  # TODO: handle None/No dim yet
        radius = scaling * origin.z_ground

        assert_type(origin.xy, GeoPoint)
        bbox = GeoSquare(origin.xy, radius)
        map_data = MapData(bbox=bbox, image=Img(np.zeros(self._map_size_with_padding)))  # TODO: handle no dim yet
        return map_data

    #region Match
    def _compute_output(self, image_pair: ImagePair, pose: Pose, input_data: InputData) -> Optional[OutputData]:
        """Process the estimated camera _match into OutputData

        :param match: Estimated _match between images
        :param input_data: InputData of vehicle state variables from the time the image was taken
        :return: Computed output_data if a valid estimate was obtained, None otherwise
        """
        attitude = self._estimate_attitude(pose, image_pair.ref.rotation)

        # Init output
        # TODO: make OutputData immutable and assign all values in constructor here
        try:
            fixed_camera = FixedCamera(pose=pose, image_pair=image_pair, ground_elevation=input_data.ground_elevation)
        except DataValueError as _:
            # Could not estimate a valid FixedCamera
            return None

        output_data = OutputData(input=input_data,
                                 fixed_camera=fixed_camera,
                                 filtered_position = None,
                                 attitude = attitude)

        return output_data

    def _match_inputs(self) -> Tuple[InputData, ContextualMapData]:
        """Returns a dictionary input_data of the input data required to perform and process a match.

        Processing of matches is asynchronous, so this method provides a way of taking a input_data of the input arguments
        to :meth:`_process_matches` from the time image used for the matching was taken.

        :return: The input data
        """
        input_data = InputData(
            ground_elevation=self._ground_elevation_amsl  # TODO: handle None
        )

        # Get cropped and rotated map
        if self._gimbal_set_attitude is not None:
            camera_yaw = self._gimbal_set_attitude.yaw
            assert_type(camera_yaw, float)
            assert -np.pi <= camera_yaw <= np.pi, f'Unexpected gimbal yaw value: {camera_yaw} ([-pi, pi] expected).'
        else:
            self.get_logger().warn(f'Camera yaw unknown, cannot match.')
            return False

        contextual_map_data = ContextualMapData(rotation=camera_yaw, map_data=self._map_data, crop=self._img_dim)
        return input_data, contextual_map_data

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
        if not (self._pose_estimation_query is None or self._pose_estimation_query.result.ready()):  # TODO: confirm that hasattr result
            return False

        # Check condition (2) - whether camera pitch is too large
        max_pitch = self.get_parameter('misc.max_pitch').get_parameter_value().integer_value
        if self._camera_pitch_too_high(max_pitch):
            self.get_logger().warn(f'Camera pitch is not available or above limit {max_pitch}. Skipping matching.')
            return False

        # Check condition (3) - whether vehicle altitude is too low
        min_alt = self.get_parameter('misc.min_match_altitude').get_parameter_value().integer_value
        # TODO: handle none altitude_agl
        if not isinstance(min_alt, int) or self._altitude_agl < min_alt:
            self.get_logger().warn(f'Altitude {self._altitude_agl} was lower than minimum threshold for matching ({min_alt}) or '
                                   f'could not be determined. Skipping matching.')
            return False

        # Check condition (4) - is image too blurry?
        if self._image_too_blurry(img):
            return False

        return True

    def _estimate(self, image_pair: ImagePair, input_data: InputData) -> None:
        """Instructs the pose estimator to estimate the pose between the image pair

        :param image_pair: The image pair to estimate the pose from
        :param input_data: Input data context
        :return:
        """
        assert self._pose_estimation_query is None or self._pose_estimation_query.result.ready()
        pose_guess = None if self._pose_guess is None else (self._pose_guess.r, self._pose_guess.t)  # TODO: to_tuple() for data.Pose?
        self._pose_estimation_query = AsyncPoseQuery(
            result=self._pose_estimator_pool.apply_async(
                self._pose_estimator.worker,
                (image_pair.qry.image.arr, image_pair.ref.image.arr, image_pair.qry.camera_data.k, pose_guess),
                callback=self._pose_estimation_worker_callback,
                error_callback=self._pose_estimation_worker_error_callback
            ),
            image_pair=image_pair,
            input_data=input_data  # TODO: no longer passed to matching, this is "context", not input
        )
    #endregion

    # region PublicAPI
    @abstractmethod
    def publish(self, output_data: OutputData) -> None:
        """Publishes the estimated position

        This method should be implemented by the extending class to adapt the base node for any given use case.
        """
        pass

    def terminate_pools(self) -> None:
        """Terminates the WMS and pose estimator pools

        .. note::
            Call this method before destroying your node and shutting down for a clean exit.
        """
        if self._pose_estimator_pool is not None:
            self.get_logger().info('Terminating pose estimator pool.')
            self._pose_estimator_pool.terminate()

        if self._wms_pool is not None:
            self.get_logger().info('Terminating WMS pool.')
            self._wms_pool.terminate()

    def destroy_timers(self) -> None:
        """Destroys the map update timer

        .. note::
            Call this method before destroying your node and shutting down for a clean exit.
        """
        if self._map_update_timer is not None:
            self.get_logger().info('Destroying map update timer.')
            self._map_update_timer.destroy()
    # endregion
