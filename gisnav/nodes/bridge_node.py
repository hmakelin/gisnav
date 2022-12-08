"""Module that contains the BridgeNode ROS 2 node."""
import sys
import rclpy
import traceback
import math
import numpy as np
import importlib
import time


from typing import Optional, Union, Tuple, get_args, Callable
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.timer import Timer

from rcl_interfaces.msg import ParameterDescriptor
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image
from mavros_msgs.msg import Altitude as ROSAltitude
from geographic_msgs.msg import BoundingBox, GeoPoint as ROSGeoPoint
from geometry_msgs.msg import Quaternion
from shapely.geometry import box
from std_msgs.msg import Float32, Header
from builtin_interfaces.msg import Time

from gisnav.data import Dim, MapData, Attitude, DataValueError, InputData, FixedCamera, Img, Pose, Position, \
    Altitude, BBox
from gisnav.geo import GeoPoint, GeoSquare
from gisnav.assertions import assert_type, assert_len, assert_ndim, assert_shape
from gisnav.autopilots.autopilot import Autopilot
from gisnav.autopilots.px4_micrortps import PX4microRTPS
from gisnav.autopilots.ardupilot_mavros import ArduPilotMAVROS

from gisnav_msgs.msg import OrthoImage3D
from gisnav_msgs.srv import GetMap

class BridgeNode(Node):
    """ROS 2 node that publishes position estimate based on visual match of drone video to map of same location"""

    # Encoding of input video (input to CvBridge)
    # e.g. gscam2 only supports bgr8 so this is used to override encoding in image header
    _IMAGE_ENCODING = 'bgr8'

    # region ROS Parameter Defaults
    ROS_D_MISC_AUTOPILOT = 'gisnav.autopilots.px4_micrortps.PX4microRTPS'
    """Default autopilot adapter"""

    ROS_D_MAP_UPDATE_MAX_PITCH = 30
    """Default maximum camera pitch from nadir in degrees for attempting to update the stored map

    This limit only applies when camera field of view (FOV) projection is enabled. This value will prevent unnecessary 
    WMS GetMap requests when the camera is looking far into the horizon and it would be unrealistic to get a good pose 
    estimate against a map.

    .. seealso::
        :py:attr:`.ROS_D_MISC_MAX_PITCH`
        :py:attr:`.ROS_D_MAP_UPDATE_GIMBAL_PROJECTION`
    """

    read_only = ParameterDescriptor(read_only=True)
    _ROS_PARAMS = [
        ('map_update.max_pitch', ROS_D_MAP_UPDATE_MAX_PITCH),
    ]
    """ROS parameter configuration to declare
    
    .. note::
        Some parameters are declared read_only and cannot be changed at runtime because there is currently no way to 
        reinitialize the WMS client, pose estimator, Kalman filter, WMS map update timer, nor the autopilot or ROS 
        subscriptions.
    """
    # endregion

    def __init__(self, name: str, px4_micrortps: bool = True) -> None:
        """Initializes the ROS 2 node.

        :param name: Name of the node
        :param px4_micrortps: Set True to use PX4 microRTPS bridge, MAVROS otherwise
        """
        assert_type(name, str)
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.__declare_ros_params()

        #self._map_cli = self.create_client(GetMap, 'orthoimage_3d_service')
        #while not self._map_cli.wait_for_service(timeout_sec=1.0):
        #    self.get_logger().info('Waiting for GetMap service...')

        self._altitude_agl_pub = self.create_publisher(Float32, 'altitude_agl', QoSPresetProfiles.SENSOR_DATA.value)  # TODO: just publish vehicle altitude
        self._camera_yaw_pub = self.create_publisher(Float32, 'camera_yaw', QoSPresetProfiles.SENSOR_DATA.value)  # TODO: just publish camera attitude (once bridge is properly refactored)
        self._vehicle_attitude_pub = self.create_publisher(Quaternion, 'attitude', QoSPresetProfiles.SENSOR_DATA.value)
        self._vehicle_position_pub = self.create_publisher(ROSGeoPoint, 'position', QoSPresetProfiles.SENSOR_DATA.value)
        self._home_position_pub = self.create_publisher(ROSGeoPoint, 'home_position', QoSPresetProfiles.SENSOR_DATA.value)
        self._gimbal_attitude_pub = self.create_publisher(Quaternion, 'gimbal_attitude', QoSPresetProfiles.SENSOR_DATA.value)
        #self._geopoint_timer = self._setup_terrain_altitude_amsl_update_timer(publish_rate)

        self._terrain_altitude_sub = self.create_subscription(ROSAltitude, "terrain_altitude",
                                                 self._terrain_altitude_callback,
                                                 QoSPresetProfiles.SENSOR_DATA.value)
        self._terrain_altitude = None

        # Autopilot bridge
        ap = 'gisnav.autopilots.px4_micrortps.PX4microRTPS' if px4_micrortps \
            else 'gisnav.autopilots.ardupilot_mavros.ArduPilotMAVROS'
        ap: Autopilot = self._load_autopilot(ap)
        # TODO: implement passing init args, kwargs
        self._bridge = ap(self, self._image_raw_callback)

        # Converts image_raw to cv2 compatible image
        self._cv_bridge = CvBridge()

        # Initialize remaining properties (does not include computed properties)
        # self._image_data = None  # Not currently used / needed
        self._map_data = None
        self._pose_guess = None
        self._msg = None  # orthoimage3d message from map node

        self._altitude_header_seq_id = 0  # TODO: for _get_header - make static function and move to common module

    # region Properties
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

    @ property
    def _map_data(self) -> Optional[MapData]:
        """The map raster from the WMS server response along with supporting metadata."""
        return self.__map_data

    @_map_data.setter
    def _map_data(self, value: Optional[MapData]) -> None:
        assert_type(value, get_args(Optional[MapData]))
        self.__map_data = value

    @property
    def _cv_bridge(self) -> CvBridge:
        """CvBridge that decodes incoming PX4-ROS 2 bridge images to cv2 images."""
        return self.__cv_bridge

    @_cv_bridge.setter
    def _cv_bridge(self, value: CvBridge) -> None:
        assert_type(value, CvBridge)
        self.__cv_bridge = value

    @property
    def _autopilot(self) -> Autopilot:
        """Autopilot bridge adapter"""
        return self.__autopilot

    @_autopilot.setter
    def _autopilot(self, value: Autopilot) -> None:
        assert_type(value, Autopilot)
        self.__autopilot = value
    # endregion

    # region Computed Properties
    @property
    def _map_size_with_padding(self) -> Optional[Tuple[int, int]]:
        """Padded map size tuple (height, width) or None if the information is not available.

        Because the deep learning models used for predicting matching keypoints or poses between camera image frames
        and map rasters are not assumed to be rotation invariant in general, the map rasters are rotated based on
        camera yaw so that they align with the camera images. To keep the scale of the map after rotation the same,
        black corners would appear unless padding is used. Retrieved maps therefore have to be squares with the side
        lengths matching the diagonal of the camera frames so that scale is preserved and no black corners appear in
        the map rasters after rotation. The height and width will both be equal to the diagonal of the declared
        (:py:attr:`._bridge.camera_data`) camera frame dimensions.
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
        if self._bridge.camera_data is not None:
            return self._bridge.camera_data.dim
        else:
            self.get_logger().warn('Camera data was not available, returning None as declared image size.')
            return None

    @property
    def _vehicle_position(self) -> Optional[Position]:
        """Vehicle position guess in WGS84 coordinates and altitude in meters above ground, None if not available"""
        if self._bridge.global_position is not None:
            assert_type(self._bridge.global_position, get_args(Optional[GeoPoint]))

            crs = 'epsg:4326'
            if self._bridge.attitude is None:
                self.get_logger().warn('Vehicle attitude not yet available, cannot determine vehicle Position.')
                return None

            if self._terrain_altitude is None:
                self.get_logger().warn('Terrain altitude not yet available, cannot determine vehicle Position.')
                return None

            try:
                position = Position(
                    xy=self._bridge.global_position,
                    altitude=Altitude(
                        agl=self._bridge.altitude_agl(self._terrain_altitude.amsl),
                        amsl=self._terrain_altitude.amsl,  # self._bridge.altitude_amsl
                        ellipsoid=self._bridge.altitude_ellipsoid,
                        home=self._bridge.altitude_home
                    ),
                    attitude=self._bridge.attitude,
                    timestamp=self._bridge.synchronized_time
                )
                return position
            except DataValueError as dve:
                self.get_logger().warn(f'Error determining vehicle position:\n{dve},\n{traceback.print_exc()}.')
                return None
        else:
            return None

    @property
    def _vehicle_latlon(self) -> Optional[GeoPoint]:
        """Vehicle lat lon coordinates, None if not available"""
        if self._bridge.global_position is not None:
            assert_type(self._bridge.global_position, get_args(Optional[GeoPoint]))
            geopoint = GeoPoint(x=self._bridge.global_position.lon, y=self._bridge.global_position.lat)
            return geopoint
        else:
            self.get_logger().warn('Could not determine vehicle latlon, returning none.')
            return None

    # endregion

    # region Initialization
    def __declare_ros_params(self) -> None:
        """Declares ROS parameters"""
        # Need to declare parameters one by one since declare_parameters will not declare remaining parameters if it
        # raises a ParameterAlreadyDeclaredException
        for param_tuple in self._ROS_PARAMS:
            try:
                self.declare_parameter(*param_tuple)
                self.get_logger().debug(f'Using default value {param_tuple[1]} for ROS parameter {param_tuple[0]}')
            except rclpy.exceptions.ParameterAlreadyDeclaredException as _:
                # This means parameter is declared from YAML file
                pass

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

    def _load_autopilot(self, autopilot: str) -> Callable:
        """Returns :class:`.Autopilot` instance from provided class path

        :param autopilot: Full path of :class:`.Autopilot` adapter class
        :return: Initialized :class:`.Autopilot` instance
        """
        assert_type(autopilot, str)
        module_name, class_name = autopilot.rsplit('.', 1)
        class_ = self._import_class(class_name, module_name)
        return class_
    # endregion

    def _image_raw_callback(self, msg: Image) -> None:
        """Handles latest :class:`px4_msgs.msg.Image` message

        :param msg: The :class:`px4_msgs.msg.Image` message from the PX4-ROS 2 bridge
        """
        # TODO Dummy callback for bridge (temporary until bridge is removed)
        if self._vehicle_position is not None:
            assert hasattr(self._vehicle_position.altitude, 'ellipsoid')
            assert self._vehicle_position.altitude.ellipsoid is not None
            position_msg = ROSGeoPoint(latitude=self._vehicle_position.lat, longitude=self._vehicle_position.lon,
                                       altitude=self._vehicle_position.altitude.ellipsoid)
            self._vehicle_position_pub.publish(position_msg)
        elif self._vehicle_latlon is not None:
            position_msg = ROSGeoPoint(latitude=self._vehicle_latlon.lat, longitude=self._vehicle_latlon.lon,
                                       altitude=np.nan)
            self._vehicle_position_pub.publish(position_msg)
        else:
            self.get_logger().warn(f'Could not determine vehicle lat lon, vehicle position.')

        if self._bridge.local_frame_origin is not None:
            position_msg = ROSGeoPoint(latitude=self._bridge.local_frame_origin.lat,
                                       longitude=self._bridge.local_frame_origin.lon,
                                       altitude=self._bridge.local_frame_origin.altitude.ellipsoid)
            self._home_position_pub.publish(position_msg)

        # Publish altitude_agl
        # TODO: parse terrain_altitude_amsl
        if self._terrain_altitude is not None:
            assert hasattr(self._terrain_altitude, 'amsl')
            assert self._terrain_altitude.amsl is not None

            altitude_agl = self._bridge.altitude_agl(self._terrain_altitude.amsl)
            if altitude_agl is not None:
                altitude_agl_msg = Float32(data=altitude_agl)
                self._altitude_agl_pub.publish(altitude_agl_msg)  # TODO: temporary, publish proper vehicle Position/Altitude message once bridge is refactored
            else:
                self.get_logger().warn(f'Could not determine vehicle altitude AGL, skipping publishing altitude')

        # Publish camera yaw
        if self._bridge.gimbal_set_attitude is not None:
            camera_yaw = self._bridge.gimbal_set_attitude.yaw
            assert_type(camera_yaw, float)
            assert -np.pi <= camera_yaw <= np.pi, f'Unexpected gimbal yaw value: {camera_yaw} ([-pi, pi] expected).'
            camera_yaw_msg = Float32(data=camera_yaw)
            self._camera_yaw_pub.publish(camera_yaw_msg)
        else:
            self.get_logger().warn(f'Could not determine camera yaw or static camera is enabled, skipping publishing camera yaw')

        # Publish vehicle attitude
        if self._bridge.attitude is not None:
            assert hasattr(self._bridge.attitude, 'q')
            quaternion = self._bridge.attitude.q
            quaternion_msg = Quaternion(x=float(quaternion[0]), y=float(quaternion[1]), z=float(quaternion[2]),
                                        w=float(quaternion[3]))
            self._vehicle_attitude_pub.publish(quaternion_msg)
        else:
            self.get_logger().warn(f'Could not determine vehicle attitude, skipping publishing it.')

        # Publish gimbal attitude
        if self._bridge.gimbal_set_attitude is not None:
            assert hasattr(self._bridge.gimbal_set_attitude, 'q')
            quaternion = self._bridge.gimbal_set_attitude.q
            quaternion_msg = Quaternion(x=float(quaternion[0]), y=float(quaternion[1]), z=float(quaternion[2]),
                                        w=float(quaternion[3]))
            self._gimbal_attitude_pub.publish(quaternion_msg)
        else:
            self.get_logger().warn(f'Could not determine vehicle attitude, skipping publishing it.')

    def _terrain_altitude_callback(self, msg: ROSAltitude) -> None:
        """Handles terrain altitude message"""
        self._terrain_altitude = msg

    # TODO: redudnant implementation in pose_estimation_node.py
    def _get_header(self) -> Header:
        """Creates class:`std_msgs.msg.Header` for an outgoing ROS message"""
        ns = time.time_ns()
        sec = int(ns / 1e9)
        nanosec = int(ns - (1e9 * sec))
        header = Header()
        time_ = Time()
        time_.sec = sec
        time_.nanosec = nanosec
        header.stamp = time_
        header.frame_id = 'base_link'

        #header.seq = self._altitude_header_seq_id
        #self._altitude_header_seq_id += 1

        return header

    # region PublicAPI
    def unsubscribe_topics(self) -> None:
        """Unsubscribes from all ROS topics

        .. note::
            Call this method when before destroying your node for a clean exit.
        """
        self._bridge.unsubscribe_all()
    # endregion
