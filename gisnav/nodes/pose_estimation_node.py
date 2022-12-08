"""Module that contains the pose estimation node"""
import rclpy
import os
import sys
import yaml
import io
import pstats
import numpy as np
import cProfile
import time
import traceback
import importlib


from typing import Optional, Union, Callable, Tuple, get_args

import cv2
from scipy.spatial.transform import Rotation
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from ament_index_python.packages import get_package_share_directory

from cv_bridge import CvBridge

from geometry_msgs.msg import Pose
from sensor_msgs.msg import CameraInfo, Image
from mavros_msgs.msg import Altitude
from geometry_msgs.msg import Quaternion
from geographic_msgs.msg import GeoPoint as ROSGeoPoint, GeoPoseStamped, GeoPose as ROSGeoPose
from std_msgs.msg import Header, Float32
from builtin_interfaces.msg import Time

from gisnav_msgs.msg import OrthoImage3D
from gisnav.pose_estimators.pose_estimator import PoseEstimator
from gisnav.assertions import assert_type, assert_ndim, assert_shape
from gisnav.data import Pose, FixedCamera, DataValueError, ImageData, Img, Attitude, ContextualMapData, BBox, Dim, \
    MapData, CameraData, ImagePair
from gisnav.geo import GeoPoint, GeoTrapezoid
from gisnav.autopilots.autopilot import Autopilot
from gisnav.nodes.map_node import MapNode


class PoseEstimationNode(Node):
    """Publishes pose between two images"""

    # Encoding of input video (input to CvBridge)
    # e.g. gscam2 only supports bgr8 so this is used to override encoding in image header
    _IMAGE_ENCODING = 'bgr8'

    ROS_D_STATIC_CAMERA = False
    """Default value for static camera flag (true for static camera facing down from vehicle body)"""

    ROS_D_DEBUG_EXPORT_POSITION = '' # 'position.json'
    """Default filename for exporting GeoJSON containing estimated field of view and position

    Set to '' to disable
    """

    ROS_D_MISC_MAX_PITCH = 30
    """Default maximum camera pitch from nadir in degrees for attempting to estimate pose against reference map

    .. seealso::
        :py:attr:`.ROS_D_MAP_UPDATE_MAX_PITCH` 
        :py:attr:`.ROS_D_MAP_UPDATE_GIMBAL_PROJECTION`
    """

    ROS_D_MISC_MIN_MATCH_ALTITUDE = 80
    """Default minimum ground altitude in meters under which matches against map will not be attempted"""

    ROS_D_MISC_ATTITUDE_DEVIATION_THRESHOLD = 10
    """Magnitude of allowed attitude deviation of estimate from expectation in degrees"""

    _ROS_PARAMS = [
        ('static_camera', ROS_D_STATIC_CAMERA),
        ('max_pitch', ROS_D_MISC_MAX_PITCH),
        ('min_match_altitude', ROS_D_MISC_MIN_MATCH_ALTITUDE),
        ('attitude_deviation_threshold', ROS_D_MISC_ATTITUDE_DEVIATION_THRESHOLD),
        ('export_position', ROS_D_DEBUG_EXPORT_POSITION),
    ]
    """ROS parameters used by this node to declare"""

    def __init__(self, name: str, params_file: str,  package_share_dir: str, px4_micrortps: bool = True) -> None:
        """Node initializer

        :param name: Node name
        :param params_file: Pose estimator params file
        :param package_share_dir: Package shared directory
        :param px4_micrortps: Set to False to assume ArduPilot/MAVROS
        """
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self._package_share_dir = package_share_dir  # TODO: temporary, do before loading autopilot config

        #self._image_sub = self.create_subscription(Image, 'camera/image_raw', self._image_raw_callback,
        #                                           QoSPresetProfiles.SENSOR_DATA.value)
        self._camera_info_sub = self.create_subscription(CameraInfo, "camera/camera_info",
                                                         self._camera_info_callback,
                                                         QoSPresetProfiles.SENSOR_DATA.value)
        self._map_sub = self.create_subscription(OrthoImage3D, "orthoimage_3d",
                                                 self._map_callback,
                                                 QoSPresetProfiles.SENSOR_DATA.value)
        self._terrain_altitude_sub = self.create_subscription(Altitude, "terrain_altitude",
                                                 self._terrain_altitude_callback,
                                                 QoSPresetProfiles.SENSOR_DATA.value)
        self._terrain_geopoint_sub = self.create_subscription(ROSGeoPoint, "ground_geopoint",
                                                 self._ground_geopoint_callback,
                                                 QoSPresetProfiles.SENSOR_DATA.value)
        self._altitude_agl_sub = self.create_subscription(Float32, "altitude_agl",
                                                 self._altitude_agl_callback,
                                                 QoSPresetProfiles.SENSOR_DATA.value)
        self._camera_yaw_sub = self.create_subscription(Float32, "camera_yaw",
                                                 self._camera_yaw_callback,
                                                 QoSPresetProfiles.SENSOR_DATA.value)
        self._vehicle_attitude_sub = self.create_subscription(Quaternion, "attitude",
                                                 self._vehicle_attitude_callback,
                                                 QoSPresetProfiles.SENSOR_DATA.value)
        self._gimbal_attitude_sub = self.create_subscription(Quaternion, "gimbal_attitude",
                                                 self._gimbal_attitude_callback,
                                                 QoSPresetProfiles.SENSOR_DATA.value)
        self._home_position_sub = self.create_subscription(ROSGeoPoint, MapNode.DEFAULT_HOME_POSITION_TOPIC,
                                                           self._home_position_callback,
                                                           QoSPresetProfiles.SENSOR_DATA.value)
        self._camera_data = None
        self._map = None
        self._terrain_altitude = None
        self._altitude_agl = None
        self._camera_yaw = None
        self._vehicle_attitude = None
        self._home_position = None
        #self._pub = self.create_publisher(Pose, 'pose', QoSPresetProfiles.SENSOR_DATA.value)
        self._geopoint_pub = self.create_publisher(ROSGeoPoint, 'geopoint_estimate', QoSPresetProfiles.SENSOR_DATA.value)
        self._geopose_pub = self.create_publisher(GeoPoseStamped, 'geopose_estimate', QoSPresetProfiles.SENSOR_DATA.value)
        self._altitude_pub = self.create_publisher(Altitude, 'altitude_estimate', QoSPresetProfiles.SENSOR_DATA.value)

        # Converts image_raw to cv2 compatible image
        self._cv_bridge = CvBridge()

        # TODO: redundant, replace with ROS topics
        # Autopilot bridge
        ap = 'gisnav.autopilots.px4_micrortps.PX4microRTPS' if px4_micrortps \
            else 'gisnav.autopilots.ardupilot_mavros.ArduPilotMAVROS'
        ap: Autopilot = self._load_autopilot(ap)
        self._bridge = ap(self, self._image_raw_callback)

        params = self._load_config(params_file)
        module_name, class_name = params.get('class_name', '').rsplit('.', 1)
        pose_estimator: PoseEstimator = self._import_class(class_name, module_name)
        self._estimator = pose_estimator(*params.get('args', []))

        # Increasing header nonce, see :meth:`._get_header`
        self._altitude_header_seq_id = 0

    # TODO: redundant, should only be used in basenode?
    def _load_autopilot(self, autopilot: str) -> Callable:
        """Returns :class:`.Autopilot` instance from provided class path

        :param autopilot: Full path of :class:`.Autopilot` adapter class
        :return: Initialized :class:`.Autopilot` instance
        """
        assert_type(autopilot, str)
        module_name, class_name = autopilot.rsplit('.', 1)
        class_ = self._import_class(class_name, module_name)
        return class_

    # TODO: redundant, should only be used in basenode?
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

    # TODO: redundant implementation in map_node, base_node, bbox_node
    def _declare_ros_params(self) -> None:
        """Declares ROS parameters"""
        # Declare parameters one by one because declare_parameters will not declare remaining parameters if it
        # raises a ParameterAlreadyDeclaredException
        for param_tuple in self._ROS_PARAMS:
            param, default_value = param_tuple
            try:
                self.declare_parameter(param, default_value)
                self.get_logger().info(f'Using default value "{default_value}" for ROS parameter "{param}".')
            except ParameterAlreadyDeclaredException as _:
                # This means parameter is already declared (e.g. from a YAML file)
                value = self.get_parameter(param).value
                self.get_logger().info(f'ROS parameter "{param}" already declared with value "{value}".')

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

    @property
    def _altitude_scaling(self) -> Optional[float]:
        """Returns camera focal length divided by camera altitude in meters."""
        if self._camera_data is not None and self._altitude_agl is not None:
            return self._camera_data.k[0][0] / self._altitude_agl.data  # TODO: assumes fx == fy
        else:
            self.get_logger().warn('Could not estimate elevation scale because camera focal length and/or vehicle '
                                   'altitude is unknown.')
            return None

    @property
    def _img_dim(self) -> Optional[Dim]:
        """Image resolution from latest :class:`px4_msgs.msg.CameraInfo` message, None if not available"""
        if self._camera_data is not None:
            return self._bridge.camera_data.dim
        else:
            self.get_logger().warn('Camera data was not available, returning None as declared image size.')
            return None

    @property
    def _r_guess(self) -> Optional[np.ndarray]:
        """Gimbal rotation matrix guess (based on :class:`px4_msgs.GimbalDeviceSetAttitude` message)

        .. note::
            Should be roughly same as rotation matrix stored in :py:attr:`._pose_guess`, even though it is derived via
            a different route. If gimbal is not stabilized to its set position, the rotation matrix will be different.
        """
        static_camera = self.get_parameter('static_camera').get_parameter_value().bool_value
        if self._gimbal_attitude is None:
            if not static_camera:
                self.get_logger().warn('Gimbal set attitude not available, will not provide pose guess.')
                return None
            else:
                if self._gimbal_attitude is not None:
                    # TODO: redundant for parsing vehicle attitude
                    q_ = np.array([self._gimbal_attitude.x, self._gimbal_attitude.y, self._gimbal_attitude.z,
                                  self._gimbal_attitude.w])
                    attitude = Attitude(q=q_)
                    attitude = attitude.as_rotation()
                    attitude *= Rotation.from_euler('XYZ', [0, -np.pi/2, 0])
                    return Attitude(attitude.as_quat()).to_esd().r
                else:
                    self.get_logger().warn('Vehicle attitude not available, will not provide pose guess for static '
                                           'camera.')
                    return None
        else:
            assert_type(self._gimbal_attitude, Quaternion)
            # TODO: redundant with above parsing, also with vehicle attitude parsing
            q_ = np.array([self._gimbal_attitude.x, self._gimbal_attitude.y, self._gimbal_attitude.z,
                          self._gimbal_attitude.w])
            attitude = Attitude(q=q_)
            gimbal_attitude = attitude.to_esd()  # Need coordinates in image frame, not NED
            return gimbal_attitude.r

    # TODO: redundant implementation in map node
    @property
    def _get_map_size(self) -> Optional[Tuple[int, int]]:
        """GetMap request size parameter or None if not available

        .. note::
            This is a square with side length equal to the diagonal of the camera resolution. This resolution is chosen
            to give the map raster sufficient padding so that it can be center-cropped to the same resolution as the
            camera image without creating any corners that are black. This is needed to support pose estimators that
            are not rotation agnostic.
        """
        if self._camera_data is not None:
            diagonal = int(np.ceil(np.sqrt(self._camera_data.dim.width ** 2 + self._camera_data.dim.height ** 2)))
            assert_type(diagonal, int)
            return diagonal, diagonal
        else:
            self.get_logger().warn('Cannot compute GetMap request raster size, CameraInfo not yet received.')
            return None

    @property
    def _contextual_map_data(self) -> Optional[ContextualMapData]:
        """Returns contextual (rotated) map data for pose estimation

        :return: Rotated map with associated metadata, or None if not available
        """
        #input_data = InputData(
        #    r_guess=self._r_guess,
        #    snapshot=self._bridge.snapshot(self._terrain_altitude_amsl_at_position(self._bridge.global_position))
        #)

        static_camera = self.get_parameter('static_camera').get_parameter_value().bool_value

        # Get cropped and rotated map
        if self._camera_yaw is not None and not static_camera:
            camera_yaw = self._camera_yaw.data
            assert_type(camera_yaw, float)
            assert -np.pi <= camera_yaw <= np.pi, f'Unexpected gimbal yaw value: {camera_yaw} ([-pi, pi] expected).'
        else:
            if not static_camera:
                self.get_logger().warn(f'Camera yaw unknown, cannot estimate pose.')
                return
            else:
                self.get_logger().debug(f'Assuming zero yaw relative to vehicle body for static nadir-facing camera.')
                assert self._vehicle_attitude is not None
                q_ = np.array([self._vehicle_attitude.x, self._vehicle_attitude.y, self._vehicle_attitude.z,
                              self._vehicle_attitude.w])
                attitude = Attitude(q=q_)
                camera_yaw = attitude.yaw

        contextual_map_data = ContextualMapData(rotation=camera_yaw, map_data=self._map, crop=self._img_dim,
                                                altitude_scaling=self._altitude_scaling)
        return contextual_map_data

    def _should_estimate(self, img: np.ndarray) -> bool:
        """Determines whether :meth:`._estimate` should be called

        Match should be attempted if (1) a reference map has been retrieved, (2) camera roll or pitch is not too high
        (e.g. facing horizon instead of nadir), and (3) drone is not flying too low.

        :param img: Image from which to estimate pose
        :return: True if pose estimation be attempted
        """
        # Check condition (1) - that _map_data exists
        if self._map is None:
            self.get_logger().warn(f'No reference map available. Skipping pose estimation.')
            return False

        # Check condition (2) - whether camera roll/pitch is too large
        max_pitch = self.get_parameter('max_pitch').get_parameter_value().integer_value
        if self._camera_roll_or_pitch_too_high(max_pitch):
            self.get_logger().warn(f'Camera roll or pitch not available or above limit {max_pitch}. Skipping pose '
                                   f'estimation.')
            return False

        # Check condition (3) - whether vehicle altitude is too low
        min_alt = self.get_parameter('min_match_altitude').get_parameter_value().integer_value
        #altitude = self._bridge.altitude_agl(self._terrain_altitude_amsl_at_position(self._bridge.global_position))
        altitude = self._altitude_agl
        if altitude is None:
            self.get_logger().warn('Cannot determine altitude AGL, skipping map update.')
            return None
        if not isinstance(min_alt, int) or altitude is None or altitude.data < min_alt:
            self.get_logger().warn(f'Assumed altitude {altitude.data} was lower than minimum threshold for matching '
                                   f'({min_alt}) or could not be determined. Skipping pose estimation.')
            return False

        return True

    def _image_raw_callback(self, msg: Image) -> None:
        """Handles latest :class:`px4_msgs.msg.Image` message

        :param msg: The :class:`px4_msgs.msg.Image` message from the PX4-ROS 2 bridge
        """
        # Estimate EKF2 timestamp first to get best estimate
        if self._bridge.synchronized_time is None:
            self.get_logger().warn('Image frame received but could not estimate EKF2 system time, skipping frame.')
            return None

        assert_type(msg, Image)
        cv_image = self._cv_bridge.imgmsg_to_cv2(msg, self._IMAGE_ENCODING)

        # Check that image dimensions match declared dimensions
        if self._img_dim is not None:
            cv_img_shape = cv_image.shape[0:2]
            assert cv_img_shape == self._img_dim, f'Converted cv_image shape {cv_img_shape} did not match '\
                                                  f'declared image shape {self._img_dim}.'

        if self._bridge.camera_data is None:
            self.get_logger().warn('Camera data not yet available, skipping frame.')
            return None

        image_data = ImageData(image=Img(cv_image), frame_id=msg.header.frame_id,
                               timestamp=self._bridge.synchronized_time, camera_data=self._bridge.camera_data)

        if self._should_estimate(image_data.image.arr):
            assert self._map is not None
            assert self._camera_data is not None
            assert hasattr(self._map, 'image'), 'Map data unexpectedly did not contain the image data.'

            # TODO: grab estimation inputs (elevation etc.) from ROS topics (see BaseNode._estimation_inputs)
            #inputs, contextual_map_data = self._estimation_inputs
            #self._map_input_data = inputs
            image_pair = ImagePair(image_data, self._contextual_map_data)
            pose = self._estimator.estimate(image_data.image.arr, image_pair.ref.image.arr, self._camera_data.k)
            if pose is not None:
                pose = Pose(*pose)
                self._pose_estimation_worker_callback(pose, image_pair)
            else:
                self.get_logger().warn(f'Worker did not return a pose, skipping this frame.')
                return None

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        """Handles latest :class:`sensor_msgs.msg.CameraInfo` message

        .. note::
            Checks that intrinsic matrix is received and then destroys subscription (assumes camera info is static)

        :param msg: Latest :class:`sensor_msgs.msg.CameraInfo` message
        """
        if not all(hasattr(msg, attr) for attr in ['k', 'height', 'width']):
            # TODO: check that k and height/width match
            return None
        else:
            #self._k = msg.k.reshape((3, 3))  # store camera data instead
            self._camera_data = CameraData(msg.k.reshape((3, 3)), dim=Dim(msg.height, msg.width))
            #if self._camera_info_sub is not None:
            #    # Assume camera info is static, destroy subscription
            #    self.get_logger().warn("CameraInfo received, destroying subscription.")

    def _map_callback(self, msg: OrthoImage3D) -> None:
        """Handles latest :class:`gisnav_msgs.msg.OrthoImage3D` message

        :param msg: Latest :class:`gisnav_msgs.msg.OrthoImage3D` message
        """
        bbox = msg.bbox
        bbox = BBox(bbox.min_pt.longitude, bbox.min_pt.latitude, bbox.max_pt.longitude, bbox.max_pt.latitude)
        img = self._cv_bridge.imgmsg_to_cv2(msg.img, desired_encoding='passthrough')
        dem = self._cv_bridge.imgmsg_to_cv2(msg.dem, desired_encoding='passthrough')
        self._map = self._wms_pool_worker_callback(bbox, img, dem)

    def _terrain_altitude_callback(self, msg: Altitude) -> None:
        """Handles latest terrain altitude message

        :param msg: Latest :class:`mavros_msgs.msg.Altitude` message
        """
        self._terrain_altitude = msg

    def _home_position_callback(self, msg: ROSGeoPoint) -> None:
        """Handles latest home position message

        :param msg: Latest :class:`geographic_msgs.msg.GeoPoint` message
        """
        self._home_position = msg

    def _ground_geopoint_callback(self, msg: ROSGeoPoint) -> None:
        """Handles latest terrain geopoint message

        :param msg: Latest :class:`geographic_msgs.msg.GeoPoint` message
        """
        self._ground_geopoint = msg

    def _altitude_agl_callback(self, msg: Float32) -> None:
        """Handles latest altitude AGL (meters) message

        :param msg: Latest :class:`std_msgs.msg.Float32` message (altitude AGL in meters)
        """
        self._altitude_agl = msg

    def _camera_yaw_callback(self, msg: Float32) -> None:
        """Handles latest camera yaw (radians) message

        :param msg: Latest :class:`std_msgs.msg.Float32` message (camera yaw in radians)
        """
        self._camera_yaw = msg

    def _vehicle_attitude_callback(self, msg: Quaternion) -> None:
        """Handles latest vehicle attitude message

        :param msg: Latest :class:`geometry_msgs.msg.Quaternion` message
        """
        self._vehicle_attitude = msg

    def _gimbal_attitude_callback(self, msg: Quaternion) -> None:
        """Handles latest gimbal attitude message

        .. note::
            This could be gimbal set attitude, not actual attitude

        :param msg: Latest :class:`geometry_msgs.msg.Quaternion` message
        """
        self._gimbal_attitude = msg

    # TODO: make MapData out of OrthoImage3D message
    def _wms_pool_worker_callback(self, bbox: BBox, img: np.ndarray, dem: np.ndarray) -> Optional[MapData]:
        """Makes MapData out of OrthoImage3D message

        :param bbox: Map bounding box (WGS 84)
        :param img: Orthoimage raster
        :param dem: DEM raster (zero array of same size as :param img: if DEMs are not available)
        :return: MapData or None if not available
        """
        assert_type(img, np.ndarray)
        assert_ndim(dem, 2)
        assert_shape(dem, img.shape[0:2])

        assert self._get_map_size is not None

        # Should already have received camera info so _map_size_with_padding should not be None
        assert img.shape[0:2] == self._get_map_size, f'Decoded map {img.shape[0:2]} is not of specified ' \
                                                     f'size {self._get_map_size}.'

        elevation = Img(dem) if dem is not None else None
        map_data = MapData(bbox=bbox, image=Img(img), elevation=elevation)

        #self._map_data = map_data
        return map_data

    def _pose_estimation_worker_callback(self, pose: Pose, image_pair: ImagePair) -> None:
        """Handles estimated pose

        Retrieves latest :py:attr:`._pose_estimation_query.input_data` and uses it to call :meth:`._compute_output`.
        The input data is needed so that the post-processing is done using the same state information that was used for
        initiating the pose estimation in the first place. For example, camera pitch may have changed since then,
        and current camera pitch should therefore not be used for processing the matches.

        :param pose: Pose result from WMS worker, or None if pose could not be estimated
        """
        try:
            # Compute DEM value at estimated position
            # This is in camera intrinsic (pixel) units with origin at whatever the DEM uses
            # For example, the USGS DEM uses NAVD 88
            x, y = -pose.t.squeeze()[0:2]
            x, y = int(x), int(y)
            elevation = image_pair.ref.elevation.arr[y, x]
            pose = Pose(pose.r, pose.t - elevation)
        except DataValueError as _:
            self.get_logger().warn(f'Estimated pose was not valid, skipping this frame.')
            return None
        except IndexError as __:
            # TODO: might be able to handle this
            self.get_logger().warn(f'Estimated pose was not valid, skipping this frame.')
            return None

        try:
            #input_data = self._pose_estimation_query.input_data
            #self.get_logger().info(f'snapshot terrain alt {input_data.snapshot.terrain_altitude}')
            # TODO: pass terrain_altitude_amsl and terrain_altitude_ellipsoid instead of snapshot
            # TODO: get ellipsoid altitude, or publish ellipsoid values for current position and use them here
            #altitude = Altitude(amsl=self._terrain_altitude.amsl, ellipsoid=self._terrain_altitude.ellipsoid)
            assert self._ground_geopoint is not None
            assert hasattr(self._ground_geopoint, 'altitude')
            assert self._terrain_altitude is not None
            fixed_camera = FixedCamera(pose=pose, image_pair=image_pair,
                                       terrain_altitude_amsl=self._terrain_altitude.amsl,
                                       terrain_altitude_ellipsoid=self._ground_geopoint.altitude,
                                       home_position=self._home_position,
                                       timestamp=image_pair.qry.timestamp)
        except DataValueError as _:
            self.get_logger().warn(f'Could not estimate a valid camera position, skipping this frame.')
            return None

        if not self._is_valid_estimate(fixed_camera, self._r_guess):
            self.get_logger().warn('Estimate did not pass post-estimation validity check, skipping this frame.')
            return None

        assert fixed_camera is not None
        # noinspection PyUnreachableCode
        if __debug__:
            # Visualize projected FOV estimate
            fov_pix = fixed_camera.fov.fov_pix
            ref_img = fixed_camera.image_pair.ref.image.arr
            map_with_fov = cv2.polylines(ref_img.copy(),
                                         [np.int32(fov_pix)], True,
                                         255, 3, cv2.LINE_AA)

            img = np.vstack((map_with_fov, fixed_camera.image_pair.qry.image.arr))
            cv2.imshow("Projected FOV", img)
            cv2.waitKey(1)

            # Export GeoJSON
            export_geojson = self.get_parameter('export_position').get_parameter_value().string_value
            if export_geojson != '':
                self._export_position(fixed_camera.position.xy, fixed_camera.fov.fov, export_geojson)

        self.publish(fixed_camera)

    def _is_valid_estimate(self, fixed_camera: FixedCamera, r_guess_: np.ndarray) -> bool:
        """Returns True if the estimate is valid

        Compares computed estimate to guess based on set gimbal device attitude. This will reject estimates made when
        the gimbal was not stable (which is strictly not necessary), which is assumed to filter out more inaccurate
        estimates.
        """
        static_camera = self.get_parameter('misc.static_camera').get_parameter_value().bool_value
        if static_camera:
            vehicle_attitude = self._bridge.attitude
            if vehicle_attitude is None:
                self.get_logger().warn('Gimbal attitude was not available, cannot do post-estimation validity check'
                                       'for static camera.')
                return False

            # Add vehicle roll & pitch (yaw handled separately through map rotation)
            #r_guess = Attitude(Rotation.from_rotvec([vehicle_attitude.roll, vehicle_attitude.pitch - np.pi/2, 0])
            #                   .as_quat()).to_esd().as_rotation()
            r_guess = Attitude(Rotation.from_euler('XYZ', [vehicle_attitude.roll, vehicle_attitude.pitch - np.pi / 2,
                                                           0]).as_quat()).to_esd().as_rotation()

        if r_guess_ is None and not static_camera:
            self.get_logger().warn('Gimbal attitude was not available, cannot do post-estimation validity check.')
            return False

        if not static_camera:
            r_guess = Rotation.from_matrix(r_guess_)
            # Adjust for map rotation
            camera_yaw = fixed_camera.image_pair.ref.rotation
            camera_yaw = Rotation.from_euler('xyz', [0, 0, camera_yaw], degrees=False)
            r_guess *= camera_yaw

        r_estimate = Rotation.from_matrix(fixed_camera.pose.r)

        magnitude = Rotation.magnitude(r_estimate * r_guess.inv())

        threshold = self.get_parameter('attitude_deviation_threshold').get_parameter_value().integer_value
        threshold = np.radians(threshold)

        if magnitude > threshold:
            self.get_logger().warn(f'Estimated rotation difference to expected was too high (magnitude '
                                   f'{np.degrees(magnitude)}).')
            return False

        return True

    def _camera_roll_or_pitch_too_high(self, max_pitch: Union[int, float]) -> bool:
        """Returns True if (set) camera roll or pitch exceeds given limit OR camera pitch is unknown

        Used to determine whether camera roll or pitch is too high up from nadir to make matching against a map
        not worthwhile. Checks roll for static camera, but assumes zero roll for 2-axis gimbal (static_camera: False).

        .. note::
            Uses actual vehicle attitude (instead of gimbal set attitude) if static_camera ROS param is True

        :param max_pitch: The limit for the pitch in degrees from nadir over which it will be considered too high
        :return: True if pitch is too high
        """
        assert_type(max_pitch, get_args(Union[int, float]))
        static_camera = self.get_parameter('misc.static_camera').get_parameter_value().bool_value
        pitch = None
        if self._bridge.gimbal_set_attitude is not None and not static_camera:
            # TODO: do not assume zero roll here - camera attitude handling needs refactoring
            # +90 degrees to re-center from FRD frame to nadir-facing camera as origin for max pitch comparison
            pitch = np.degrees(self._bridge.gimbal_set_attitude.pitch) + 90
        else:
            if not static_camera:
                self.get_logger().warn('Gimbal attitude was not available, assuming camera pitch too high.')
                return True
            else:
                if self._bridge.attitude is None:
                    self.get_logger().warn('Vehicle attitude was not available, assuming static camera pitch too high.')
                    return True
                else:
                    pitch = max(self._bridge.attitude.pitch, self._bridge.attitude.roll)

        assert pitch is not None
        if pitch > max_pitch:
            self.get_logger().warn(f'Camera pitch {pitch} is above limit {max_pitch}.')
            return True

        return False

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

    def publish(self, fixed_camera: FixedCamera) -> None:
        """Publishes estimated position

        :param fixed_camera: Estimated position and other metadata
        """
        geopoint_msg = ROSGeoPoint(latitude=fixed_camera.position.lat, longitude=fixed_camera.position.lon,
                            altitude=fixed_camera.position.altitude.ellipsoid)
        #self._geopoint_pub.publish(geopoint_msg)

        altitude_msg = Altitude(header=self._get_header(),
                                amsl=fixed_camera.position.altitude.amsl,
                                local=fixed_camera.position.altitude.home,
                                relative=fixed_camera.position.altitude.home,
                                terrain=fixed_camera.position.altitude.agl,
                                bottom_clearance=fixed_camera.position.altitude.agl)
        self._altitude_pub.publish(altitude_msg)

        q_xyzw = fixed_camera.position.attitude.q
        geopose_msg = GeoPoseStamped(
            header=self._get_header(),
            pose=ROSGeoPose(
                position=geopoint_msg,
                orientation=Quaternion(x=float(q_xyzw[0]), y=float(q_xyzw[1]), z=float(q_xyzw[2]), w=float(q_xyzw[3]))
            )
        )
        self._geopose_pub.publish(geopose_msg)

    def _get_header(self) -> Header:
        """Creates class:`std_msgs.msg.Header` for an outgoing ROS message"""
        ns = time.time_ns()
        sec = int(ns / 1e9)
        nanosec = int(ns - (1e9 * sec))
        header = Header()
        #time_ = Time()
        #time_.sec = sec
        #time_.nanosec = nanosec
        #header.stamp = time_
        header.stamp.sec = sec
        header.stamp.nanosec = nanosec
        header.frame_id = 'base_link'

        #header.seq = self._altitude_header_seq_id
        #self._altitude_header_seq_id += 1

        return header

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

    node = None
    try:
        rclpy.init(args=args)
        node = PoseEstimationNode('bbox_node', 'config/loftr_params.yaml', get_package_share_directory('gisnav'),
                                  px4_micrortps='--mavros' not in sys.argv)
        rclpy.spin(node)
    except KeyboardInterrupt as e:
        print(f'Keyboard interrupt received:\n{e}')
        if pr is not None:
            # Print out profiling stats
            pr.disable()
            s = io.StringIO()
            ps = pstats.Stats(pr, stream=s).sort_stats(pstats.SortKey.CUMULATIVE)
            ps.print_stats(40)
            print(s.getvalue())
    finally:
        if node is not None:
            #node.destroy()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
