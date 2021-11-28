import rclpy
import sys
import os
import traceback
import yaml
import importlib
import math
import time
import warnings

from enum import Enum
from typing import Any, Optional, Union
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from owslib.wms import WebMapService
from cv2 import VideoCapture, imwrite, imdecode
import numpy as np
import cv2
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation
from functools import partial
from wms_map_matching.util import get_bbox, setup_sys_path, convert_fov_from_pix_to_wgs84,\
    write_fov_and_camera_location_to_geojson, get_bbox_center, BBox, Dimensions, rotate_and_crop_map, \
    visualize_homography, get_fov, get_camera_distance, get_distance_of_fov_center, LatLon, fov_to_bbox,\
    get_angle, create_src_corners, uncrop_pixel_coordinates, rotate_point, move_distance, RPY, LatLonAlt, distances,\
    ImageFrame, distance

from px4_msgs.msg import VehicleVisualOdometry, VehicleAttitude, VehicleLocalPosition, VehicleGlobalPosition,\
    GimbalDeviceAttitudeStatus, GimbalDeviceSetAttitude
from sensor_msgs.msg import CameraInfo, Image

# Add the share folder to Python path
share_dir, superglue_dir = setup_sys_path()

# Import this after util.setup_sys_path has been called
from wms_map_matching.superglue import SuperGlue

class Matcher(Node):
    # scipy Rotations: {‘X’, ‘Y’, ‘Z’} for intrinsic, {‘x’, ‘y’, ‘z’} for extrinsic rotations
    EULER_SEQUENCE = 'YXZ'

    # Minimum matches for homography estimation, should be at least 4
    MINIMUM_MATCHES = 4

    # Encoding of input video (input to CvBridge)
    IMAGE_ENCODING = 'bgr8'  # E.g. gscam2 only supports bgr8 so this is used to override encoding in image header

    # Local frame reference for px4_msgs.msg.VehicleVisualOdometry messages
    LOCAL_FRAME_NED = 0

    # Maps properties to microRTPS bridge topics and message definitions
    # TODO: get rid of static TOPICS and dynamic _topics dictionaries - just use one dictionary, initialize it in constructor?
    TOPIC_NAME_KEY = 'topic_name'
    CLASS_KEY = 'class'
    SUBSCRIBE_KEY = 'subscribe'  # Used as key in both Matcher.TOPICS and Matcher._topics
    PUBLISH_KEY = 'publish'  # Used as key in both Matcher.TOPICS and Matcher._topics
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
            TOPIC_NAME_KEY: 'VehicleVisualOdometry_PubSubTopic',
            CLASS_KEY: VehicleVisualOdometry,
            PUBLISH_KEY: True
        }
    ]

    def __init__(self, share_directory, superglue_directory, config='config.yml'):
        """Initializes the node.

        Arguments:
            share_dir - String path of the share directory where configuration and other files are.
            superglue_dir - String path of the directory where SuperGlue related files are.
            config - String path to the config file in the share folder.
        """
        super().__init__('matcher')
        self.share_dir = share_directory  # TODO: make private?
        self.superglue_dir = superglue_directory  # TODO: move this to _setup_superglue? private _superglue_dir instead?
        self._load_config(config)
        self._init_wms()

        # Dict for storing all microRTPS bridge subscribers and publishers
        self._topics = dict()
        self._setup_topics()

        # Dict for storing latest microRTPS messages
        #self._topics_msgs = dict()

        # Converts image_raw to cv2 compatible image
        self._cv_bridge = CvBridge()

        # Store map raster received from WMS endpoint here along with its bounding box
        self._map = None  # TODO: put these together in a MapBBox structure to make them 'more atomic' (e.g. ImageFrameStamp named tuple)
        self._map_bbox = None

        self._setup_superglue()

        self._gimbal_fov_wgs84 = []  # TODO: remove this attribute, just passing it through here from _update_map to _match (temp hack)

        # To be used for pyproj transformations
        #self._g = pyproj.Geod(ellps='clrk66')  # TODO: move pyproj stuff from util.py here under Matcher() class

        self._previous_image_frame = None  # ImageFrame from previous match, needed to compute velocity

        # Properties that are mapped to microRTPS topics
        self._camera_info = None
        self._vehicle_local_position = None
        self._vehicle_global_position = None
        self._vehicle_attitude = None
        self._gimbal_device_attitude_status = None
        self._gimbal_device_set_attitude = None

    @staticmethod
    def _assert_same_type(type_: Any, value: object) -> None:
        """Asserts that inputs are of same type."""
        assert isinstance(value, type_), f'Type {type(value)} provided when {type_} was expected.'

    @property
    def previous_image_frame(self) -> ImageFrame:
        return self._previous_image_frame

    @previous_image_frame.setter
    def previous_image_frame(self, value: ImageFrame) -> None:
        self._assert_same_type(ImageFrame, value)
        self._previous_image_frame = value

    @property
    def camera_info(self) -> CameraInfo:
        return self._camera_info

    @camera_info.setter
    def camera_info(self, value: CameraInfo) -> None:
        self._assert_same_type(CameraInfo, value)
        self._camera_info = value

    @property
    def vehicle_local_position(self) -> VehicleLocalPosition:
        return self._vehicle_local_position

    @vehicle_local_position.setter
    def vehicle_local_position(self, value: VehicleLocalPosition) -> None:
        self._assert_same_type(VehicleLocalPosition, value)
        self._vehicle_local_position = value

    @property
    def vehicle_global_position(self) -> VehicleGlobalPosition:
        return self._vehicle_global_position

    @vehicle_global_position.setter
    def vehicle_global_position(self, value: VehicleGlobalPosition) -> None:
        self._assert_same_type(VehicleGlobalPosition, value)
        self._vehicle_global_position = value

    @property
    def vehicle_attitude(self) -> VehicleAttitude:
        return self._vehicle_attitude

    @vehicle_attitude.setter
    def vehicle_attitude(self, value: VehicleAttitude) -> None:
        self._assert_same_type(VehicleAttitude, value)
        self._vehicle_attitude = value

    @property
    def gimbal_device_attitude_status(self) -> GimbalDeviceAttitudeStatus:
        return self._gimbal_device_attitude_status

    @gimbal_device_attitude_status.setter
    def gimbal_device_attitude_status(self, value: GimbalDeviceAttitudeStatus) -> None:
        self._assert_same_type(GimbalDeviceAttitudeStatus, value)
        self._gimbal_device_attitude_status = value

    @property
    def gimbal_device_set_attitude(self) -> GimbalDeviceSetAttitude:
        return self._gimbal_device_set_attitude

    @gimbal_device_set_attitude.setter
    def gimbal_device_set_attitude(self, value: GimbalDeviceSetAttitude) -> None:
        self._assert_same_type(GimbalDeviceSetAttitude, value)
        self._gimbal_device_set_attitude = value

    def _setup_superglue(self):
        """Sets up SuperGlue."""  # TODO: make all these private?
        self._superglue = SuperGlue(self._config['superglue'], self.get_logger())

    def _get_previous_global_position(self):
        """Returns previous global position (WGS84)."""
        raise NotImplementedError

    def _load_config(self, yaml_file):
        """Loads config from the provided YAML file."""
        with open(os.path.join(share_dir, yaml_file), 'r') as f:
            try:
                self._config = yaml.safe_load(f)
                self.get_logger().info('Loaded config:\n{}.'.format(self._config))
            except Exception as e:
                self.get_logger().error('Could not load config file {} because of exception: {}\n{}' \
                                        .format(yaml_file, e, traceback.print_exc()))

    def _use_gimbal_projection(self):
        """Returns True if gimbal projection is enabled for fetching map bbox rasters."""
        gimbal_projection_flag = self._config.get('misc', {}).get('gimbal_projection', False)
        if type(gimbal_projection_flag) is bool:
            return gimbal_projection_flag
        else:
            self.get_logger().warn(f'Could not read gimbal projection flag: {gimbal_projection_flag}. Assume False.')
            return False

    def _restrict_affine(self):
        """Returns True if homography matrix should be restricted to an affine transformation (nadir facing camera)."""
        restrict_affine_flag = self._config.get('misc', {}).get('affine', False)
        if type(restrict_affine_flag) is bool:
            return restrict_affine_flag
        else:
            self.get_logger().warn(f'Could not read affine restriction flag: {restrict_affine_flag}. Assume False.')
            return False

    def _import_class(self, class_name, module_name):
        """Imports class from module if not yet imported."""
        if module_name not in sys.modules:
            self.get_logger().info('Importing module ' + module_name + '.')
            importlib.import_module(module_name)
        imported_class = getattr(sys.modules[module_name], class_name, None)
        assert imported_class is not None, class_name + ' was not found in module ' + module_name + '.'
        return imported_class

    def _setup_topics(self) -> None:
        """Creates publishers and subscribers for microRTPS bridge topics."""
        for topic in self.TOPICS:
            topic_name = topic.get(self.TOPIC_NAME_KEY, None)
            class_ = topic.get(self.CLASS_KEY, None)
            assert topic_name is not None, f'Topic name not provided in topic: {topic}.'
            assert class_ is not None, f'Class not provided in topic: {topic}.'

            publish = topic.get(self.PUBLISH_KEY, None)
            if publish is not None:
                assert isinstance(publish, bool), f'Type {type(publish)} provided when bool was expected.'
                self._topics.update({self.PUBLISH_KEY: {topic_name: self._create_publisher(topic_name, class_)}})

            subscribe = topic.get(self.SUBSCRIBE_KEY, None)
            if subscribe is not None:
                assert isinstance(subscribe, bool), f'Type {type(subscribe)} provided when bool was expected.'
                self._topics.update({self.SUBSCRIBE_KEY: {topic_name: self._create_subscriber(topic_name, class_)}})

        self.get_logger().info('Topics setup complete.')

    def _create_publisher(self, topic_name: str, class_: object) -> rclpy.publisher.Publisher:
        """Sets up an rclpy publisher."""
        return self.create_publisher(class_, topic_name, 10)

    def _create_subscriber(self, topic_name: str, class_: object) -> rclpy.subscription.Subscription:
        """Sets up an rclpy subscriber."""
        callback_name = '_' + topic_name.lower() + '_callback'
        callback = getattr(self, callback_name, None)
        assert callback is not None, f'Missing callback implementation for {callback_name}.'
        return self.create_subscription(class_, topic_name, callback, 10)

    def _init_wms(self):
        """Initializes the Web Map Service (WMS) client used by the node to request map rasters.

        The url and version parameters are required to initialize the WMS client and are therefore set to read only. The
        layer and srs parameters can be changed dynamically.
        """
        self.declare_parameter('url', self._config['wms']['url'], ParameterDescriptor(read_only=True))
        self.declare_parameter('version', self._config['wms']['version'], ParameterDescriptor(read_only=True))
        self.declare_parameter('layer', self._config['wms']['layer'])
        self.declare_parameter('srs', self._config['wms']['srs'])

        try:
            self._wms = WebMapService(self.get_parameter('url').get_parameter_value().string_value,
                                      version=self.get_parameter('version').get_parameter_value().string_value)
        except Exception as e:
            self.get_logger().error('Could not connect to WMS server.')
            raise e

    def _map_dim(self) -> Optional[tuple]:
        # TODO: docstring after this thing returns a Dimensions tuple and not a regular tuple
        camera_info = self.camera_info
        if camera_info is not None:
            max_dim = max(self.camera_info.width, self.camera_info.height)
            return max_dim, max_dim  # TODO: return a Dimensions tuple, not a regular tuple
        else:
            self.get_logger().warn('Camera info not available, returning None for map dim.')
            return None

    def _map_size_with_padding(self):
        dim = self._img_dimensions()
        if type(dim) is not Dimensions:
            self.get_logger().warn(f'Dimensions not available - returning None as map size.')
            return None
        assert hasattr(dim, 'width') and hasattr(dim, 'height'), 'Dimensions did not have expected attributes.'
        diagonal = math.ceil(math.sqrt(dim.width ** 2 + dim.height ** 2))
        return diagonal, diagonal

    def _map_dimensions_with_padding(self):
        map_size = self._map_size_with_padding()
        if map_size is None:
            self.get_logger().warn(f'Map size with padding not available - returning None as map dimensions.')
            return None
        assert len(map_size) == 2, f'Map size was unexpected length {len(map_size)}, 2 expected.'
        return Dimensions(*map_size)

    def _declared_img_size(self) -> Optional[tuple]:
        """Returns image resolution size as it is declared in the latest CameraInfo message."""
        if self.camera_info is not None:
            return self.camera_info.height, self.camera_info.width  # numpy order: h, w, c --> height first
        else:
            self.get_logger().warn('Camera info was not available - returning None as declared image size.')
            return None

    def _img_dimensions(self):
        declared_size = self._declared_img_size()
        if declared_size is None:
            self.get_logger().warn('CDeclared size not available - returning None as image dimensions.')
            return None
        else:
            return Dimensions(*declared_size)

    def _project_gimbal_fov(self, altitude_meters: float) -> Optional[np.ndarray]:
        """Returns field of view BBox projected using gimbal attitude and camera intrinsics information."""
        rpy = self._get_camera_rpy()
        if rpy is None:
            self.get_logger().warn('Could not get RPY - cannot project gimbal fov.')
            return

        r = Rotation.from_euler(self.EULER_SEQUENCE, list(rpy), degrees=True).as_matrix()
        e = np.hstack((r, np.expand_dims(np.array([0, 0, altitude_meters]), axis=1)))  # extrinsic matrix  # [0, 0, 1]
        assert e.shape == (3, 4), 'Extrinsic matrix had unexpected shape: ' + str(e.shape) \
                                  + ' - could not project gimbal FoV.'

        if self.camera_info is None:
            self.get_logger().warn('Could not get camera info - cannot project gimbal fov.')
            return
        h, w = self._img_dimensions()
        # TODO: assert h w not none and integers? and divisible by 2?

        # Intrinsic matrix
        k = np.array(self.camera_info.k).reshape([3, 3])

        # Project image corners to z=0 plane (ground)
        src_corners = create_src_corners(h, w)

        e = np.delete(e, 2, 1)  # Remove z-column, making the matrix square
        p = np.matmul(k, e)
        p_inv = np.linalg.inv(p)

        dst_corners = cv2.perspectiveTransform(src_corners, p_inv)  # TODO: use util.get_fov here?
        dst_corners = dst_corners.squeeze()  # See get_fov usage elsewhere -where to do squeeze if at all?

        return dst_corners

    def _vehicle_global_position_latlonalt(self) -> Optional[LatLonAlt]:
        """Returns vehicle global position as a LatLonAlt tuple."""
        if self.vehicle_global_position is None:
            self.get_logger().warn('Could not get vehicle global position - returning None.')
            return None
        return LatLonAlt(self.vehicle_global_position.lat, self.vehicle_global_position.lon,
                         self.vehicle_global_position.alt)

    def _vehicle_local_position_ref_latlonalt(self) -> Optional[LatLonAlt]:
        """Returns vehicle local frame origin as LatLonAlt tuple."""
        if self.vehicle_local_position is None:
            self.get_logger().warn('Could not get vehicle local position - returning None as local frame reference.')
            return None

        if self.vehicle_local_position.xy_global is True and self.vehicle_local_position.z_global is True:
            return LatLonAlt(self.vehicle_local_position.ref_lat, self.vehicle_local_position.ref_lon,
                             self.vehicle_local_position.ref_alt)
        else:
            # TODO: z may not be needed - make a separate _ref_latlon method!
            self.get_logger().warn('No valid global reference for local frame origin - returning None.')
            return None

    def _update_map(self) -> None:
        """Gets latest map from WMS server and saves it."""
        global_position_latlonalt = self._vehicle_global_position_latlonalt()
        if global_position_latlonalt is None:
            self.get_logger().warn('Could not get vehicle global position latlonalt. Cannot update map.')
            return None

        # Use these coordinates for fetching map from server
        map_center_latlon = LatLon(global_position_latlonalt.lat, global_position_latlonalt.lon)

        if self._use_gimbal_projection():
            if self.camera_info is not None:
                gimbal_fov_pix = self._project_gimbal_fov(global_position_latlonalt.alt)

                # Convert gimbal field of view from pixels to WGS84 coordinates
                if gimbal_fov_pix is not None:
                    azimuths = list(map(lambda x: math.degrees(math.atan2(x[0], x[1])), gimbal_fov_pix))
                    distances = list(map(lambda x: math.sqrt(x[0]**2 + x[1]**2), gimbal_fov_pix))
                    zipped = list(zip(azimuths, distances))
                    to_wgs84 = partial(move_distance, map_center_latlon)
                    self._gimbal_fov_wgs84 = np.array(list(map(to_wgs84, zipped)))
                    ### TODO: add some sort of assertion hat projected FoV is contained in size and makes sense

                    # Use projected field of view center instead of global position as map center
                    map_center_latlon = get_bbox_center(fov_to_bbox(self._gimbal_fov_wgs84))
                else:
                    self.get_logger().warn('Could not project camera FoV, getting map raster assuming nadir-facing '
                                           'camera.')
            else:
                self.get_logger().debug('Camera info not available, cannot project gimbal FoV, defaulting to global '
                                        'position.')

        self._map_bbox = get_bbox(map_center_latlon)

        # Build and send WMS request
        layer_str = self.get_parameter('layer').get_parameter_value().string_value
        srs_str = self.get_parameter('srs').get_parameter_value().string_value
        self.get_logger().debug(f'Getting map for bounding box: {self._map_bbox}, layer: {layer_str}, srs: {srs_str}.')
        try:
            self._map = self._wms.getmap(layers=[layer_str], srs=srs_str, bbox=self._map_bbox,
                                         size=self._map_size_with_padding(), format='image/png',
                                         transparent=True)
        except Exception as e:
            self.get_logger().warn('Exception from WMS server query: {}\n{}'.format(e, traceback.print_exc()))
            return

        # Decode response from WMS server
        self._map = np.frombuffer(self._map.read(), np.uint8)
        self._map = imdecode(self._map, cv2.IMREAD_UNCHANGED)
        assert self._map.shape[0:2] == self._map_size_with_padding(), 'Decoded map is not the specified size.'

    def _image_raw_callback(self, msg):
        """Handles latest image frame from camera."""
        self.get_logger().debug('Camera image callback triggered.')
        #self._topics_msgs['image_raw'] = msg

        # Get image data
        assert hasattr(msg, 'data'), f'No data present in received image message.'
        if msg.data is None:  # TODO: do an explicit type check here?
            self.get_logger().warn('No data present in received image message - cannot process image.')
            return

        cv_image = self._cv_bridge.imgmsg_to_cv2(msg, self.IMAGE_ENCODING)

        img_size = self._declared_img_size()
        if img_size is not None:
            cv_img_shape = cv_image.shape[0:2]
            declared_shape = self._declared_img_size()
            assert cv_img_shape == declared_shape, f'Converted cv_image shape {cv_img_shape} did not match declared ' \
                                                   f'image shape {declared_shape}.'

        # Get image frame_id and stamp from message header
        assert hasattr(msg, 'header'), f'No header present in received image message.'
        if msg.header is None:  # TODO: do an explicit type check here?
            self.get_logger().warn('No header present in received image message - cannot process image.')
            return
        assert hasattr(msg.header, 'frame_id'), f'No frame_id present in received image header.'
        assert hasattr(msg.header, 'stamp'), f'No stamp present in received image header.'
        frame_id = msg.header.frame_id
        timestamp = msg.header.stamp
        if frame_id is None or timestamp is None:  # TODO: do an explicit type check here?
            self.get_logger().warn(f'No frame_id or stamp in received header: {msg.header}, cannot process image.')
            return
        image_frame = ImageFrame(cv_image, frame_id, timestamp)

        self._match(image_frame)

    def _camera_yaw(self):
        """Returns camera yaw in degrees."""
        rpy = self._get_camera_rpy()

        assert rpy is not None, 'RPY is None, cannot retrieve camera yaw.'
        assert len(rpy) == 3, f'Unexpected length for RPY: {len(rpy)}.'
        assert hasattr(rpy, 'yaw'), f'No yaw attribute found for named tuple: {rpy}.'

        camera_yaw = rpy.yaw
        return camera_yaw

    def _get_camera_rpy(self) -> RPY:
        """Returns roll-pitch-yaw euler vector."""
        gimbal_attitude = self._gimbal_attitude()
        if gimbal_attitude is None:
            self.get_logger().warn('Gimbal attitude not available, cannot return RPY.')
            return None
        assert hasattr(gimbal_attitude, 'q'), 'Gimbal attitude quaternion not available - cannot compute RPY.'
        gimbal_euler = Rotation.from_quat(gimbal_attitude.q).as_euler(self.EULER_SEQUENCE, degrees=True)

        if self.vehicle_local_position is None:
            self.get_logger().warn('VehicleLocalPosition is unknown, cannot get heading. Cannot return RPY.')
            return None

        pitch_index = self._pitch_index()
        assert pitch_index != -1, 'Could not identify pitch index in gimbal attitude, cannot return RPY.'

        yaw_index = self._yaw_index()
        assert yaw_index != -1, 'Could not identify yaw index in gimbal attitude, cannot return RPY.'

        self.get_logger().warn('Assuming stabilized gimbal - ignoring vehicle intrinsic pitch and roll for camera RPY.')
        self.get_logger().warn('Assuming zero roll for camera RPY.')  # TODO remove zero roll assumption

        heading = self.vehicle_local_position.heading
        assert -math.pi <= heading <= math.pi, 'Unexpected heading value: ' + str(
            heading) + '([-pi, pi] expected). Cannot compute RPY.'
        heading = math.degrees(heading)

        gimbal_yaw = gimbal_euler[yaw_index]
        assert -180 <= gimbal_yaw <= 180, 'Unexpected gimbal yaw value: ' + str(
            heading) + '([-180, 180] expected). Cannot compute RPY.'
        yaw = heading + gimbal_yaw  # TODO: if over 180, make it negative instead
        assert abs(yaw) <= 360, f'Yaw was unexpectedly large: {abs(yaw)}, max 360 expected.'
        if abs(yaw) > 180:  # Important: >, not >= (because we are using mod 180 operation below)
            yaw = yaw % 180 if yaw < 0 else yaw % -180  # Make the compound yaw between -180 and 180 degrees
        pitch = -(90 + gimbal_euler[pitch_index])  # TODO: ensure abs(pitch) <= 90?
        roll = 0  # TODO remove zero roll assumption
        rpy = RPY(roll, pitch, yaw)

        return rpy

    def _get_camera_normal(self):
        nadir = np.array([0, 0, 1])
        rpy = self._get_camera_rpy()
        if rpy is None:
            self.get_logger().warn('Could not get RPY - cannot compute camera normal.')
            return None

        r = Rotation.from_euler(self.EULER_SEQUENCE, list(rpy), degrees=True)
        camera_normal = r.apply(nadir)

        assert camera_normal.shape == nadir.shape, f'Unexpected camera normal shape {camera_normal.shape}.'
        # TODO: this assertion is arbitrary? how to handle unexpected camera normal length?
        camera_normal_length = np.linalg.norm(camera_normal)
        assert abs(camera_normal_length-1) <= 0.001, f'Unexpected camera normal length {camera_normal_length}.'

        return camera_normal

    def _pitch_index(self):
        return self.EULER_SEQUENCE.lower().find('y')

    def _yaw_index(self):
        return self.EULER_SEQUENCE.lower().find('x')

    def _camera_info_callback(self, msg: CameraInfo) -> None:
        """Handles latest camera info message."""
        self.get_logger().debug(f'Camera info received:\n{msg}.')
        self.camera_info = msg
        camera_info_topic = self._topics.get(self.SUBSCRIBE_KEY, {}).get('camera_info', None)
        if camera_info_topic is not None:
            self.get_logger().warn('Assuming camera_info is static - destroying the subscription.')
            camera_info_topic.destroy()

    def _vehiclelocalposition_pubsubtopic_callback(self, msg: VehicleLocalPosition) -> None:
        """Handles latest VehicleLocalPosition message."""
        self.vehicle_local_position = msg

    def _vehicleglobalposition_pubsubtopic_callback(self, msg: VehicleGlobalPosition) -> None:
        """Handles latest VehicleGlobalPosition message."""
        self.vehicle_global_position = msg
        self._update_map()

    def _gimbaldeviceattitudestatus_pubsubtopic_callback(self, msg: GimbalDeviceAttitudeStatus) -> None:
        """Handles latest GimbalDeviceAttitudeStatus message."""
        self.gimbal_device_attitude_status = msg

    def _gimbaldevicesetattitude_pubsubtopic_callback(self, msg: GimbalDeviceSetAttitude) -> None:
        """Handles latest GimbalDeviceSetAttitude message."""
        self.gimbal_device_set_attitude = msg

    def _vehicleattitude_pubsubtopic_callback(self, msg: VehicleAttitude) -> None:
        """Handles latest VehicleAttitude message."""
        self.vehicle_attitude = msg

    def _publish_vehicle_visual_odometry(self, position: tuple, velocity: tuple) -> None:
        """Publishes a VehicleVisualOdometry message over the microRTPS bridge as defined in
        https://github.com/PX4/px4_msgs/blob/master/msg/VehicleVisualOdometry.msg. """
        module_name = 'px4_msgs.msg'   #TODO: get ffrom config file
        class_name = 'VehicleVisualOdometry'  # TODO: get from config file or look at _import_class stuff in this file
        #VehicleVisualOdometry = getattr(sys.modules[module_name], class_name, None)
        assert VehicleVisualOdometry is not None, f'{class_name} was not found in module {module_name}.'
        msg = VehicleVisualOdometry()

        # TODO: could throw a warning if position and velocity BOTH are None - would publish a message full of NaN

        # Timestamp
        now = int(time.time() * 1e6)  # uint64 time in microseconds  # TODO: should be time since system start?
        msg.timestamp = now
        msg.timestamp_sample = now  # uint64 TODO: what's this?

        # Position and linear velocity local frame of reference
        msg.local_frame = self.LOCAL_FRAME_NED  # uint8

        # Position
        if position is not None:
            assert len(position) == 3, f'Unexpected length for position estimate: {len(position)} (3 expected).'  # TODO: can also be length 2 if altitude is not published, handle that
            assert all(isinstance(x, float) for x in position), f'Position contained non-float elements.'
            # TODO: check for np.float32?
            msg.x, msg.y, msg.z = position  # float32 North, East, Down
        else:
            self.get_logger().warn('Position tuple was None - publishing NaN as position.')
            msg.x, msg.y, msg.z = (float('nan'), ) * 3  # float32 North, East, Down

        # Attitude quaternions - not used
        msg.q = (float('nan'), ) * 4  # float32
        msg.q_offset = (float('nan'), ) * 4
        msg.pose_covariance = (float('nan'), ) * 21

        # Velocity frame of reference
        msg.velocity_frame = self.LOCAL_FRAME_NED  # uint8

        # Velocity
        if velocity is not None:
            assert len(velocity) == 3, f'Unexpected length for velocity estimate: {len(velocity)} (3 expected).'
            assert all(isinstance(x, float) for x in velocity), f'Velocity contained non-float elements.'
            # TODO: check for np.float32?
            msg.vx, msg.vy, msg.vz = velocity  # float32 North, East, Down
        else:
            self.get_logger().warn('Velocity tuple was None - publishing NaN as velocity.')
            msg.vx, msg.vy, msg.vz = (float('nan'), ) * 3  # float32 North, East, Down

        # Angular velocity - not used
        msg.rollspeed, msg.pitchspeed, msg.yawspeed = (float('nan'), ) * 3  # float32 TODO: remove redundant np.float32?
        msg.velocity_covariance = (float('nan'), ) * 21  # float32 North, East, Down

    def _camera_pitch(self):
        """Returns camera pitch in degrees relative to vehicle frame."""
        rpy = self._get_camera_rpy()
        if rpy is None:
            self.get_logger().warn('Gimbal RPY not available, cannot compute camera pitch.')

        assert len(rpy) == 3, 'Unexpected length of euler angles vector: ' + str(len(rpy))
        assert hasattr(rpy, 'pitch'), f'Pitch attribute not found in named tuple {rpy}.'

        return rpy.pitch

    def _gimbal_attitude(self) -> Optional[Union[GimbalDeviceAttitudeStatus, GimbalDeviceSetAttitude]]:
        """Returns 1. GimbalDeviceAttitudeStatus, or 2. GimbalDeviceSetAttitude if 1. is not available."""
        gimbal_attitude = self.gimbal_device_attitude_status
        if gimbal_attitude is None:
            # Try alternative topic
            self.get_logger().warn('GimbalDeviceAttitudeStatus not available. Trying GimbalDeviceSetAttitude instead.')
            gimbal_attitude = self.gimbal_device_set_attitude
            if gimbal_attitude is None:
                self.get_logger().warn('GimbalDeviceSetAttitude not available. Gimbal attitude status not available.')
        return gimbal_attitude

    def _process_matches(self, mkp_img, mkp_map, k, camera_normal, reproj_threshold=1.0, method=cv2.RANSAC, affine=False):
        """Processes matching keypoints from img and map and returns essential, and homography matrices & pose.

        Arguments:
            mkp_img - The matching keypoints from image.
            mkp_map - The matching keypoints from map.
            k - The intrinsic camera matrix.
            camera_normal - The camera normal unit vector.
            reproj_threshold - The RANSAC reprojection threshold for homography estimation.
            method - Method to use for estimation.
            affine - Boolean flag indicating that transformation should be restricted to 2D affine transformation
        """
        min_points = 4
        assert len(mkp_img) >= min_points and len(mkp_map) >= min_points, 'Four points needed to estimate homography.'
        if not affine:
            h, h_mask = cv2.findHomography(mkp_img, mkp_map, method, reproj_threshold)
        else:
            h, h_mask = cv2.estimateAffinePartial2D(mkp_img, mkp_map)
            h = np.vstack((h, np.array([0, 0, 1])))  # Make it into a homography matrix

        num, Rs, Ts, Ns = cv2.decomposeHomographyMat(h, k)

        # Get the one where angle between plane normal and inverse of camera normal is smallest
        # Plane is defined by Z=0 and "up" is in the negative direction on the z-axis in this case
        get_angle_partial = partial(get_angle, -camera_normal)
        angles = list(map(get_angle_partial, Ns))
        index_of_smallest_angle = angles.index(min(angles))
        rotation, translation = Rs[index_of_smallest_angle], Ts[index_of_smallest_angle]

        self.get_logger().debug('decomposition R:\n{}.'.format(rotation))
        self.get_logger().debug('decomposition T:\n{}.'.format(translation))
        self.get_logger().debug('decomposition Ns:\n{}.'.format(Ns))
        self.get_logger().debug('decomposition Ns angles:\n{}.'.format(angles))
        self.get_logger().debug('decomposition smallest angle index:\n{}.'.format(index_of_smallest_angle))
        self.get_logger().debug('decomposition smallest angle:\n{}.'.format(min(angles)))

        return h, h_mask, translation, rotation

    def _match(self, image_frame) -> None:
        """Matches camera image to map image and computes camera position and field of view."""
        try:
            self.get_logger().debug('Matching image to map.')

            if self._map is None:
                self.get_logger().warn('Map not yet available - skipping matching.')
                return

            yaw = self._camera_yaw()
            if yaw is None:
                self.get_logger().warn('Could not get camera yaw. Skipping matching.')
                return
            rot = math.radians(yaw)
            assert -math.pi <= rot <= math.pi, 'Unexpected gimbal yaw value: ' + str(rot) + ' ([-pi, pi] expected).'
            self.get_logger().debug('Current camera yaw: ' + str(rot) + ' radians.')

            map_cropped = rotate_and_crop_map(self._map, rot, self._img_dimensions())
            assert map_cropped.shape[0:2] == self._declared_img_size(), 'Cropped map did not match declared shape.'

            mkp_img, mkp_map = self._superglue.match(image_frame.image, map_cropped)

            match_count_img = len(mkp_img)
            assert match_count_img == len(mkp_map), 'Matched keypoint counts did not match.'
            if match_count_img < self.MINIMUM_MATCHES:
                self.get_logger().warn(f'Found {match_count_img} matches, {self.MINIMUM_MATCHES} required. Skip frame.')
                return

            camera_normal = self._get_camera_normal()
            if camera_normal is None:
                self.get_logger().warn('Could not get camera normal. Skipping matching.')
                return

            if self.camera_info is None:
                self.get_logger().warn('Could not get camera info. Skipping matching.')
                return
            k = self.camera_info.k.reshape([3, 3])

            h, h_mask, t, r = self._process_matches(mkp_img, mkp_map, k, camera_normal, affine=self._restrict_affine())

            assert h.shape == (3, 3), f'Homography matrix had unexpected shape: {h.shape}.'
            assert t.shape == (3, 1), f'Translation vector had unexpected shape: {t.shape}.'
            assert r.shape == (3, 3), f'Rotation matrix had unexpected shape: {r.shape}.'

            fov_pix = get_fov(image_frame.image, h)
            visualize_homography('Matches and FoV', image_frame.image, map_cropped, mkp_img, mkp_map, fov_pix)

            map_lat, map_lon = get_bbox_center(BBox(*self._map_bbox))

            map_dims_with_padding = self._map_dimensions_with_padding()
            if map_dims_with_padding is None:
                self.get_logger().warn('Could not get map dimensions info. Skipping matching.')
                return

            img_dimensions = self._img_dimensions()
            if map_dims_with_padding is None:
                self.get_logger().warn('Could not get img dimensions info. Skipping matching.')
                return

            fov_wgs84, fov_uncropped, fov_unrotated = convert_fov_from_pix_to_wgs84(
                fov_pix, map_dims_with_padding, self._map_bbox, rot, img_dimensions)
            image_frame.fov = fov_wgs84

            # Compute camera altitude, and distance to principal point using triangle similarity
            # TODO: _update_map has similar logic used in gimbal fov projection, try to combine
            fov_center_line_length = get_distance_of_fov_center(fov_wgs84)
            focal_length = k[0][0]
            assert hasattr(img_dimensions, 'width') and hasattr(img_dimensions, 'height'), \
                'Img dimensions did not have expected attributes.'
            camera_distance = get_camera_distance(focal_length, img_dimensions.width, fov_center_line_length)
            camera_pitch = self._camera_pitch()
            camera_altitude = None
            if camera_pitch is None:
                # TODO: Use some other method to estimate altitude if pitch not available?
                self.get_logger().warn('Camera pitch not available - cannot estimate altitude visually.')
            else:
                camera_altitude = math.cos(math.radians(camera_pitch)) * camera_distance  # TODO: use rotation from decomposeHomography for getting the pitch in this case (use visual info, not from sensors)
            self.get_logger().debug(f'Camera pitch {camera_pitch} deg, distance to principal point {camera_distance} m,'
                                    f' altitude {camera_altitude} m.')

            """
            mkp_map_uncropped = []
            for i in range(0, len(mkp_map)):
                mkp_map_uncropped.append(list(
                    uncrop_pixel_coordinates(self._img_dimensions(), self._map_dimensions_with_padding(), mkp_map[i])))
            mkp_map_uncropped = np.array(mkp_map_uncropped)

            mkp_map_unrotated = []
            for i in range(0, len(mkp_map_uncropped)):
                mkp_map_unrotated.append(
                    list(rotate_point(rot, self._map_dimensions_with_padding(), mkp_map_uncropped[i])))
            mkp_map_unrotated = np.array(mkp_map_unrotated)

            h2, h_mask2, translation_vector2, rotation_matrix2 = self._process_matches(mkp_img, mkp_map_unrotated,
                                                                                 # mkp_map_uncropped,
                                                                                 self._camera_info().k.reshape([3, 3]),
                                                                                 cam_normal,
                                                                                 affine=
                                                                                 self._config['misc']['affine'])

            fov_pix_2 = get_fov(self._cv_image, h2)
            visualize_homography('Uncropped and unrotated', self._cv_image, self._map, mkp_img, mkp_map_unrotated,
                                 fov_pix_2)  # TODO: separate calculation of fov_pix from their visualization!
            """

            # Convert translation vector to WGS84 coordinates
            # Translate relative to top left corner, not principal point/center of map raster
            h, w = img_dimensions
            t[0] = (1 - t[0]) * w / 2
            t[1] = (1 - t[1]) * h / 2
            cam_pos_wgs84, cam_pos_wgs84_uncropped, cam_pos_wgs84_unrotated = convert_fov_from_pix_to_wgs84(  # TODO: break this func into an array and single version?
                np.array(t[0:2].reshape((1, 1, 2))), map_dims_with_padding,
                self._map_bbox, rot, img_dimensions)
            cam_pos_wgs84 = cam_pos_wgs84.squeeze()  # TODO: eliminate need for this squeeze
            # TODO: turn cam_pos_wgs84 into a LatLonAlt
            # TODO: something is wrong with camera_altitude - should be a scalar but is array
            lalt = LatLonAlt(*(tuple(cam_pos_wgs84) + (camera_altitude,)))  # TODO: alt should not be None? Use LatLon instead?
            image_frame.position = lalt

            fov_gimbal = self._gimbal_fov_wgs84
            write_fov_and_camera_location_to_geojson(fov_wgs84, cam_pos_wgs84, (map_lat, map_lon, camera_distance),
                                                     fov_gimbal)

            # Compute position (meters) and velocity (meters/second) in local frame
            local_position = None
            local_frame_origin_latlonalt = self._vehicle_local_position_ref_latlonalt()
            if local_frame_origin_latlonalt is not None:
                local_position = distances(local_frame_origin_latlonalt, LatLon(*tuple(cam_pos_wgs84))) \
                                 + (camera_altitude,)  # TODO: see lalt and set_esitmated_camera_position call above - should not need to do this twice?
            else:
                self.get_logger().debug(f'Could not get local frame origin - will not compute local position.')

            velocity = None
            # TODO: Make it so that previous global position can be fetched without risk of mixing the order of these operations (e.g. use timestamps and/or frame_id or something).
            if self.previous_image_frame is not None:
                assert self.previous_image_frame.position is not None, f'Previous camera position was unexpectedly None.'  # TODO: is it possible that this is None? Need to do warning instead of assert?
                assert image_frame.position is not None, f'Current camera position was unexpectedly None.'  # TODO: is it possible that this is None? Need to do warning instead of assert?
                assert hasattr(self.previous_image_frame, 'stamp'),\
                    'Previous image frame timstamp not found.'  # TODO: is this assertion needed?

                # TODO: refactor this assertion so that it's more compact
                if self.previous_image_frame.stamp.sec == image_frame.stamp.sec:
                    assert self.previous_image_frame.stamp.nanosec < image_frame.stamp.nanosec, \
                        f'Previous image frame timestamp {self.previous_image_frame.stamp} was >= than ' \
                        f'current image frame timestamp {image_frame.stamp}.'
                else:
                    assert self.previous_image_frame.stamp.sec < image_frame.stamp.sec,\
                        f'Previous image frame timestamp {self.previous_image_frame.stamp} was >= than ' \
                        f'current image frame timestamp {image_frame.stamp}.'
                time_difference = image_frame.stamp.sec - self.previous_image_frame.stamp.sec
                if time_difference == 0:
                    time_difference = (image_frame.stamp.nanosec -
                                       self.previous_image_frame.stamp.nanosec) / 1e9
                assert time_difference > 0, f'Time difference between frames was 0.'
                x_dist, y_dist = distances(image_frame.position, self.previous_image_frame.position)  # TODO: compute x,y,z components separately!
                z_dist = image_frame.position.alt - self.previous_image_frame.position.alt
                dist = (x_dist, y_dist, z_dist)
                assert all(isinstance(x, float) for x in dist), f'Expected all float values for distance: {dist}.'  # TODO: z could be None/NaN - handle it!
                velocity = tuple(x / time_difference for x in dist)
            else:
                self.get_logger().warning(f'Could not get previous image frame stamp - will not compute velocity.')

            self.get_logger().debug(f'Local frame position: {local_position}, velocity: {velocity}.')
            self.get_logger().debug(f'Local frame origin: {self._vehicle_local_position_ref_latlonalt()}.')
            self._publish_vehicle_visual_odometry(local_position, velocity)  # TODO: enable

            self.previous_image_frame = image_frame

        except Exception as e:
            self.get_logger().error('Matching returned exception: {}\n{}'.format(e, traceback.print_exc()))


def main(args=None):
    rclpy.init(args=args)
    matcher = Matcher(share_dir, superglue_dir)
    rclpy.spin(matcher)
    matcher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
