import rclpy
import sys
import os
import traceback
import yaml
import importlib
import math
import time

from pyproj import Geod, Proj, transform
from typing import Optional, Union, Tuple, get_args
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from owslib.wms import WebMapService
from cv2 import VideoCapture, imwrite, imdecode
import numpy as np
import cv2
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation
from functools import partial
from wms_map_matching.util import setup_sys_path, convert_fov_from_pix_to_wgs84, \
    write_fov_and_camera_location_to_geojson, get_bbox_center, BBox, Dim, rotate_and_crop_map, \
    visualize_homography, get_fov, get_camera_distance, LatLon, fov_to_bbox, \
    get_angle, create_src_corners, uncrop_pixel_coordinates, rotate_point, RPY, LatLonAlt, \
    ImageFrame, assert_type, assert_ndim, assert_len, assert_shape, assert_first_stamp_greater, MapFrame

from px4_msgs.msg import VehicleVisualOdometry, VehicleAttitude, VehicleLocalPosition, VehicleGlobalPosition, \
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

    # Maximum radius for map requests (in meters)
    MAX_MAP_RADIUS = 1000  # TODO: put default into config.yaml, make them all ROS params so they can be changed on the fly

    # Ellipsoid model used by pyproj
    PYPROJ_ELLIPSOID = 'WGS84'

    # Fetched map rasters are squares that enclose a circle of given radius
    MAP_RADIUS_METERS_DEFAULT = 400  # in meters

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

    def __init__(self, share_directory: str, superglue_directory: str, config: str = 'config.yml') -> None:
        """Initializes the node.

        Arguments:
            share_dir - String path of the share directory where configuration and other files are.
            superglue_dir - String path of the directory where SuperGlue related files are.
            config - String path to the config file in the share folder.
        """
        super().__init__('matcher')
        assert_type(str, share_directory)
        assert_type(str, superglue_directory)
        assert_type(str, config)
        self._share_dir = share_directory
        self._superglue_dir = superglue_directory

        # Setup config
        self._config = None
        self._load_config(config)

        # Setup WMS server
        self._wms = None
        self._init_wms()

        # Dict for storing all microRTPS bridge subscribers and publishers
        self._topics = {self.PUBLISH_KEY: {}, self.SUBSCRIBE_KEY: {}}
        self._setup_topics()

        # Converts image_raw to cv2 compatible image
        self._cv_bridge = CvBridge()

        # Setup SuperGlue
        self._superglue = None
        self._setup_superglue()

        self._gimbal_fov_wgs84 = []  # TODO: remove this attribute, just passing it through here from _update_map to _match (temp hack)

        # To be used for pyproj transformations
        self._geod = Geod(ellps=self.PYPROJ_ELLIPSOID)

        # self._image_frame = None  # Not currently used / needed
        self._previous_image_frame = None  # ImageFrame from previous match, needed to compute velocity
        self._map_frame = None  # Map raster received from WMS endpoint here along with its bounding box
        # self._previous_map_frame  # Todo: to be used by _should_update_map()

        # Properties that are mapped to microRTPS topics
        self._camera_info = None
        self._vehicle_local_position = None
        self._vehicle_global_position = None
        self._vehicle_attitude = None
        self._gimbal_device_attitude_status = None
        self._gimbal_device_set_attitude = None

    @property
    def config(self) -> dict:
        return self._config

    @config.setter
    def config(self, value: dict) -> None:
        assert_type(dict, value)
        self._config = value

    @property
    def wms(self) -> WebMapService:
        return self._wms

    @wms.setter
    def wms(self, value: WebMapService) -> None:
        # TODO: is this setter even needed, this is done during Matcher initialization?
        # assert_type(WebMapService, value)
        # TODO: check that this is correct type - needs a bit more work than above,
        #  example: <owslib.map.wms111.WebMapService_1_1_1 object at 0x7f3e7e4e3e50>
        self._wms = value

    @property
    def superglue(self) -> SuperGlue:
        return self._superglue

    @superglue.setter
    def superglue(self, value: SuperGlue) -> None:
        assert_type(SuperGlue, value)
        self._superglue = value

    @property
    def geod(self) -> Geod:
        return self._geod

    @property
    def share_dir(self) -> str:
        return self._share_dir

    @property
    def superglue_dir(self) -> str:
        return self._superglue_dir

    @property
    def map_frame(self) -> MapFrame:
        return self._map_frame

    @map_frame.setter
    def map_frame(self, value: MapFrame) -> None:
        assert_type(MapFrame, value)
        self._map_frame = value

    @property
    def previous_image_frame(self) -> ImageFrame:
        return self._previous_image_frame

    @previous_image_frame.setter
    def previous_image_frame(self, value: ImageFrame) -> None:
        assert_type(ImageFrame, value)
        self._previous_image_frame = value

    @property
    def camera_info(self) -> CameraInfo:
        return self._camera_info

    @camera_info.setter
    def camera_info(self, value: CameraInfo) -> None:
        assert_type(CameraInfo, value)
        self._camera_info = value

    @property
    def vehicle_local_position(self) -> VehicleLocalPosition:
        return self._vehicle_local_position

    @vehicle_local_position.setter
    def vehicle_local_position(self, value: VehicleLocalPosition) -> None:
        assert_type(VehicleLocalPosition, value)
        self._vehicle_local_position = value

    @property
    def vehicle_global_position(self) -> VehicleGlobalPosition:
        return self._vehicle_global_position

    @vehicle_global_position.setter
    def vehicle_global_position(self, value: VehicleGlobalPosition) -> None:
        assert_type(VehicleGlobalPosition, value)
        self._vehicle_global_position = value

    @property
    def vehicle_attitude(self) -> VehicleAttitude:
        return self._vehicle_attitude

    @vehicle_attitude.setter
    def vehicle_attitude(self, value: VehicleAttitude) -> None:
        assert_type(VehicleAttitude, value)
        self._vehicle_attitude = value

    @property
    def gimbal_device_attitude_status(self) -> GimbalDeviceAttitudeStatus:
        return self._gimbal_device_attitude_status

    @gimbal_device_attitude_status.setter
    def gimbal_device_attitude_status(self, value: GimbalDeviceAttitudeStatus) -> None:
        assert_type(GimbalDeviceAttitudeStatus, value)
        self._gimbal_device_attitude_status = value

    @property
    def gimbal_device_set_attitude(self) -> GimbalDeviceSetAttitude:
        return self._gimbal_device_set_attitude

    @gimbal_device_set_attitude.setter
    def gimbal_device_set_attitude(self, value: GimbalDeviceSetAttitude) -> None:
        assert_type(GimbalDeviceSetAttitude, value)
        self._gimbal_device_set_attitude = value

    def _setup_superglue(self) -> None:
        """Sets up SuperGlue."""
        self.superglue = SuperGlue(self._config['superglue'], self.get_logger())

    def _load_config(self, yaml_file: str) -> None:
        """Loads config from the provided YAML file."""
        assert_type(str, yaml_file)
        with open(os.path.join(self.share_dir, yaml_file), 'r') as f:
            try:
                self.config = yaml.safe_load(f)
                self.get_logger().info(f'Loaded config:\n{self.config}.')
            except Exception as e:
                self.get_logger().error(f'Could not load config file {yaml_file} because of exception:'
                                        f'\n{e}\n{traceback.print_exc()}')

    def _use_gimbal_projection(self) -> bool:
        """Returns True if gimbal projection is enabled for fetching map bbox rasters."""
        gimbal_projection_flag = self._config.get('misc', {}).get('gimbal_projection', False)
        if type(gimbal_projection_flag) is bool:
            return gimbal_projection_flag
        else:
            self.get_logger().warn(f'Could not read gimbal projection flag: {gimbal_projection_flag}. Assume False.')
            return False

    def _restrict_affine(self) -> bool:
        """Returns True if homography matrix should be restricted to an affine transformation (nadir facing camera)."""
        restrict_affine_flag = self._config.get('misc', {}).get('affine', False)
        if type(restrict_affine_flag) is bool:
            return restrict_affine_flag
        else:
            self.get_logger().warn(f'Could not read affine restriction flag: {restrict_affine_flag}. Assume False.')
            return False

    def _setup_topics(self) -> None:
        """Creates publishers and subscribers for microRTPS bridge topics."""
        for topic in self.TOPICS:
            topic_name = topic.get(self.TOPIC_NAME_KEY, None)
            class_ = topic.get(self.CLASS_KEY, None)
            assert topic_name is not None, f'Topic name not provided in topic: {topic}.'
            assert class_ is not None, f'Class not provided in topic: {topic}.'

            publish = topic.get(self.PUBLISH_KEY, None)
            if publish is not None:
                assert_type(bool, publish)
                # TODO: this just overwrites previous publish_key
                self._topics.get(self.PUBLISH_KEY).update({topic_name: self._create_publisher(topic_name, class_)})

            subscribe = topic.get(self.SUBSCRIBE_KEY, None)
            if subscribe is not None:
                assert_type(bool, subscribe)
                # TODO: this just overwrites previous subscribe_key
                self._topics.get(self.SUBSCRIBE_KEY).update({topic_name: self._create_subscriber(topic_name, class_)})

        self.get_logger().info(f'Topics setup complete:\n{self._topics}.')

    def _create_publisher(self, topic_name: str, class_: object) -> rclpy.publisher.Publisher:
        """Sets up an rclpy publisher."""
        return self.create_publisher(class_, topic_name, 10)

    def _create_subscriber(self, topic_name: str, class_: object) -> rclpy.subscription.Subscription:
        """Sets up an rclpy subscriber."""
        callback_name = topic_name.lower() + '_callback'
        callback = getattr(self, callback_name, None)
        assert callback is not None, f'Missing callback implementation for {callback_name}.'
        return self.create_subscription(class_, topic_name, callback, 10)

    def _init_wms(self) -> None:
        """Initializes the Web Map Service (WMS) client used by the node to request map rasters.

        The url and version parameters are required to initialize the WMS client and are therefore set to read only. The
        layer and srs parameters can be changed dynamically.
        """
        self.declare_parameter('url', self._config['wms']['url'], ParameterDescriptor(read_only=True))
        self.declare_parameter('version', self._config['wms']['version'], ParameterDescriptor(read_only=True))
        self.declare_parameter('layer', self._config['wms']['layer'])
        self.declare_parameter('srs', self._config['wms']['srs'])

        try:
            self.wms = WebMapService(self.get_parameter('url').get_parameter_value().string_value,
                                     version=self.get_parameter('version').get_parameter_value().string_value)
        except Exception as e:
            self.get_logger().error('Could not connect to WMS server.')
            raise e

    def _get_bbox(self, latlon: Union[LatLon, LatLonAlt], radius_meters: Union[int, float] = MAP_RADIUS_METERS_DEFAULT)\
            -> BBox:
        """Gets the bounding box containing a circle with given radius centered at given lat-lon fix.

        Uses azimuthal equidistant projection. Based on Mike T's answer at
        https://gis.stackexchange.com/questions/289044/creating-buffer-circle-x-kilometers-from-point-using-python.

        Arguments:
            latlon: The lat-lon tuple (EPSG:4326) for the circle.
            radius_meters: Radius in meters of the circle.

        Returns:
            The bounding box (left, bottom, right, top).
        """
        assert_type(get_args(Union[LatLon, LatLonAlt]), latlon)
        assert_type(get_args(Union[int, float]), radius_meters)
        ul = self._move_distance(latlon, (-45, radius_meters))
        lr = self._move_distance(latlon, (135, radius_meters))
        return BBox(ul.lon, lr.lat, lr.lon, ul.lat)

    def _get_distance_of_fov_center(self, fov_wgs84: np.ndarray) -> float:
        """Calculate distance between middle of sides of FoV based on triangle similarity."""
        midleft = ((fov_wgs84[0] + fov_wgs84[1]) * 0.5).squeeze()
        midright = ((fov_wgs84[2] + fov_wgs84[3]) * 0.5).squeeze()
        _, __, dist = self.geod.inv(midleft[1], midleft[0], midright[1], midright[0])  # TODO: use distance method here
        return dist

    def _distances(self, latlon1: Union[LatLon, LatLonAlt], latlon2: Union[LatLon, LatLonAlt]) -> Tuple[float, float]:
        """Calculate distance in meters in x and y dimensions of two LatLons."""
        assert_type(get_args(Union[LatLon, LatLonAlt]), latlon1)
        assert_type(get_args(Union[LatLon, LatLonAlt]), latlon2)
        lats1 = (latlon1.lat, latlon1.lat)
        lons1 = (latlon1.lon, latlon1.lon)
        lats2 = (latlon1.lat, latlon2.lat)  # Lon difference for first, lat difference for second --> y, x
        lons2 = (latlon2.lon, latlon1.lon)
        _, __, dist = self.geod.inv(lons1, lats1, lons2, lats2)

        # invert order to x, y (lat diff, lon diff in meters) in NED frame dimensions,
        # also invert X axis so that it points north
        dist = (-dist[1], dist[0])
        return dist

    #def distance(self, latlon1: Union[LatLon, LatLonAlt], latlon2: Union[LatLon, LatLonAlt]) -> float:
    #    """Calculate distance in meters between two latlons."""
    #    _, __, dist = self.g.inv(latlon1.lon, latlon1.lat, latlon2.lon, latlon2.lat)
    #    return dist

    def _move_distance(self, latlon: Union[LatLon, LatLonAlt], azmth_dist: Tuple[Union[int, float], Union[int, float]])\
            -> LatLon:
        """Returns LatLon given distance in the direction of azimuth (degrees) from original point."""
        assert_type(tuple, azmth_dist)
        assert_type(get_args(Union[LatLon, LatLonAlt]), latlon)  # TODO: accept both LatLon and LatLonAlt
        azmth, dist = azmth_dist  # TODO: silly way of providing these args just to map over a zipped list in _update_map, fix it
        assert_type(get_args(Union[int, float]), azmth)
        assert_type(get_args(Union[int, float]), dist)
        lon, lat, azmth = self.geod.fwd(latlon.lon, latlon.lat, azmth, dist)
        return LatLon(lat, lon)

    def _map_size_with_padding(self) -> Optional[Tuple[int, int]]:
        dim = self._img_dim()
        if dim is None:
            self.get_logger().warn(f'Dimensions not available - returning None as map size.')
            return None
        assert_type(Dim, dim)
        diagonal = math.ceil(math.sqrt(dim.width ** 2 + dim.height ** 2))
        assert_type(int, diagonal)  # TODO: What if this is float?
        return diagonal, diagonal

    def _map_dim_with_padding(self) -> Optional[Dim]:
        map_size = self._map_size_with_padding()
        if map_size is None:
            self.get_logger().warn(f'Map size with padding not available - returning None as map dimensions.')
            return None
        assert_type(tuple, map_size)
        assert_len(map_size, 2)
        return Dim(*map_size)

    def _declared_img_size(self) -> Optional[Tuple[int, int]]:
        """Returns image resolution size as it is declared in the latest CameraInfo message."""
        if self.camera_info is not None:
            return self.camera_info.height, self.camera_info.width  # numpy order: h, w, c --> height first
        else:
            self.get_logger().warn('Camera info was not available - returning None as declared image size.')
            return None

    def _img_dim(self) -> Optional[Dim]:
        declared_size = self._declared_img_size()
        if declared_size is None:
            self.get_logger().warn('CDeclared size not available - returning None as image dimensions.')
            return None
        assert_type(tuple, declared_size)
        assert_len(declared_size, 2)
        return Dim(*declared_size)

    def _project_gimbal_fov(self, altitude_meters: float) -> Optional[np.ndarray]:
        """Returns field of view BBox projected using gimbal attitude and camera intrinsics information."""
        rpy = self._get_camera_rpy()
        if rpy is None:
            self.get_logger().warn('Could not get RPY - cannot project gimbal fov.')
            return None

        r = Rotation.from_euler(self.EULER_SEQUENCE, list(rpy), degrees=True).as_matrix()
        e = np.hstack((r, np.expand_dims(np.array([0, 0, altitude_meters]), axis=1)))
        assert_shape(e, (3, 4))

        if self.camera_info is None:
            self.get_logger().warn('Could not get camera info - cannot project gimbal fov.')
            return None
        h, w = self._img_dim()
        # TODO: assert h w not none and integers? and divisible by 2?

        # Intrinsic matrix
        k = np.array(self.camera_info.k).reshape([3, 3])

        # Project image corners to z=0 plane (ground)
        src_corners = create_src_corners(h, w)
        assert_shape(src_corners, (4, 1, 2))

        e = np.delete(e, 2, 1)  # Remove z-column, making the matrix square
        p = np.matmul(k, e)
        p_inv = np.linalg.inv(p)
        assert_shape(p_inv, (3, 3))

        dst_corners = cv2.perspectiveTransform(src_corners, p_inv)  # TODO: use util.get_fov here?
        assert_shape(dst_corners, src_corners.shape)
        dst_corners = dst_corners.squeeze()  # See get_fov usage elsewhere -where to do squeeze if at all?

        return dst_corners

    # TODO: this has no usages? Remove?
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

    def _projected_field_of_view_center(self, origin: LatLonAlt) -> Optional[LatLon]:
        """Return WGS84 coordinates of projected camera field of view."""
        if self.camera_info is not None:
            gimbal_fov_pix = self._project_gimbal_fov(origin.alt)

            # Convert gimbal field of view from pixels to WGS84 coordinates
            if gimbal_fov_pix is not None:
                azmths = list(map(lambda x: math.degrees(math.atan2(x[0], x[1])), gimbal_fov_pix))
                dists = list(map(lambda x: math.sqrt(x[0] ** 2 + x[1] ** 2), gimbal_fov_pix))
                zipped = list(zip(azmths, dists))
                to_wgs84 = partial(self._move_distance, origin)
                self._gimbal_fov_wgs84 = np.array(list(map(to_wgs84, zipped)))
                ### TODO: add some sort of assertion hat projected FoV is contained in size and makes sense

                # Use projected field of view center instead of global position as map center
                map_center_latlon = get_bbox_center(fov_to_bbox(self._gimbal_fov_wgs84))
            else:
                self.get_logger().warn('Could not project camera FoV, getting map raster assuming nadir-facing camera.')
                return None
        else:
            self.get_logger().debug('Camera info not available, cannot project FoV, defaulting to global position.')
            return None

        return map_center_latlon

    def _update_map(self, center: LatLon, radius: int) -> None:
        """Gets latest map from WMS server for given location and radius and saves it."""
        assert_type(LatLon, center)
        assert_type(int, radius)
        assert 0 < radius < self.MAX_MAP_RADIUS, f'Radius should be between 0 and {self.MAX_MAP_RADIUS}.'

        bbox = self._get_bbox(center)
        assert_type(BBox, bbox)

        # Build and send WMS request
        layer_str = self.get_parameter('layer').get_parameter_value().string_value
        srs_str = self.get_parameter('srs').get_parameter_value().string_value
        assert_type(str, layer_str)
        assert_type(str, srs_str)
        try:
            self.get_logger().info(f'Getting map for bbox: {bbox}, layer: {layer_str}, srs: {srs_str}.')
            map_ = self.wms.getmap(layers=[layer_str], srs=srs_str, bbox=bbox, size=self._map_size_with_padding(),
                                    format='image/png', transparent=True)
        except Exception as e:
            self.get_logger().warn(f'Exception from WMS server query:\n{e},\n{traceback.print_exc()}.')
            return None

        # Decode response from WMS server
        map_ = np.frombuffer(map_.read(), np.uint8)
        map_ = imdecode(map_, cv2.IMREAD_UNCHANGED)
        assert_type(np.ndarray, map_)
        assert_ndim(map_, 3)
        self.map_frame = MapFrame(center, radius, bbox, map_)
        assert self.map_frame.image.shape[0:2] == self._map_size_with_padding(), \
            'Decoded map is not the specified size.'

    def image_raw_callback(self, msg: Image) -> None:
        """Handles latest image frame from camera."""
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

    def _camera_yaw(self) -> Optional[int]:
        """Returns camera yaw in degrees."""
        rpy = self._get_camera_rpy()
        if rpy is None:
            self.get_logger().warn(f'Could not get camera RPY - cannot return yaw.')
            return None

        assert_type(RPY, rpy)
        camera_yaw = rpy.yaw
        return camera_yaw

    def _get_camera_rpy(self) -> Optional[RPY]:
        """Returns roll-pitch-yaw euler vector."""
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
        if self.vehicle_local_position is None:
            self.get_logger().warn('VehicleLocalPosition is unknown, cannot get heading. Cannot return RPY.')
            return None

        heading = self.vehicle_local_position.heading
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
        return self.EULER_SEQUENCE.lower().find('y')

    def _yaw_index(self) -> int:
        return self.EULER_SEQUENCE.lower().find('x')

    def camera_info_callback(self, msg: CameraInfo) -> None:
        """Handles latest camera info message."""
        self.get_logger().debug(f'Camera info received:\n{msg}.')
        self.camera_info = msg
        camera_info_topic = self._topics.get(self.SUBSCRIBE_KEY, {}).get('camera_info', None)
        if camera_info_topic is not None:
            self.get_logger().warn('Assuming camera_info is static - destroying the subscription.')
            camera_info_topic.destroy()

    def vehiclelocalposition_pubsubtopic_callback(self, msg: VehicleLocalPosition) -> None:
        """Handles latest VehicleLocalPosition message."""
        self.vehicle_local_position = msg

    def _get_dynamic_map_radius(self) -> int:
        """Returns map radius that determines map size for WMS map requests."""
        return self.MAP_RADIUS_METERS_DEFAULT  # TODO: assume constant, figure out an algorithm to adjust this dynamically

    def vehicleglobalposition_pubsubtopic_callback(self, msg: VehicleGlobalPosition) -> None:
        """Handles latest VehicleGlobalPosition message."""
        self.vehicle_global_position = msg  # TODO: seems like self.vehicle_global_position property is never accessed currently, info only needed here to trigger _update_map? Comment out this variabhle completely?
        center = LatLon(msg.lat, msg.lon)
        origin = LatLonAlt(*(center + (msg.alt,)))
        if self._use_gimbal_projection():
            projected_center = self._projected_field_of_view_center(origin)
            if projected_center is None:
                self.get_logger().warn('Could not project field of view center. Using global position for map instead.')
            else:
                center = projected_center
        if self._should_update_map(center):
            self._update_map(center, self._get_dynamic_map_radius())

    def _should_update_map(self, center: LatLon) -> bool:
        """Returns true if map should be updated."""
        # TODO: based on some config variable, do not update map if center is very close to previous map frame center
        return True  # TODO: placeholder return value always True

    def gimbaldeviceattitudestatus_pubsubtopic_callback(self, msg: GimbalDeviceAttitudeStatus) -> None:
        """Handles latest GimbalDeviceAttitudeStatus message."""
        self.gimbal_device_attitude_status = msg

    def gimbaldevicesetattitude_pubsubtopic_callback(self, msg: GimbalDeviceSetAttitude) -> None:
        """Handles latest GimbalDeviceSetAttitude message."""
        self.gimbal_device_set_attitude = msg

    def vehicleattitude_pubsubtopic_callback(self, msg: VehicleAttitude) -> None:
        """Handles latest VehicleAttitude message."""
        self.vehicle_attitude = msg

    def _publish_vehicle_visual_odometry(self, position: tuple, velocity: tuple) -> None:
        """Publishes a VehicleVisualOdometry message over the microRTPS bridge as defined in
        https://github.com/PX4/px4_msgs/blob/master/msg/VehicleVisualOdometry.msg. """
        module_name = 'px4_msgs.msg'  # TODO: get ffrom config file
        class_name = 'VehicleVisualOdometry'  # TODO: get from config file or look at _import_class stuff in this file
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
            assert len(
                position) == 3, f'Unexpected length for position estimate: {len(position)} (3 expected).'  # TODO: can also be length 2 if altitude is not published, handle that
            assert all(isinstance(x, float) for x in position), f'Position contained non-float elements.'
            # TODO: check for np.float32?
            msg.x, msg.y, msg.z = position  # float32 North, East, Down
        else:
            self.get_logger().warn('Position tuple was None - publishing NaN as position.')
            msg.x, msg.y, msg.z = (float('nan'),) * 3  # float32 North, East, Down

        # Attitude quaternions - not used
        msg.q = (float('nan'),) * 4  # float32
        msg.q_offset = (float('nan'),) * 4
        msg.pose_covariance = (float('nan'),) * 21

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
            msg.vx, msg.vy, msg.vz = (float('nan'),) * 3  # float32 North, East, Down

        # Angular velocity - not used
        msg.rollspeed, msg.pitchspeed, msg.yawspeed = (float('nan'),) * 3  # float32 TODO: remove redundant np.float32?
        msg.velocity_covariance = (float('nan'),) * 21  # float32 North, East, Down

    def _camera_pitch(self) -> Optional[int]:
        """Returns camera pitch in degrees relative to vehicle frame."""
        rpy = self._get_camera_rpy()
        if rpy is None:
            self.get_logger().warn('Gimbal RPY not available, cannot compute camera pitch.')
            return None
        assert_type(RPY, rpy)
        return rpy.pitch

    def _gimbal_attitude(self) -> Optional[Union[GimbalDeviceAttitudeStatus, GimbalDeviceSetAttitude]]:
        """Returns 1. GimbalDeviceAttitudeStatus, or 2. GimbalDeviceSetAttitude if 1. is not available."""
        gimbal_attitude = self.gimbal_device_attitude_status
        if gimbal_attitude is None:
            self.get_logger().warn('GimbalDeviceAttitudeStatus not available. Trying GimbalDeviceSetAttitude instead.')
            gimbal_attitude = self.gimbal_device_set_attitude
            if gimbal_attitude is None:
                self.get_logger().warn('GimbalDeviceSetAttitude not available. Gimbal attitude status not available.')
        return gimbal_attitude

    @staticmethod
    def _find_and_decompose_homography(mkp_img: np.ndarray, mkp_map: np.ndarray, k: np.ndarray,
                                       camera_normal: np.ndarray, reproj_threshold: float = 1.0,
                                       method: int = cv2.RANSAC, affine: bool = False) -> Tuple[np.ndarray, np.ndarray,
                                                                                                np.ndarray, np.ndarray]:
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
        assert_type(np.ndarray, mkp_img)
        assert_type(np.ndarray, mkp_map)
        assert len(mkp_img) >= min_points and len(mkp_map) >= min_points, 'Four points needed to estimate homography.'

        assert_type(bool, affine)
        assert_type(int, method)
        assert_type(float, reproj_threshold)
        if not affine:
            h, h_mask = cv2.findHomography(mkp_img, mkp_map, method, reproj_threshold)
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

    def _local_frame_position(self, local_frame_origin: LatLonAlt, camera_position: LatLon, camera_altitude: float) \
            -> Tuple[float, float, float]:
        """Returns camera position tuple (x, y) in meters in local frame."""
        assert_type(LatLonAlt, local_frame_origin)
        assert_type(LatLon, camera_position)
        assert_type(float, camera_altitude)
        return self._distances(local_frame_origin, camera_position) + (camera_altitude,)

    def _local_frame_velocity(self, image_frame: ImageFrame, previous_image_frame: ImageFrame)\
            -> Tuple[float, float, Optional[float]]:
        """Computes velocity in meters per second for position between two image frames."""
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

    def _match_inputs(self) -> Tuple[bool, Tuple[np.ndarray, LatLonAlt, CameraInfo, np.ndarray, np.ndarray,
                                                 float, float, Dim, Dim, bool, Optional[np.ndarray]]]:
        """Returns success, data where success is False if there are any Nones in the list.

        This performs a check that all required data is available for performing a _match.

        Data consists of:
            map_frame - np.darray map_frame to match
            local_frame_origin_position - LatLonAlt origin of local frame global frame WGS84
            camera_info - CameraInfo
            gimbal_fov_wgs4 - np.ndarray Gimbal FoV projection to ground
            camera_normal - np.ndarray Camera normal unit vector
            camera_yaw - float
            camera_pitch - float
            map_dim_with_padding - Dim map dimensions including padding for rotation
            img_dim - Dim image dimensions
            restrict_affine - bool flag indicating whether homography matrix should be restricted to 2D affine tform
            previous_image_frame - Optional[np.ndarray], previous image frame, if available, None otherwise
        """
        # Vehicle local frame global reference position
        local_frame_origin_position = self._vehicle_local_position_ref_latlonalt()

        # Camera information
        camera_normal, camera_yaw, camera_pitch = self._camera_normal(), self._camera_yaw(), self._camera_pitch()

        # Image and map raster dimensions
        map_dim_with_padding, img_dim = self._map_dim_with_padding(), self._img_dim()

        # Should homography be restricted to 2D affine transformation
        restrict_affine = self._restrict_affine()

        # Make sure all info is available before attempting to match
        # TODO: _gimbal_fov_wgs84 should be handled in some better way - now we have a property jsut to pass it from update_map-> project_gimabal_fov to matcher (and matcher just writes it into file, does nothing else with it, could be written where it is created!)
        required_info = (self.map_frame, local_frame_origin_position, self.camera_info, self._gimbal_fov_wgs84,
                         camera_normal, camera_yaw, camera_pitch, map_dim_with_padding, img_dim, restrict_affine)
        optional_info = (self.previous_image_frame, )

        if not all(x is not None for x in required_info):
            self.get_logger().warn(f'At least one of following was None: {required_info}. Cannot do matching.')
            return False, required_info + optional_info
        else:
            assert -180 <= camera_yaw <= 180, f'Unexpected gimbal yaw value: {camera_yaw} ([-180, 180] expected).'
            return True, required_info + optional_info

    @staticmethod
    def _compute_camera_position(t: np.ndarray, map_dim_with_padding: Dim, bbox: BBox, camera_yaw: float, img_dim: Dim)\
            -> LatLon:
        """Returns camera position based on translation vector and metadata """
        # This computse 1. position and attaches it to image_frame
        # Convert translation vector to WGS84 coordinates
        # Translate relative to top left corner, not principal point/center of map raster
        t[0] = (1 - t[0]) * img_dim.width / 2
        t[1] = (1 - t[1]) * img_dim.height / 2
        # TODO: break this func into an array and single version?
        cam_pos_wgs84, cam_pos_wgs84_uncropped, cam_pos_wgs84_unrotated = convert_fov_from_pix_to_wgs84(
            np.array(t[0:2].reshape((1, 1, 2))), map_dim_with_padding, bbox, camera_yaw, img_dim)
        cam_pos_wgs84 = cam_pos_wgs84.squeeze()  # TODO: eliminate need for this squeeze
        # TODO: something is wrong with camera_altitude - should be a scalar but is array
        #latlonalt = LatLonAlt(
        #    *(tuple(cam_pos_wgs84) + (camera_altitude,)))  # TODO: alt should not be None? Use LatLon instead?
        latlon = LatLon(*tuple(cam_pos_wgs84))
        return latlon

    def _compute_camera_distance(self, fov_wgs84: np.ndarray, focal_length: float, img_dim: Dim) -> float:
        assert_type(np.ndarray, fov_wgs84)
        assert_type(float, focal_length)
        assert_type(Dim, img_dim)
        fov_center_line_length = self._get_distance_of_fov_center(fov_wgs84)
        camera_distance = get_camera_distance(focal_length, img_dim.width, fov_center_line_length)  # TODO: move logic here, this is only place where this util function is used?
        assert_type(float, camera_distance)
        return camera_distance

    @staticmethod
    def _compute_camera_altitude(camera_distance: float, camera_pitch: float) -> float:
        """Computes camera altitude based on input."""
        camera_altitude = math.cos(math.radians(
            camera_pitch)) * camera_distance  # TODO: use rotation from decomposeHomography for getting the pitch in this case (use visual info, not from sensors)
        #self.get_logger().debug(f'Camera pitch {camera_pitch} deg, camera yaw {camera_yaw}rd, distance to principal'
        #                        f'point {camera_distance} m, altitude {camera_altitude} m.')
        return camera_altitude

    # TODO: the 'output' of this function is that it attaches stuff to image_frame? or what does it do?
    # Current tasks too many:
    # 1. attach fov and position to image_frame
    # 2. COmpute and publish position and velocity,
    # 3. Visualize homography,
    # 4. Output fov and position as geoJSON
    def _match(self, image_frame: np.ndarray, map_frame: np.ndarray, local_frame_origin_position: LatLonAlt,
               camera_info: CameraInfo, gimbal_fov_wgs84: np.ndarray, camera_normal: np.ndarray, camera_yaw: float,
               camera_pitch: float, map_dim_with_padding: Dim, img_dim: Dim, restrict_affine: bool,
               previous_image_frame: Optional[np.ndarray]) -> None:
        """Matches camera image to map image and computes camera position and field of view."""
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

            # Center of bbox
            #map_lat, map_lon = get_bbox_center(BBox(*map_frame.bbox))

            # Compute camera altitude, and distance to principal point using triangle similarity
            # TODO: _update_map or _project_gimbal_fov_center has similar logic used in gimbal fov projection, try to combine
            camera_distance = self._compute_camera_distance(fov_wgs84, k[0][0], img_dim)
            camera_altitude = self._compute_camera_altitude(camera_distance, camera_pitch)

            position = self._compute_camera_position(t, map_dim_with_padding, map_frame.bbox, camera_yaw, img_dim)
            local_position = self._local_frame_position(local_frame_origin_position, position, camera_altitude)
            image_frame.position = LatLonAlt(*(position + (camera_altitude,)))  # TODO: alt should not be None? Use LatLon instead?  # TODO: move to _compute_camera_position?

            # The stuff below is all 2. and 4. (not 1. nor 3.)

            # Write data to GeoJSON for visualization and debugging in external GIS software
            #write_fov_and_camera_location_to_geojson(fov_wgs84, cam_pos_wgs84, (map_lat, map_lon, camera_distance),
            #                                         gimbal_fov_wgs84)

            velocity = None
            if previous_image_frame is not None:
                velocity = self._local_frame_velocity(image_frame, previous_image_frame)
            else:
                self.get_logger().warning(f'Could not get previous image frame stamp - will not compute velocity.')

            self.get_logger().debug(f'Local frame position: {local_position}, velocity: {velocity}.')
            self.get_logger().debug(f'Local frame origin: {local_frame_origin_position}.')
            self._publish_vehicle_visual_odometry(local_position, velocity)

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
