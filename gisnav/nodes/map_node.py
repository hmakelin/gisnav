"""Contains :class:`.Node` that provides :class:`OrthoImage3D`s"""
import numpy as np
import cv2
import rclpy
import math
import time

from typing import Optional, Tuple, Union, get_args

from shapely.geometry import box
from pygeodesy.geoids import GeoidPGM

from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from rclpy.service import Service
from rclpy.timer import Timer
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.qos import QoSPresetProfiles

from sensor_msgs.msg import CameraInfo
from geographic_msgs.msg import GeoPoint as ROSGeoPoint, BoundingBox
from mavros_msgs.msg import Altitude as ROSAltitude
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

from owslib.wms import WebMapService
from owslib.util import ServiceException

from cv_bridge import CvBridge

from gisnav.assertions import assert_len, assert_type, assert_ndim
from gisnav_msgs.msg import OrthoImage3D
from gisnav_msgs.srv import GetMap
from gisnav.data import BBox, MapData, CameraData, Dim, Img
from gisnav.geo import GeoSquare, GeoPoint



class MapNode(Node):
    """Publishes :class:`.OrthoImage3D` of approximate location to a topic and provides them through a service

    Downloads and stores orthoimage and optional DEM from WMS for location of projected camera field of view.

    Subscribes to :class:`.CameraInfo` and :class:`.FOV` messages to determine bounding box for next map to be cached.
    Requests new map whenever FOV overlap with current cached map gets too small. Publishes a :class:`.OrthoImage3D`
    with a high-resolution image and an optional digital elevation model (DEM) that can be used for pose estimation.

    .. warning::
        ``OWSLib`` *as of version 0.25.0* uses the Python ``requests`` library under the hood but does not seem to
        document the various exceptions it raises that are passed through by ``OWSLib`` as part of its public API.
        The :meth:`.get_map` method is therefore expected to raise `errors and exceptions
        <https://requests.readthedocs.io/en/latest/user/quickstart/#errors-and-exceptions>`_ that are specific to the
        ``requests`` library.

        These errors and exceptions are not handled by the :class:`.MapNode` to avoid a direct dependency to
        ``requests``. They are therefore handled as unexpected errors.
    """

    DEFAULT_URL = 'http://localhost:80/wms'
    """Default WMS URL"""

    DEFAULT_VERSION = '1.3.0'
    """Default WMS version"""

    DEFAULT_TIMEOUT = 10
    """Default WMS GetMap request timeout in seconds"""

    DEFAULT_CAMERA_INFO_TOPIC = 'camera/camera_info'
    """Default ROS subscribe topic for :class:`.CameraInfo` message

    .. note::
        :class:`.CameraInfo` is needed to determine correct size for rasters requested from external WMS
    """

    DEFAULT_BBOX_TOPIC = 'bbox'
    """Default ROS subscribe topic for :class:`.BoundingBox` message representing camera field of view projected to ground"""

    DEFAULT_ORTHOIMAGE_TOPIC = 'orthoimage_3d'
    """Default ROS publish topic for :class:`.OrthoImage3D` message"""

    DEFAULT_VEHICLE_POSITION_TOPIC = 'position'
    """Default ROS subscribe topic for :class:`geographic_msgs.msg.GeoPoint` vehicle position message"""

    DEFAULT_HOME_POSITION_TOPIC = 'home_position'
    """Default ROS subscribe topic for :class:`geographic_msgs.msg.GeoPoint` home position message"""

    DEFAULT_PUBLISH_RATE = 1
    """Default publish rate for :class:`.OrthoImage3D` messages in Hz"""

    ROS_D_LAYERS = ['imagery']
    """Default WMS GetMap request layers parameter for image raster

    .. note::
        The combined layers should cover the flight area of the vehicle at high resolution. Typically this list would 
        have just one layer for high resolution aerial or satellite imagery.
    """

    ROS_D_DEM_LAYERS = ['osm-buildings-dem']
    """Default WMS GetMap request layers parameter for DEM raster

    .. note::
        This is an optional elevation layer that makes the pose estimation more accurate especially when flying at low 
        altitude. It should be a grayscale raster with pixel values corresponding meters relative to origin. Origin
        can be whatever system is used (e.g. USGS DEM uses NAVD 88).
    """

    ROS_D_STYLES = ['']
    """Default WMS GetMap request styles parameter for image raster

    .. note::
        Must be same length as :py:attr:`.ROS_D_LAYERS`. Use empty strings for server default styles.
    """

    ROS_D_DEM_STYLES = ['']
    """Default WMS GetMap request styles parameter for DEM raster

    .. note::
        Must be same length as :py:attr:`.ROS_D_DEM_LAYERS`. Use empty strings for server default styles.
    """

    ROS_D_SRS = 'EPSG:4326'
    """Default WMS GetMap request SRS parameter"""

    ROS_D_IMAGE_FORMAT = 'image/jpeg'
    """Default WMS GetMap request image format"""

    ROS_D_IMAGE_TRANSPARENCY = False
    """Default WMS GetMap request image transparency 

    .. note::
        Not supported by jpeg format
    """

    ROS_D_MAP_OVERLAP_UPDATE_THRESHOLD = 0.85
    """Overlap ration between FOV and current map, under which a new map will be requested."""

    ROS_D_MAP_UPDATE_UPDATE_DELAY = 1
    """Default delay in seconds for throttling WMS GetMap requests
    
    .. note::
        TODO: ROS_D_MAP_UPDATE_UPDATE_DELAY not currently used but could be useful (old param from basenode)

    When the camera is mounted on a gimbal and is not static, this delay should be set quite low to ensure that whenever
    camera field of view is moved to some other location, the map update request will follow very soon after. The field
    of view of the camera projected on ground generally moves *much faster* than the vehicle itself.

    .. note::
        This parameter provides a hard upper limit for WMS GetMap request frequency. Even if this parameter is set low, 
        WMS GetMap requests will likely be much less frequent because they will throttled by the conditions set in  
        :meth:`._should_update_map` (private method - see source code for reference).
    """

    # Altitude in meters used for DEM request to get local origin elevation
    _DEM_REQUEST_ALTITUDE = 100

    _ROS_PARAMS = [
        ('layers', ROS_D_LAYERS),
        ('styles', ROS_D_STYLES),
        ('dem_layers', ROS_D_DEM_LAYERS),
        ('dem_styles', ROS_D_DEM_STYLES),
        ('srs', ROS_D_SRS),
        ('transparency', ROS_D_IMAGE_TRANSPARENCY),
        ('format', ROS_D_IMAGE_FORMAT),
        ('map_overlap_update_threshold', ROS_D_MAP_OVERLAP_UPDATE_THRESHOLD),
    ]
    """ROS parameters used by this node to declare"""

    def __init__(self, name: str, url: str = DEFAULT_URL, version: str = DEFAULT_VERSION,
                 timeout: int = DEFAULT_TIMEOUT, publish_rate: int = DEFAULT_PUBLISH_RATE):
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self._declare_ros_params()
        self._cv_bridge = CvBridge()
        self._wms_client = WebMapService(url, version=version, timeout=timeout)
        self._terrain_altitude_pub = self.create_publisher(ROSAltitude, 'terrain_altitude', QoSPresetProfiles.SENSOR_DATA.value)
        self._geopoint_pub = self.create_publisher(ROSGeoPoint, 'ground_geopoint', QoSPresetProfiles.SENSOR_DATA.value)
        self._orthoimage_pub = self.create_publisher(OrthoImage3D, self.DEFAULT_ORTHOIMAGE_TOPIC,
                                                     QoSPresetProfiles.SENSOR_DATA.value)
        self._bbox_sub = self.create_subscription(BoundingBox, MapNode.DEFAULT_BBOX_TOPIC, self._bbox_callback,
                                                  QoSPresetProfiles.SENSOR_DATA.value)
        self._camera_info_sub = self.create_subscription(CameraInfo, MapNode.DEFAULT_CAMERA_INFO_TOPIC,
                                                         self._camera_info_callback,
                                                         QoSPresetProfiles.SENSOR_DATA.value)
        self._vehicle_position_sub = self.create_subscription(ROSGeoPoint, MapNode.DEFAULT_VEHICLE_POSITION_TOPIC,
                                                         self._vehicle_position_callback,
                                                         QoSPresetProfiles.SENSOR_DATA.value)
        self._home_position_sub = self.create_subscription(ROSGeoPoint, MapNode.DEFAULT_HOME_POSITION_TOPIC,
                                                         self._home_position_callback,
                                                         QoSPresetProfiles.SENSOR_DATA.value)
        self._srv = self.create_service(GetMap, 'orthoimage_3d_service', self._srv_callback)
        self._timer = self._setup_map_update_timer(publish_rate)
        self._msg = None

        self._vehicle_position = None
        self._home_position = None
        self._camera_data = None

        # For map update timer / DEM requests
        self._origin_dem_altitude = None
        self._home_dem = None  # dem map data
        self._map_data = None

        # TODO: make configurable / use shared folder home path instead
        self._egm96 = GeoidPGM('/usr/share/GeographicLib/geoids/egm96-5.pgm', kind=-3)

    # region Properties
    @property
    def _wms_client(self) :
        """OWSLib generated WMS client"""
        return self.__wms_client

    @_wms_client.setter
    def _wms_client(self, value: Publisher) -> None:
        self.__wms_client = value

    @property
    def _orthoimage_pub(self) -> Publisher:
        """Publisher for :class:`OrthoImage3D` message"""
        return self.__orthoimage_pub

    @_orthoimage_pub.setter
    def _orthoimage_pub(self, value: Publisher) -> None:
        assert_type(value, Publisher)
        self.__orthoimage_pub = value

    @property
    def _camera_info_sub(self) -> Subscription:
        """Subscriber for :class:`.CameraInfo`"""
        return self.__camera_info_sub

    @_camera_info_sub.setter
    def _camera_info_sub(self, value: Subscription) -> None:
        assert_type(value, Subscription)
        self.__camera_info_sub = value

    @property
    def _bbox_sub(self) -> Subscription:
        """Subscriber for :class:`.FOV`"""
        return self.__fov_sub

    @_bbox_sub.setter
    def _bbox_sub(self, value: Subscription) -> None:
        assert_type(value, Subscription)
        self.__fov_sub = value

    @property
    def _timer(self) -> Timer:
        """:class:`.OrthoImage3D` publish and map update timer"""
        return self.__timer

    @_timer.setter
    def _timer(self, value: Timer) -> None:
        assert_type(value, Timer)
        self.__timer = value

    @property
    def _msg(self) -> Optional[OrthoImage3D]:
        """:class:`.OrthoImage3D` message to publish"""
        return self.__msg

    @_msg.setter
    def _msg(self, value: Optional[OrthoImage3D]) -> None:
        assert_type(value, get_args(Optional[OrthoImage3D]))
        self.__msg = value

    @property
    def _camera_data(self) -> Optional[CameraData]:
        """Latest :class:`sensor_msgs.msg.CameraInfo` message"""
        return self.__camera_data

    @_camera_data.setter
    def _camera_data(self, value: Optional[CameraData]) -> None:
        assert_type(value, get_args(Optional[CameraData]))
        self.__camera_data = value

    @property
    def _bbox(self) -> BBox:
        """Latest :class:`.BoundingBox` message"""
        return self.__bbox

    @_bbox.setter
    def _bbox(self, value: BBox) -> None:
        assert_type(value, BBox)
        self.__bbox = value

    @property
    def _srv(self) -> Service:
        """:class:`.OrthoImage3D` service"""
        return self.__srv

    @_srv.setter
    def _srv(self, value: Service) -> None:
        assert_type(value, Service)
        self.__srv = value

    @property
    def _origin_dem_altitude(self) -> Optional[float]:
        """Elevation layer (DEM) altitude at local frame origin"""
        return self.__origin_dem_altitude

    @_origin_dem_altitude.setter
    def _origin_dem_altitude(self, value: Optional[float]) -> None:
        assert_type(value, get_args(Optional[float]))
        self.__origin_dem_altitude = value

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
    # endregion Properties

    # region rclpy subscriber callbacks
    def _camera_info_callback(self, msg: CameraInfo) -> None:
        """Stores :class:`.CameraInfo` message"""
        #self._camera_info = msg
        if not all(hasattr(msg, attr) for attr in ['k', 'height', 'width']):
            # TODO: check that k and height/width match
            return None
        else:
            #self._k = msg.k.reshape((3, 3))  # store camera data instead
            self._camera_data = CameraData(msg.k.reshape((3, 3)), dim=Dim(msg.height, msg.width))
            if self._camera_info_sub is not None:
                # Assume camera info is static, destroy subscription
                #self.get_logger().warn("CameraInfo received, destroying subscription.")
                pass

    def _vehicle_position_callback(self, msg: ROSGeoPoint) -> None:
        """Stores :class:`geographic_msgs.msg.GeoPoint` message"""
        self._vehicle_position = msg

    def _home_position_callback(self, msg: ROSGeoPoint) -> None:
        """Stores :class:`geographic_msgs.msg.GeoPoint` message"""
        self._home_position = msg

    def _bbox_callback(self, msg: BoundingBox) -> None:
        """Stores :class:`.BoundingBox` message"""
        bbox = BBox(msg.min_pt.longitude, msg.min_pt.latitude, msg.max_pt.longitude, msg.max_pt.latitude)
        self._bbox = bbox
        size = self._get_map_size
        if self._should_request_new_map(bbox):
            if size is not None:
                img, dem = self._get_map(bbox, size)
                self._map_data = MapData(bbox=bbox, image=Img(img), elevation=Img(dem))
                self._msg = self._create_msg(bbox, img, dem)
            else:
                self.get_logger().warn(f'Cannot request new map, could not determine size ({size}) parameter for '
                                       f'GetMap request.')
    # endregion rclpy subscriber callbacks

    def _should_request_new_map(self, bbox: BBox) -> bool:
        """Returns True if a new map should be requested

        This check is made to avoid retrieving a new map that is almost the same as the previous map. Relaxing map
        update constraints should not improve accuracy of position estimates unless the map is so old that the field of
        view either no longer completely fits inside (vehicle has moved away or camera is looking in other direction)
        or is too small compared to the size of the map (vehicle altitude has significantly decreased).

        :param bbox: Bounding box of latest :class:`.FOV`
        :return: True if new map should be requested
        """
        if self._bbox is not None:
            if self._msg is not None:
                bbox_previous = BBox(self._msg.bbox.min_pt.longitude, self._msg.bbox.min_pt.latitude,
                                     self._msg.bbox.max_pt.longitude, self._msg.bbox.max_pt.latitude)
                threshold = self.get_parameter('map_overlap_update_threshold').get_parameter_value().double_value
                bbox1, bbox2 = box(*bbox), box(*bbox_previous)
                ratio1 = bbox1.intersection(bbox2).area / bbox1.area
                ratio2 = bbox2.intersection(bbox1).area / bbox2.area
                ratio = min(ratio1, ratio2)
                if ratio < threshold:
                    return True
            else:
                return True
        else:
            return False

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

    def _setup_map_update_timer(self, publish_rate: int) -> Timer:
        """Returns a timer to publish :class:`.OrthoImage3D`

        :param publish_rate: Publishing rate for the timer (in Hz)
        :return: The :class:`.Timer` instance
        """
        if publish_rate <= 0:
            error_msg = f'Map update rate must be positive ({publish_rate} Hz provided).'
            self.get_logger().error(error_msg)
            raise ValueError(error_msg)
        timer = self.create_timer(1 / publish_rate, self._publish)
        return timer

    def _get_map(self, bbox: BBox, size: Tuple[int, int]) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Sends GetMap request to WMS for image and DEM layers and returns them as a tuple, or None if not available

        Returns zero raster as DEM if no DEM layer is available.

        TODO: Currently no support for separate arguments for imagery and height layers. Assumes height layer is
        available at same CRS as imagery layer.

        :param bbox: Bounding box of the map (left, bottom, right, top)
        :param size: Map raster resolution (height, width)
        :return: Tuple of imagery and DEM rasters, or None if not available
        """
        assert_type(bbox, BBox)
        assert_type(size, tuple)

        layers, styles = self.get_parameter('layers').get_parameter_value().string_array_value,\
                         self.get_parameter('styles').get_parameter_value().string_array_value
        assert_len(styles, len(layers))
        assert all(isinstance(x, str) for x in layers)
        assert all(isinstance(x, str) for x in styles)

        dem_layers, dem_styles = self.get_parameter('dem_layers').get_parameter_value().string_array_value,\
                                 self.get_parameter('dem_styles').get_parameter_value().string_array_value
        assert_len(dem_styles, len(dem_layers))
        assert all(isinstance(x, str) for x in dem_layers)
        assert all(isinstance(x, str) for x in dem_styles)

        srs = self.get_parameter('srs').get_parameter_value().string_value
        format_ = self.get_parameter('format').get_parameter_value().string_value
        transparency = self.get_parameter('transparency').get_parameter_value().bool_value

        self.get_logger().info(f'Requesting orthoimage and DEM for\n'
                               f'bbox: {bbox},\n'
                               f'layers: {layers},\n'
                               f'styles: {styles},\n'
                               f'DEM layers: {dem_layers},\n'
                               f'DEM styles: {dem_styles},\n'
                               f'SRS: {srs},\n'
                               f'size: {size},\n'
                               f'transparency: {transparency},\n'
                               f'format: {format_}.')

        # Do not handle possible requests library related exceptions here (see class docstring)
        try:
            img = self._wms_client.getmap(layers=layers, styles=styles, srs=srs, bbox=bbox, size=size,
                                          format=format_, transparent=transparency)
        except ServiceException as se:
            self.get_logger().error(f'GetMap request for orthoimage ran into an unexpected exception: {se}')
            return None
        img = self._read_img(img)

        dem = None
        if len(dem_layers) > 0 and dem_layers[0]:
            self.get_logger().info(f'Getting DEM...')
            try:
                dem = self._wms_client.getmap(layers=dem_layers, styles=dem_styles, srs=srs, bbox=bbox, size=size,
                                              format=format_, transparent=transparency)
            except ServiceException as se:
                self.get_logger().error(f'GetMap request for DEM ran into an unexpected exception: {se}')
                return None
            dem = self._read_img(dem, True)
        else:
            # Assume flat (:=zero) terrain if no DEM layer provided
            dem = np.zeros_like(img)

        return img, dem

    def _srv_callback(self, request, response):
        """Service callback"""
        bbox = BBox(request.bbox.min_pt.longitude, request.bbox.min_pt.latitude,
                    request.bbox.max_pt.longitude, request.bbox.max_pt.latitude)
        size = request.height, request.width
        img, dem = self._get_map(bbox, size)
        response.image = self._create_msg(bbox, img, dem)

        return response

    @staticmethod
    def _read_img(img: bytes, grayscale: bool = False) -> np.ndarray:
        """Reads image bytes and returns numpy array

        :return: Image as np.ndarray
        """
        img = np.frombuffer(img.read(), np.uint8)
        img = cv2.imdecode(img, cv2.IMREAD_UNCHANGED) if not grayscale else cv2.imdecode(img, cv2.IMREAD_GRAYSCALE)
        assert_type(img, np.ndarray)
        #assert_ndim(img, 3)
        return img

    def _create_msg(self, bbox: BBox, img: np.ndarray, dem: np.ndarray) -> OrthoImage3D:
        """Stores :class:`.OrthoImage3D message for later publication

        :param bbox: The bounding box for the two maps
        :param img: The high resolution image
        :param dem: The DEM raster
        """
        return OrthoImage3D(
            bbox=BoundingBox(
                min_pt=ROSGeoPoint(latitude=bbox.bottom, longitude=bbox.left, altitude=np.nan),
                max_pt=ROSGeoPoint(latitude=bbox.top, longitude=bbox.right, altitude=np.nan)
            ),
            img=self._cv_bridge.cv2_to_imgmsg(img, encoding="passthrough"),
            dem=self._cv_bridge.cv2_to_imgmsg(dem, encoding="passthrough")
            #dem=self._cv_bridge.cv2_to_imgmsg(dem, encoding="mono8")  # mono16?
        )

    def _get_dynamic_map_radius(self, altitude: Union[int, float]) -> int:
        """Returns map radius that adjusts for camera altitude to be used for new map requests

        :param altitude: Altitude of camera in meters
        :return: Suitable map radius in meters
        """
        assert_type(altitude, get_args(Union[int, float]))
        max_map_radius = self.get_parameter('max_map_radius').get_parameter_value().integer_value

        if self._camera_data is not None:
            hfov = 2 * math.atan(self._camera_data.dim.width / (2 * self._camera_data.fx))
            map_radius = 1.5*hfov*altitude  # Arbitrary padding of 50%
        else:
            # Update map before CameraInfo has been received
            self.get_logger().warn(f'Could not get camera data, using guess for map width.')
            map_radius = 3*altitude  # Arbitrary guess

        if map_radius > max_map_radius:
            self.get_logger().warn(f'Dynamic map radius {map_radius} exceeds max map radius {max_map_radius}, using '
                                   f'max radius {max_map_radius} instead.')
            map_radius = max_map_radius

        return map_radius

    def _terrain_altitude_at_position(self, position: Optional[GeoPoint], local_origin: bool = False) -> Optional[float]:
        """Raw terrain altitude from DEM if available, or None if not available

        :param position: Position to query
        :param local_origin: True to use :py:attr:`._dem` (retrieved specifically for local frame origin)
        :return: Raw altitude in DEM coordinate frame and units
        """
        map_data = self._map_data if not local_origin else self._home_dem
        if map_data is not None and position is not None:
            elevation = map_data.elevation.arr
            bbox = map_data.bbox
            #polygon = bbox._geoseries[0]
            polygon = box(*bbox)
            # position = self._bridge.global_position
            point = position._geoseries[0]

            if polygon.contains(point):
                h, w = elevation.shape[0:2]
                assert h, w == self._img_dim
                #left, bottom, right, top = bbox.bounds
                left, bottom, right, top = bbox
                x = w * (position.lon - left) / (right - left)
                y = h * (position.lat - bottom) / (top - bottom)
                try:
                    dem_elevation = elevation[int(np.floor(y)), int(np.floor(x))]
                except IndexError as _:
                    # TODO: might be able to handle this
                    self.get_logger().warn('Position seems to be outside current elevation raster, cannot compute '
                                           'terrain altitude.')
                    return None

                return float(dem_elevation)
            else:
                # Should not happen
                self.get_logger().warn('Did not have elevation raster for current location or local frame origin '
                                       'altitude was unknwon, cannot compute terrain altitude.')
                return None

        self.get_logger().warn(f'Map data or position not provided, cannot determine DEM elevation.')
        return None

    def _terrain_altitude_amsl_at_position(self, position: Optional[GeoPoint], local_origin: bool = False):
        """Terrain altitude in meters AMSL accroding to DEM if available, or None if not available

        :param position: Position to query
        :param local_origin: Retrieve for local frame origin (bypass to use _dem instead of _map_data)
        :return: Terrain altitude AMSL in meters at position
        """
        dem_elevation = self._terrain_altitude_at_position(position, local_origin)
        if dem_elevation is not None and self._origin_dem_altitude is not None and self._home_position is not None:
            elevation_relative = dem_elevation - self._origin_dem_altitude
            elevation_amsl = elevation_relative + self._home_position.altitude \
                             - self._egm96.height(self._home_position.latitude, self._home_position.longitude)
            return float(elevation_amsl)

        return None

    def _should_request_dem_for_local_frame_origin(self) -> bool:
        """Returns True if a new map should be requested to determine elevation value for local frame origin

        DEM value for local frame origin is needed if elevation layer is used in order to determine absolute altitude
        of altitude estimates (GISNav estimates altitude against DEM, or assumes altitude at 0 if no DEM is provided).

        :return: True if new map should be requested
        """
        # TODO: re-request if home position/local frame origin has changed!
        if self._origin_dem_altitude is not None:
            self.get_logger().debug(f'Not requesting DEM because origin_dem_altitude is already set.')
            return False

        if self._home_position is None:
            self.get_logger().debug(f'Not requesting DEM because local_frame_origin is not available.')
            return False

        return True

    def _map_update_timer_callback(self) -> None:
        """Attempts to update the stored map at regular intervals

        Also gets DEM for local frame origin if needed (see :meth:`._should_request_dem_

        Calls :meth:`._update_map` if the center and altitude coordinates for the new map raster are available and the
        :meth:`._should_update_map` check passes.

        New map is retrieved based on a guess of the vehicle's global position. If
        :py:attr:`._is_gimbal_projection_enabled`, the center of the projected camera field of view is used instead of
        :py:attr:`._is_gimbal_pderojection_enabled`, the center of the projected camera field of view is used instead of
        vehicle position to ensure the field of view is best contained in the new map raster.
        """
        if self._should_request_dem_for_local_frame_origin():
            # Request DEM for local frame origin
            assert self._home_position is not None
            map_radius = self._get_dynamic_map_radius(self._DEM_REQUEST_ALTITUDE)
            xy = GeoPoint(x=self._home_position.longitude, y=self._home_position.latitude)
            map_candidate = GeoSquare(xy, map_radius)

            self.get_logger().info(f'Requesting DEM for local frame origin...')
            bbox = BBox(*map_candidate.bounds)
            if self._get_map_size is not None:
                img, dem = self._get_map(bbox, self._get_map_size)
                self._home_dem = MapData(bbox=bbox, image=Img(img), elevation=Img(dem))

                # TODO: assumes that this local_frame_origin is the starting location, same that was used for the request
                #  --> not strictly true even if it works for the simulation
                if self._origin_dem_altitude is None:
                    if self._home_position is not None:
                        self._origin_dem_altitude = self._terrain_altitude_at_position(xy, local_origin=True)
            else:
                self.get_logger().warn('Required map size unknown, skipping requesting DEM for home position.')

    def _publish_terrain_altitude(self) -> None:
        """Publishes terrain altitude at current position"""
        # Publish terrain_altitude
        vehicle_position = self._vehicle_position
        terrain_altitude_amsl = None
        #self.get_logger().error(f'vehicel position {vehicle_position}, home position {self._home_position}')
        if vehicle_position is not None and self._home_position is not None:
            xy = GeoPoint(x=vehicle_position.longitude, y=vehicle_position.latitude)
            terrain_altitude_amsl = self._terrain_altitude_amsl_at_position(xy)
            if terrain_altitude_amsl is None:
                # Probably have not received bbox yet so no map_data, try the data that was retrieved for local position
                # origin instead (assume we are at starting position)
                self.get_logger().warn(f'Could not get terrain altitude amsl for position from map data for publishing '
                                       f'geopoint, trying DEM which is intended for local origin...')
                terrain_altitude_amsl = self._terrain_altitude_amsl_at_position(xy, True)
            #snapshot = self._bridge.snapshot(terrain_altitude_amsl)

            # TODO: redundant in autopilot.py snapshot, remove the bridge eventually
            assert self._home_position is not None
            home_altitude_amsl = self._home_position.altitude - self._egm96.height(self._home_position.latitude,
                                                                                   self._home_position.longitude)
            terrain_altitude_agl = 0.
            terrain_altitude_ellipsoid = terrain_altitude_amsl + self._egm96.height(vehicle_position.latitude,
                                                                                    vehicle_position.longitude) \
                                             if terrain_altitude_amsl is not None else None
            terrain_altitude_home = terrain_altitude_amsl - home_altitude_amsl \
                if terrain_altitude_amsl is not None and home_altitude_amsl is not None else None

            if terrain_altitude_amsl is not None and terrain_altitude_home is not None \
                    and terrain_altitude_agl is not None:
                terrain_altitude_msg = ROSAltitude(header=self._get_header(),
                                        amsl=terrain_altitude_amsl,
                                        local=terrain_altitude_home,
                                        relative=terrain_altitude_home,
                                        terrain=terrain_altitude_agl,
                                        bottom_clearance=terrain_altitude_agl)
                self._terrain_altitude_pub.publish(terrain_altitude_msg)

                # Also publish geopoint message, includes lat and lon in atomic message
                if terrain_altitude_ellipsoid is not None:
                    geopoint_msg = ROSGeoPoint(
                        latitude=xy.lat,
                        longitude=xy.lon,
                        altitude=terrain_altitude_ellipsoid
                    )
                    self._geopoint_pub.publish(geopoint_msg)
                else:
                    self.get_logger().warn(f'Terrain altitude ellipsoide was None so skipping publishing.')
            else:
                self.get_logger().warn(f'Some altitude values were None so skipping publishing altitude '
                                       f'({terrain_altitude}).')
        else:
            self.get_logger().warn(f'Could not determine vehicle or home position, skipping publishing terrain altitude')

    def _publish(self) -> None:
        """Publishes :class:`.OrthoImage3D message"""
        self._map_update_timer_callback()
        if self._msg is not None:
            self._orthoimage_pub.publish(self._msg)
        self._publish_terrain_altitude()

    # TODO: redudnant implementation in pose_estimation_node.py and base node
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

    def destroy(self) -> None:
        """Unsubscribes ROS topics and destroys timer"""
        if self._bbox_sub is not None:
            self._bbox_sub.destroy()
        if self._camera_info_sub is not None:
            self._camera_info_sub.destroy()
        if self._timer is not None:
            self._timer.destroy()

