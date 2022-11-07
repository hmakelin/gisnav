"""Contains :class:`.Node` that provides :class:`OrthoImage3D`s"""
import numpy as np
import cv2
import io
import pstats
import cProfile
import rclpy

from typing import Optional, Tuple, get_args

from collections import namedtuple

from shapely.geometry import box

from rclpy.node import Node
from rclpy.subscription import Subscription
from rclpy.publisher import Publisher
from rclpy.service import Service
from rclpy.timer import Timer
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.qos import QoSPresetProfiles

from sensor_msgs.msg import CameraInfo
from geographic_msgs.msg import GeoPoint, BoundingBox

from owslib.wms import WebMapService
from owslib.util import ServiceException

from cv_bridge import CvBridge

from gisnav.assertions import assert_len, assert_type, assert_ndim
from gisnav_msgs.msg import OrthoImage3D
from gisnav_msgs.srv import GetMap
from gisnav.data import BBox



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
        self._orthoimage_pub = self.create_publisher(OrthoImage3D, self.DEFAULT_ORTHOIMAGE_TOPIC,
                                                     QoSPresetProfiles.SENSOR_DATA.value)
        self._bbox_sub = self.create_subscription(BoundingBox, MapNode.DEFAULT_BBOX_TOPIC, self._bbox_callback,
                                                  QoSPresetProfiles.SENSOR_DATA.value)
        self._camera_info_sub = self.create_subscription(CameraInfo, MapNode.DEFAULT_CAMERA_INFO_TOPIC,
                                                         self._camera_info_callback,
                                                         QoSPresetProfiles.SENSOR_DATA.value)
        self._srv = self.create_service(GetMap, 'orthoimage_3d_service', self._srv_callback)
        self._timer = self._setup_map_update_timer(publish_rate)
        self._msg = None

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
        """:class:`.OrthoImage3D` publish timer"""
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
    def _camera_info(self) -> CameraInfo:
        """Latest :class:`.CameraInfo` message"""
        return self.__camera_info

    @_camera_info.setter
    def _camera_info(self, value: CameraInfo) -> None:
        assert_type(value, CameraInfo)
        self.__camera_info = value

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
    def _get_map_size(self) -> Optional[Tuple[int, int]]:
        """GetMap request size parameter or None if not available

        .. note::
            This is a square with side length equal to the diagonal of the camera resolution. This resolution is chosen
            to give the map raster sufficient padding so that it can be center-cropped to the same resolution as the
            camera image without creating any corners that are black. This is needed to support pose estimators that
            are not rotation agnostic.
        """
        if self._camera_info is not None:
            assert hasattr(self._camera_info, 'height')
            assert hasattr(self._camera_info, 'width')
            diagonal = int(np.ceil(np.sqrt(self._camera_info.width ** 2 + self._camera_info.height ** 2)))
            assert_type(diagonal, int)
            return diagonal, diagonal
        else:
            self.get_logger().warn('Cannot compute GetMap request raster size, CameraInfo not yet received.')
            return None
    # endregion Properties

    # region rclpy subscriber callbacks
    def _camera_info_callback(self, msg: CameraInfo) -> None:
        """Stores :class:`.CameraInfo` message"""
        self._camera_info = msg

    def _bbox_callback(self, msg: BoundingBox) -> None:
        """Stores :class:`.BoundingBox` message"""
        bbox = BBox(msg.min_pt.longitude, msg.min_pt.latitude, msg.max_pt.longitude, msg.max_pt.latitude)
        self._bbox = bbox
        size = self._get_map_size
        if self._should_request_new_map(bbox):
            if size is not None:
                img, dem = self._get_map(bbox, size)
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
                min_pt=GeoPoint(latitude=bbox.bottom, longitude=bbox.left, altitude=np.nan),
                max_pt=GeoPoint(latitude=bbox.top, longitude=bbox.right, altitude=np.nan)
            ),
            img=self._cv_bridge.cv2_to_imgmsg(img, encoding="passthrough"),
            dem=self._cv_bridge.cv2_to_imgmsg(dem, encoding="passthrough")
            #dem=self._cv_bridge.cv2_to_imgmsg(dem, encoding="mono8")  # mono16?
        )

    def _publish(self) -> None:
        """Publishes :class:`.OrthoImage3D message"""
        if self._msg is not None:
            self._orthoimage_pub.publish(self._msg)

    def destroy(self) -> None:
        """Unsubscribes ROS topics and destroys timer"""
        if self._bbox_sub is not None:
            self._bbox_sub.destroy()
        if self._camera_info_sub is not None:
            self._camera_info_sub.destroy()
        if self._timer is not None:
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
    else:
        pr = None

    map_node = None
    try:
        rclpy.init(args=args)
        map_node = MapNode('map_node')
        rclpy.spin(map_node)
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
        if map_node is not None:
            map_node.destroy()
            map_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
