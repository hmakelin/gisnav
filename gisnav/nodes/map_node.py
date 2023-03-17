"""Contains :class:`.Node` that provides :class:`OrthoImage3D`s"""
import time
from typing import IO, Optional, Tuple, get_args

import cv2
import numpy as np
import requests
from cv_bridge import CvBridge
from geographic_msgs.msg import BoundingBox, GeoPoint, GeoPointStamped, GeoPoseStamped
from gisnav_msgs.msg import OrthoImage3D
from mavros_msgs.msg import Altitude
from owslib.util import ServiceException
from owslib.wms import WebMapService
from pygeodesy.geoids import GeoidPGM
from rclpy.qos import QoSPresetProfiles
from rclpy.timer import Timer
from sensor_msgs.msg import Image
from shapely.geometry import box
from std_msgs.msg import Float32

from ..assertions import assert_len, assert_type
from ..data import BBox, Img, MapData
from ..geo import GeoPt, GeoSquare, get_dynamic_map_radius
from . import messaging
from .base.camera_subscriber_node import CameraSubscriberNode


class MapNode(CameraSubscriberNode):
    """Publishes :class:`.OrthoImage3D` of approximate location to a topic

    Downloads and stores orthoimage and optional DEM from WMS for location of
    projected camera field of view.

    Subscribes to :class:`.CameraInfo` and :class:`.FOV` messages to determine
    bounding box for next map to be cached. Requests new map whenever FOV overlap
    with current cached map gets too small. Publishes a :class:`.OrthoImage3D`
    with a high-resolution image and an optional digital elevation model (DEM)
    that can be used for pose estimation.

    .. warning::
        ``OWSLib`` *as of version 0.25.0* uses the Python ``requests`` library
        under the hood but does not seem to document the various exceptions it
        raises that are passed through by ``OWSLib`` as part of its public API.
        The :meth:`.get_map` method is therefore expected to raise `errors and exceptions
        <https://requests.readthedocs.io/en/latest/user/quickstart/#errors-and-exceptions>`_
        that are specific to the ``requests`` library.

        These errors and exceptions are not handled by the :class:`.MapNode`
        to avoid a direct dependency to ``requests``. They are therefore
        handled as unexpected errors.
    """  # noqa: E501

    ROS_D_URL = "http://127.0.0.1:80/wms"
    """Default WMS URL"""

    ROS_D_VERSION = "1.3.0"
    """Default WMS version"""

    ROS_D_TIMEOUT = 10
    """Default WMS GetMap request timeout in seconds"""

    ROS_D_PUBLISH_RATE = 1
    """Default publish rate for :class:`.OrthoImage3D` messages in Hz"""

    ROS_D_LAYERS = ["imagery"]
    """Default WMS GetMap request layers parameter for image raster

    .. note::
        The combined layers should cover the flight area of the vehicle at high
        resolution. Typically this list would have just one layer for high
        resolution aerial or satellite imagery.
    """

    ROS_D_DEM_LAYERS = ["osm-buildings-dem"]
    """Default WMS GetMap request layers parameter for DEM raster

    .. note::
        This is an optional elevation layer that makes the pose estimation more
        accurate especially when flying at low altitude. It should be a grayscale
        raster with pixel values corresponding meters relative to origin. Origin
        can be whatever system is used (e.g. USGS DEM uses NAVD 88).
    """

    ROS_D_STYLES = [""]
    """Default WMS GetMap request styles parameter for image raster

    .. note::
        Must be same length as :py:attr:`.ROS_D_LAYERS`. Use empty strings for
        server default styles.
    """

    ROS_D_DEM_STYLES = [""]
    """Default WMS GetMap request styles parameter for DEM raster

    .. note::
        Must be same length as :py:attr:`.ROS_D_DEM_LAYERS`. Use empty strings
        for server default styles.
    """

    ROS_D_SRS = "EPSG:4326"
    """Default WMS GetMap request SRS parameter"""

    ROS_D_IMAGE_FORMAT = "image/jpeg"
    """Default WMS GetMap request image format"""

    ROS_D_IMAGE_TRANSPARENCY = False
    """Default WMS GetMap request image transparency

    .. note::
        Not supported by jpeg format
    """

    ROS_D_MAP_OVERLAP_UPDATE_THRESHOLD = 0.85
    """Overlap ration between FOV and current map, under which a new map will
    be requested."""

    ROS_D_MAX_MAP_RADIUS = 1000
    """Max radius for circle inside the maps (half map side length)"""

    ROS_D_MAP_UPDATE_UPDATE_DELAY = 1
    """Default delay in seconds for throttling WMS GetMap requests

    .. note::
        TODO: ROS_D_MAP_UPDATE_UPDATE_DELAY not currently used but could be
        useful (old param from basenode)

    When the camera is mounted on a gimbal and is not static, this delay should
    be set quite low to ensure that whenever camera field of view is moved to
    some other location, the map update request will follow very soon after.
    The field of view of the camera projected on ground generally moves
    *much faster* than the vehicle itself.

    .. note::
        This parameter provides a hard upper limit for WMS GetMap request
        frequency. Even if this parameter is set low, WMS GetMap requests will
        likely be much less frequent because they will throttled by the
        conditions set in :meth:`._should_request_new_map`.
    """

    # Altitude in meters used for DEM request to get local origin elevation
    _DEM_REQUEST_ALTITUDE = 100

    _WMS_CONNECTION_ATTEMPT_DELAY = 10
    """Delay in seconds until a new WMS connection is attempted in case of
    connection error"""

    ROS_PARAM_DEFAULTS = [
        ("url", ROS_D_URL, True),
        ("version", ROS_D_VERSION, True),
        ("timeout", ROS_D_TIMEOUT, True),
        ("publish_rate", ROS_D_PUBLISH_RATE, True),
        ("layers", ROS_D_LAYERS, False),
        ("styles", ROS_D_STYLES, False),
        ("dem_layers", ROS_D_DEM_LAYERS, False),
        ("dem_styles", ROS_D_DEM_STYLES, False),
        ("srs", ROS_D_SRS, False),
        ("transparency", ROS_D_IMAGE_TRANSPARENCY, False),
        ("format", ROS_D_IMAGE_FORMAT, False),
        ("map_overlap_update_threshold", ROS_D_MAP_OVERLAP_UPDATE_THRESHOLD, False),
        ("max_map_radius", ROS_D_MAX_MAP_RADIUS, False),
    ]
    """List containing ROS parameter name, default value and read_only flag tuples"""

    def __init__(self, name: str):
        """Class initializer

        :param name: Node name
        """
        super().__init__(name)

        # Publishers
        self._terrain_altitude_pub = self.create_publisher(
            Altitude,
            messaging.ROS_TOPIC_TERRAIN_ALTITUDE,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self._egm96_height_pub = self.create_publisher(
            Float32,
            messaging.ROS_TOPIC_EGM96_HEIGHT,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self._terrain_geopoint_pub = self.create_publisher(
            GeoPointStamped,
            messaging.ROS_TOPIC_TERRAIN_GEOPOINT,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self._ortho_image_3d_msg = (
            None  # This outgoing message is on a timer, needs to be stored
        )
        self._ortho_image_3d_pub = self.create_publisher(
            OrthoImage3D,
            messaging.ROS_TOPIC_ORTHOIMAGE,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

        # Subscribers
        self._bounding_box = None
        self._bounding_box_sub = self.create_subscription(
            BoundingBox,
            messaging.ROS_TOPIC_BOUNDING_BOX,
            self._bounding_box_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self._vehicle_geopose = None
        self._vehicle_geopose_sub = self.create_subscription(
            GeoPoseStamped,
            messaging.ROS_TOPIC_VEHICLE_GEOPOSE,
            self._vehicle_geopose_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self._home_geopoint = None
        self._home_geopoint_sub = self.create_subscription(
            GeoPointStamped,
            messaging.ROS_TOPIC_HOME_GEOPOINT,
            self._home_geopoint_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

        publish_rate = (
            self.get_parameter("publish_rate").get_parameter_value().integer_value
        )
        self._publish_timer = self._create_publish_timer(publish_rate)
        self._ortho_image_3d_msg = None

        # For map update timer / DEM requests
        self._origin_dem_altitude = None
        self._home_dem = None  # dem map data
        self._map_data = None

        # TODO: make configurable / use shared folder home path instead
        self._egm96 = GeoidPGM("/usr/share/GeographicLib/geoids/egm96-5.pgm", kind=-3)

        self._cv_bridge = CvBridge()

        url = self.get_parameter("url").get_parameter_value().string_value
        version = self.get_parameter("version").get_parameter_value().string_value
        timeout = self.get_parameter("timeout").get_parameter_value().integer_value
        self.connected = False  # TODO: handle disconnects, declare property
        while not self.connected:
            try:
                self.get_logger().error("Connecting to WMS endpoint...")
                self._wms_client = WebMapService(url, version=version, timeout=timeout)
            except requests.exceptions.ConnectionError as _:  # noqa: F841
                self.get_logger().error(
                    f"Could not connect to WMS endpoint, trying again in "
                    f"{self._WMS_CONNECTION_ATTEMPT_DELAY}s..."
                )
                time.sleep(self._WMS_CONNECTION_ATTEMPT_DELAY)

        self.connected = True
        self.get_logger().info("WMS client setup complete.")

    # region Properties
    @property
    def _publish_timer(self) -> Timer:
        """:class:`gisnav_msgs.msg.OrthoImage3D` publish and map update timer"""
        return self.__publish_timer

    @_publish_timer.setter
    def _publish_timer(self, value: Timer) -> None:
        assert_type(value, Timer)
        self.__publish_timer = value

    @property
    def _map_data(self) -> Optional[MapData]:
        """Stored map data"""
        return self.__map_data

    @_map_data.setter
    def _map_data(self, value: MapData) -> None:
        self.__map_data = value

    @property
    def _ortho_image_3d_msg(self) -> Optional[OrthoImage3D]:
        """:class:`gisnav_msgs.msg.OrthoImage3D` message to publish"""
        return self.__ortho_image_3d_msg

    @_ortho_image_3d_msg.setter
    def _ortho_image_3d_msg(self, value: Optional[OrthoImage3D]) -> None:
        assert_type(value, get_args(Optional[OrthoImage3D]))
        self.__ortho_image_3d_msg = value

    @property
    def _origin_dem_altitude(self) -> Optional[float]:
        """Elevation layer (DEM) altitude at local frame origin"""
        return self.__origin_dem_altitude

    @_origin_dem_altitude.setter
    def _origin_dem_altitude(self, value: Optional[float]) -> None:
        assert_type(value, get_args(Optional[float]))
        self.__origin_dem_altitude = value

    # region rclpy subscriber callbacks
    def _vehicle_geopose_callback(self, msg: GeoPoseStamped) -> None:
        """Stores :class:`geographic_msgs.msg.GeoPoseStamped` message"""
        self._vehicle_geopose = msg

        # Needed by autopilot node to publish vehicle/home GeoPose with
        # ellipsoid altitude
        height = self._egm96.height(
            msg.pose.position.latitude, msg.pose.position.longitude
        )
        height_msg = Float32(data=height)
        self._egm96_height_pub.publish(height_msg)

    def _home_geopoint_callback(self, msg: GeoPointStamped) -> None:
        """Stores :class:`geographic_msgs.msg.GeoPointStamped` message"""
        self._home_geopoint = msg

    def _bounding_box_callback(self, msg: BoundingBox) -> None:
        """Stores :class:`geograhpic_msgs.msg.BoundingBox` message"""
        self._bounding_box = msg

        bbox = BBox(
            msg.min_pt.longitude,
            msg.min_pt.latitude,
            msg.max_pt.longitude,
            msg.max_pt.latitude,
        )
        if self._should_request_new_map(bbox):
            if self.map_size_with_padding is not None:
                map = self._get_map(bbox, self.map_size_with_padding)
                if map is not None:
                    img, dem = map
                    self._map_data = MapData(
                        bbox=bbox, image=Img(img), elevation=Img(dem)
                    )
                    self._ortho_image_3d_msg = self._create_msg(bbox, img, dem)
                else:
                    self.get_logger().warn(
                        "Could not get new map - will not update map data nor "
                        "orthoimage message."
                    )
            else:
                self.get_logger().warn(
                    f"Cannot request new map, could not determine size "
                    f"({self.map_size_with_padding}) parameter for GetMap request."
                )

    def image_callback(self, msg: Image) -> None:
        """Receives :class:`sensor_msgs.msg.Image` message"""
        # Do nothing - enforced by CameraSubscriberNode parent

    # endregion rclpy subscriber callbacks

    def _should_request_new_map(self, bbox: BBox) -> bool:
        """Returns True if a new map should be requested

        This check is made to avoid retrieving a new map that is almost the same
        as the previous map. Relaxing map update constraints should not improve
        accuracy of position estimates unless the map is so old that the field of
        view either no longer completely fits inside (vehicle has moved away or
        camera is looking in other direction) or is too small compared to the
        size of the map (vehicle altitude has significantly decreased).

        :param bbox: Bounding box of latest containing camera field of view
        :return: True if new map should be requested
        """
        if self._bounding_box is not None:
            if self._ortho_image_3d_msg is not None:
                bbox_previous = messaging.bounding_box_to_bbox(
                    self._ortho_image_3d_msg.bbox
                )
                threshold = (
                    self.get_parameter("map_overlap_update_threshold")
                    .get_parameter_value()
                    .double_value
                )
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

    def _create_publish_timer(self, publish_rate: int) -> Timer:
        """Returns a timer to publish :class:`gisnav_msgs.msg.OrthoImage3D`

        :param publish_rate: Publishing rate for the timer (in Hz)
        :return: The :class:`.Timer` instance
        """
        if publish_rate <= 0:
            error_msg = (
                f"Map update rate must be positive ({publish_rate} Hz provided)."
            )
            self.get_logger().error(error_msg)
            raise ValueError(error_msg)
        timer = self.create_timer(1 / publish_rate, self._update_and_publish)
        return timer

    def _get_map(
        self, bbox: BBox, size: Tuple[int, int]
    ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Sends GetMap request to WMS for image and DEM layers and returns
        them as a tuple, or None if not available

        Returns zero raster as DEM if no DEM layer is available.

        TODO: Currently no support for separate arguments for imagery and height
        layers. Assumes height layer is available at same CRS as imagery layer.

        :param bbox: Bounding box of the map (left, bottom, right, top)
        :param size: Map raster resolution (height, width)
        :return: Tuple of imagery and DEM rasters, or None if not available
        """
        assert_type(bbox, BBox)
        assert_type(size, tuple)

        layers, styles = (
            self.get_parameter("layers").get_parameter_value().string_array_value,
            self.get_parameter("styles").get_parameter_value().string_array_value,
        )
        assert_len(styles, len(layers))
        assert all(isinstance(x, str) for x in layers)
        assert all(isinstance(x, str) for x in styles)

        dem_layers, dem_styles = (
            self.get_parameter("dem_layers").get_parameter_value().string_array_value,
            self.get_parameter("dem_styles").get_parameter_value().string_array_value,
        )
        assert_len(dem_styles, len(dem_layers))
        assert all(isinstance(x, str) for x in dem_layers)
        assert all(isinstance(x, str) for x in dem_styles)

        srs = self.get_parameter("srs").get_parameter_value().string_value
        format_ = self.get_parameter("format").get_parameter_value().string_value
        transparency = (
            self.get_parameter("transparency").get_parameter_value().bool_value
        )

        self.get_logger().info(
            f"Requesting orthoimage and DEM for\n"
            f"bbox: {bbox},\n"
            f"layers: {layers},\n"
            f"styles: {styles},\n"
            f"DEM layers: {dem_layers},\n"
            f"DEM styles: {dem_styles},\n"
            f"SRS: {srs},\n"
            f"size: {size},\n"
            f"transparency: {transparency},\n"
            f"format: {format_}."
        )

        # Do not handle possible requests library related exceptions here
        # (see class docstring)
        try:
            self.get_logger().info("Requesting orthoimage...")
            img: IO = self._wms_client.getmap(
                layers=layers,
                styles=styles,
                srs=srs,
                bbox=bbox,
                size=size,
                format=format_,
                transparent=transparency,
            )
        except ServiceException as se:
            self.get_logger().error(
                f"GetMap request for orthoimage ran into an unexpected exception: {se}"
            )
            return None
        finally:
            self.get_logger().info("Orthoimage request complete.")
        img_ = self._read_img(img)

        dem = None
        if len(dem_layers) > 0 and dem_layers[0]:
            try:
                self.get_logger().info("Requesting DEM...")
                dem = self._wms_client.getmap(
                    layers=dem_layers,
                    styles=dem_styles,
                    srs=srs,
                    bbox=bbox,
                    size=size,
                    format=format_,
                    transparent=transparency,
                )
            except ServiceException as se:
                self.get_logger().error(
                    f"GetMap request for DEM ran into an unexpected exception: {se}"
                )
                return None
            finally:
                self.get_logger().info("DEM request complete.")
            dem = self._read_img(dem, True)
        else:
            # Assume flat (:=zero) terrain if no DEM layer provided
            self.get_logger().debug(
                "No DEM layer provided, assuming flat (=zero) elevation model."
            )
            dem = np.zeros_like(img_)

        return img_, dem

    @staticmethod
    def _read_img(img: IO, grayscale: bool = False) -> np.ndarray:
        """Reads image bytes and returns numpy array

        :return: Image as np.ndarray
        """
        img = np.frombuffer(img.read(), np.uint8)
        img = (
            cv2.imdecode(img, cv2.IMREAD_UNCHANGED)
            if not grayscale
            else cv2.imdecode(img, cv2.IMREAD_GRAYSCALE)
        )
        assert_type(img, np.ndarray)
        # assert_ndim(img, 3)
        return img

    def _create_msg(self, bbox: BBox, img: np.ndarray, dem: np.ndarray) -> OrthoImage3D:
        """Stores :class:`.OrthoImage3D message for later publication

        :param bbox: The bounding box for the two maps
        :param img: The high resolution image
        :param dem: The DEM raster
        """
        return OrthoImage3D(
            bbox=messaging.bbox_to_bounding_box(bbox),
            img=self._cv_bridge.cv2_to_imgmsg(img, encoding="passthrough"),
            dem=self._cv_bridge.cv2_to_imgmsg(dem, encoding="passthrough")
            # dem=self._cv_bridge.cv2_to_imgmsg(dem, encoding="mono8")
        )  # TODO: need to use mono16? 255 meters max?

    def _terrain_altitude_at_position(
        self, position: Optional[GeoPt], local_origin: bool = False
    ) -> Optional[float]:
        """Raw terrain altitude from DEM if available, or None if not available

        :param position: Position to query
        :param local_origin: True to use :py:attr:`._home_dem` (retrieved
            specifically for local frame origin)
        :return: Raw altitude in DEM coordinate frame and units
        """
        map_data = self._map_data if not local_origin else self._home_dem
        if map_data is not None and position is not None:
            elevation = map_data.elevation.arr
            bbox = map_data.bbox
            polygon = box(*bbox)
            point = position._geoseries[0]

            if polygon.contains(point):
                h, w = elevation.shape[0:2]
                assert h, w == self._img_dim
                left, bottom, right, top = bbox
                x = w * (position.lon - left) / (right - left)
                y = h * (position.lat - bottom) / (top - bottom)
                try:
                    dem_elevation = elevation[int(np.floor(y)), int(np.floor(x))]
                except IndexError:
                    # TODO: might be able to handle this
                    self.get_logger().warn(
                        "Position seems to be outside current elevation raster, "
                        "cannot compute terrain altitude."
                    )
                    return None

                return float(dem_elevation)
            else:
                # Should not happen
                self.get_logger().warn(
                    "Did not have elevation raster for current location or local "
                    "frame origin altitude was unknwon, cannot compute terrain "
                    "altitude."
                )
                return None

        self.get_logger().warn(
            "Map data or position not provided, cannot determine DEM elevation."
        )
        return None

    def _terrain_altitude_amsl_at_position(
        self, position: Optional[GeoPt], local_origin: bool = False
    ):
        """Terrain altitude in meters AMSL accroding to DEM if available, or
        None if not available

        :param position: Position to query
        :param local_origin: Set to True to use :py:attr:`._home_dem` instead
            of :py:attr:`._map_data`
        :return: Terrain altitude AMSL in meters at position
        """
        dem_elevation = self._terrain_altitude_at_position(position, local_origin)
        if (
            dem_elevation is not None
            and self._origin_dem_altitude is not None
            and self._home_geopoint is not None
        ):
            elevation_relative = dem_elevation - self._origin_dem_altitude
            elevation_amsl = (
                elevation_relative
                + self._home_geopoint.position.altitude
                - self._egm96.height(
                    self._home_geopoint.position.latitude,
                    self._home_geopoint.position.longitude,
                )
            )
            return float(elevation_amsl)

        return None

    def _should_request_dem_for_local_frame_origin(self) -> bool:
        """Returns True if a new map should be requested to determine elevation
        value for local frame origin

        DEM value for local frame origin is needed if elevation layer is used
        in order to determine absolute altitude of altitude estimates (GISNav
        estimates altitude against DEM, or assumes altitude at 0 if no DEM is
        provided).

        :return: True if new map should be requested
        """
        # TODO: re-request if home position/local frame origin has changed?
        #  currently they are assumed equal
        if self._origin_dem_altitude is not None:
            self.get_logger().debug(
                "Not requesting DEM because origin_dem_altitude is already set."
            )
            return False

        if self._home_geopoint is None:
            self.get_logger().debug(
                "Not requesting DEM because local_frame_origin is not available."
            )
            return False

        if self.camera_data is None:
            self.get_logger().debug(
                "Not requesting DEM because camera_data is not available."
            )
            return False

        return True

    def _publish_terrain_altitude(self) -> None:
        """Publishes terrain altitude at current position"""
        terrain_altitude_amsl = None
        if self._vehicle_geopose is not None and self._home_geopoint is not None:
            xy = GeoPt(
                x=self._vehicle_geopose.pose.position.longitude,
                y=self._vehicle_geopose.pose.position.latitude,
            )
            terrain_altitude_amsl = self._terrain_altitude_amsl_at_position(xy)
            if terrain_altitude_amsl is None:
                # Probably have not received bbox yet so no map_data, try the
                # data that was retrieved for local position origin instead
                # (assume we are at starting position)
                self.get_logger().warn(
                    "Could not get terrain altitude amsl for position from map "
                    "data for publishing geopoint, trying DEM which is intended "
                    "for local origin..."
                )
                terrain_altitude_amsl = self._terrain_altitude_amsl_at_position(
                    xy, True
                )

            # TODO: redundant in autopilot.py snapshot, remove the bridge eventually
            assert self._home_geopoint is not None
            home_altitude_amsl = (
                self._home_geopoint.position.altitude
                - self._egm96.height(
                    self._home_geopoint.position.latitude,
                    self._home_geopoint.position.longitude,
                )
            )
            terrain_altitude_agl = 0.0
            if terrain_altitude_amsl is not None:
                terrain_altitude_ellipsoid = terrain_altitude_amsl + self._egm96.height(
                    self._vehicle_geopose.pose.position.latitude,
                    self._vehicle_geopose.pose.position.longitude,
                )
            else:
                terrain_altitude_ellipsoid = None

            terrain_altitude_home = (
                terrain_altitude_amsl - home_altitude_amsl
                if terrain_altitude_amsl is not None and home_altitude_amsl is not None
                else None
            )

            if (
                terrain_altitude_amsl is not None
                and terrain_altitude_home is not None
                and terrain_altitude_agl is not None
            ):
                terrain_altitude_msg = Altitude(
                    header=messaging.create_header("base_link"),
                    amsl=terrain_altitude_amsl,
                    local=terrain_altitude_home,
                    relative=terrain_altitude_home,
                    terrain=terrain_altitude_agl,
                    bottom_clearance=terrain_altitude_agl,
                )
                self._terrain_altitude_pub.publish(terrain_altitude_msg)

                # Also publish geopoint message, includes lat and lon in atomic message
                if terrain_altitude_ellipsoid is not None:
                    geopoint_msg = GeoPointStamped(
                        header=messaging.create_header("base_link"),
                        position=GeoPoint(
                            latitude=xy.lat,
                            longitude=xy.lon,
                            altitude=terrain_altitude_ellipsoid,
                        ),
                    )
                    self._terrain_geopoint_pub.publish(geopoint_msg)
                else:
                    self.get_logger().warn(
                        "Terrain altitude ellipsoid was None so skipping publishing."
                    )
            else:
                self.get_logger().warn(
                    f"Some altitude values were None so skipping publishing altitude "
                    f"({terrain_altitude_amsl}, {terrain_altitude_home}, "
                    f"{terrain_altitude_agl})."
                )
        else:
            self.get_logger().warn(
                "Could not determine vehicle or home position, skipping "
                "publishing terrain altitude"
            )

    def _update_and_publish(self) -> None:
        """Updates map and publishes :class:`gisnav_msgs.msg.OrthoImage3d`
        and terrain :class:`mavros_msgs.msg.Altitude`

        Additionally requests DEM for home (assumed local frame origin) if needed (see
        :meth:`._should_request_dem_for_local_frame_origin).

        Calls :meth:`._request_new_map` if the center and altitude coordinates
        for the new map raster are available and the :meth:`._should_request_new_map`
        check passes.
        """
        if self._should_request_dem_for_local_frame_origin():
            assert self._home_geopoint is not None
            assert self.camera_data is not None
            max_map_radius = (
                self.get_parameter("max_map_radius").get_parameter_value().integer_value
            )
            map_radius = get_dynamic_map_radius(
                self.camera_data, max_map_radius, self._DEM_REQUEST_ALTITUDE
            )
            if map_radius <= 0:
                self.get_logger().warn(
                    f"Could not determine valid map radius ({map_radius}), "
                    f"skipping requesting DEM for home."
                )
            else:
                assert -180 <= abs(self._home_geopoint.position.longitude) <= 180
                assert -90 <= abs(self._home_geopoint.position.latitude) <= 90
                assert_type(map_radius, float)
                xy = GeoPt(
                    x=self._home_geopoint.position.longitude,
                    y=self._home_geopoint.position.latitude,
                )
                map_candidate = GeoSquare(xy, map_radius)

                bbox = BBox(*map_candidate.bounds)
                if self.map_size_with_padding is not None:
                    self.get_logger().info(
                        "Requesting DEM for home/local frame origin (assumed same!)."
                    )
                    img, dem = self._get_map(bbox, self.map_size_with_padding)
                    self._home_dem = MapData(
                        bbox=bbox, image=Img(img), elevation=Img(dem)
                    )

                    # TODO: assumes that this local_frame_origin is the starting
                    #  location, same that was used for the request --> not
                    #  strictly true even if it works for the simulation
                    if self._origin_dem_altitude is None:
                        if self._home_geopoint is not None:
                            self._origin_dem_altitude = (
                                self._terrain_altitude_at_position(
                                    xy, local_origin=True
                                )
                            )
                else:
                    self.get_logger().warn(
                        "Required map size unknown, skipping requesting DEM for home."
                    )

        if self._ortho_image_3d_msg is not None:
            self._ortho_image_3d_pub.publish(self._ortho_image_3d_msg)

        self._publish_terrain_altitude()
