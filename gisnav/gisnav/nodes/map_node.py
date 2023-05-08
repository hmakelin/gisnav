"""Contains :class:`.Node` that provides :class:`OrthoImage3D`s"""
import time
from functools import lru_cache
from typing import IO, Optional, Tuple, get_args

import cv2
import numpy as np
import requests
from cv_bridge import CvBridge
from geographic_msgs.msg import BoundingBox, GeoPoint, GeoPointStamped, GeoPoseStamped
from mavros_msgs.msg import Altitude
from owslib.util import ServiceException
from owslib.wms import WebMapService
from pygeodesy.geoids import GeoidPGM
from rclpy.qos import QoSPresetProfiles
from rclpy.timer import Timer
from sensor_msgs.msg import Image
from shapely.geometry import box
from std_msgs.msg import Float32

from gisnav_msgs.msg import OrthoImage3D  # type: ignore

from ..assertions import ROS, assert_len, assert_type, enforce_types
from ..data import BBox, Img, MapData
from ..geo import GeoPt, GeoSquare, get_dynamic_map_radius
from . import messaging
from .base.camera_subscriber_node import CameraSubscriberNode

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


@ROS.parameters(
    [
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
)
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

    # Altitude in meters used for DEM request to get local origin elevation
    _DEM_REQUEST_ALTITUDE = 100

    _WMS_CONNECTION_ATTEMPT_DELAY = 10
    """Delay in seconds until a new WMS connection is attempted in case of
    connection error"""

    # TODO: remove once all nodes have @ROS.parameters decoration
    ROS_PARAM_DEFAULTS = []
    """List containing ROS parameter name, default value and read_only flag tuples"""

    def __init__(self, name: str):
        """Class initializer

        :param name: Node name
        """
        super().__init__(name)

        publish_rate = (
            self.get_parameter("publish_rate").get_parameter_value().integer_value
        )
        self._publish_timer = self._create_publish_timer(publish_rate)

        # For map update timer / DEM requests
        self._origin_dem_altitude = None
        self._home_dem = None  # dem map data
        self._request_new_map_data = None

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
                self.connected = True
            except requests.exceptions.ConnectionError as _:  # noqa: F841
                self.get_logger().error(
                    f"Could not connect to WMS endpoint, trying again in "
                    f"{self._WMS_CONNECTION_ATTEMPT_DELAY}s..."
                )
                time.sleep(self._WMS_CONNECTION_ATTEMPT_DELAY)

        # TODO: any other errors that might prevent connection?
        #  handle disconnect & reconnect
        assert self.connected
        self.get_logger().info("WMS client setup complete.")

    @property
    def bbox(self) -> Optional[BBox]:
        """
        Geographical bounding box of area to retrieve a map for, or None if not
        available or too old

        .. note::
            This is a more convenient named tuple alternative for
            :attr:`.bounding_box`
        """

        @enforce_types(self.get_logger().warn, "Cannot generate BBox")
        def _bbox(bounding_box: BoundingBox):
            bbox = BBox(
                bounding_box.min_pt.longitude,
                bounding_box.min_pt.latitude,
                bounding_box.max_pt.longitude,
                bounding_box.max_pt.latitude,
            )
            return bbox

        return _bbox(self.bounding_box)

    def bounding_box_cb(self, msg: BoundingBox) -> None:
        """Callback for :class:`geographic_msgs.msg.BoundingBox` messages"""

        bbox = self.bbox
        assert (
            bbox is not None
        )  # BoundingBox has just been received, handle this better
        if self._should_request_new_map_data(
            bbox
        ):  # todo: should not need to pass bbox as argument?
            self._request_new_map_data()

    @property
    @ROS.max_delay_ms(2000)
    @ROS.subscribe(
        messaging.ROS_TOPIC_BOUNDING_BOX,
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=bounding_box_cb,
    )
    def bounding_box(self) -> BoundingBox:
        """
        Geographical bounding box of area to retrieve a map for, or None if not
        available or too old
        """

    def geopose_cb(self, msg: GeoPoseStamped) -> None:
        """Callback for :class:`geographic_msgs.msg.GeoPoseStamped` message"""
        # Needed by autopilot node to publish vehicle/home GeoPoseStamped with
        # ellipsoid altitude
        self.egm96_height

    @property
    @ROS.max_delay_ms(400)
    @ROS.subscribe(
        messaging.ROS_TOPIC_VEHICLE_GEOPOSE,
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=geopose_cb,
    )
    def geopose(self) -> Optional[GeoPoseStamped]:
        """Vehicle GeoPoseStamped, or None if not available or too old"""

    @property
    @ROS.max_delay_ms(400)
    @ROS.subscribe(
        messaging.ROS_TOPIC_HOME_GEOPOINT, QoSPresetProfiles.SENSOR_DATA.value
    )
    def home_geopoint(self) -> Optional[GeoPoseStamped]:
        """Home position GeoPointStamped, or None if not available or too old"""

    @ROS.publish(
        messaging.ROS_TOPIC_TERRAIN_ALTITUDE,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def terrain_altitude(self) -> Optional[Altitude]:
        """Altitude of terrain directly under drone, or None if not available"""

        def _terrain_altitude(
            terrain_altitude_amsl: float,
            terrain_altitude_agl: float,
            terrain_altitude_at_home_amsl: float,
        ):
            terrain_altitude = Altitude(
                header=messaging.create_header("base_link"),
                amsl=terrain_altitude_amsl,
                local=terrain_altitude_at_home_amsl,
                relative=terrain_altitude_at_home_amsl,
                terrain=terrain_altitude_agl,
                bottom_clearance=terrain_altitude_agl,
            )
            return terrain_altitude

        return _terrain_altitude(
            self.terrain_altitude_amsl,
            self.terrain_altitude_agl,
            self.terrain_altitude_at_home_amsl,
        )

    @ROS.publish(
        messaging.ROS_TOPIC_TERRAIN_GEOPOINT,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def terrain_altitude_geopoint_stamped(self) -> Optional[GeoPointStamped]:
        """
        Altitude of terrain directly under drone as
        :class:`geographic_msgs.msg.GeoPointStamped` message, or None if not
        available

        Complementary to the Altitude message, includes lat and lon in atomic
        message
        """

        @enforce_types(
            self.get_logger().warn,
            "Cannot generate terrain altitude GeoPointStamped message",
        )
        def _terrain_altitude_geopoint_stamped(geopose_stamped: GeoPoseStamped, terrain_altitude_ellipsoid: float):
            xy = GeoPt(
                x=geopose_stamped.pose.position.longitude,
                y=geopose_stamped.pose.position.latitude,
            )
            geopoint_stamped = GeoPointStamped(
                header=messaging.create_header("base_link"),
                position=GeoPoint(
                    latitude=xy.lat,
                    longitude=xy.lon,
                    altitude=terrain_altitude_ellipsoid,
                ),
            )
            return geopoint_stamped

        return _terrain_altitude_geopoint_stamped(self.geopose_stamped, self.terrain_altitude_ellipsoid)

    @ROS.publish(
        messaging.ROS_TOPIC_EGM96_HEIGHT,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def egm96_height(self) -> Optional[Float32]:
        """EGM96 geoid height at current location, or None if not available"""

        @enforce_types(self.get_logger().warn, "Cannot generate EGM96 geoid height")
        def _egm96_height(geopose_stamped: GeoPoseStamped):
            _egm96_height = self._egm96.height(
                geopose_stamped.pose.position.latitude,
                geopose_stamped.pose.position.longitude,
            )
            return Float32(data=_egm96_height)

        return _egm96_height(self.geopose_stamped)

    @ROS.publish(
        messaging.ROS_TOPIC_ORTHOIMAGE,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def orthoimage_3d(self) -> Optional[OrthoImage3D]:
        """Outgoing orthoimage and elevation raster pair"""

        @enforce_types(self.get_logger().warn, "Cannot create OrthoImage3D message")
        @lru_cache(1)
        def _orthoimage_3d(map_data: MapData):
            return OrthoImage3D(
                bbox=messaging.bounding_box_to_bbox(map_data.bbox),
                img=self._cv_bridge.cv2_to_imgmsg(
                    map_data.image.arr, encoding="passthrough"
                ),
                dem=self._cv_bridge.cv2_to_imgmsg(
                    map_data.elevation.arr, encoding="passthrough"
                )
                # dem=self._cv_bridge.cv2_to_imgmsg(dem, encoding="mono8")
            )  # TODO: need to use mono16? 255 meters max?

        return _orthoimage_3d(self.map_data)

    @property
    def terrain_altitude_amsl(self) -> Optional[float]:
        """
        Altitude of terrain directly under drone above mean sea level AMSL in
        meters, or None if not available
        """

        @enforce_types(self.get_logger().warn, "Cannot generate terrain altitude AMSL")
        def _terrain_altitude_amsl(
            geopose_stamped: GeoPoseStamped, home_geopoint: GeoPointStamped
        ):
            xy = GeoPt(
                x=self.geopose_stamped.pose.position.longitude,
                y=self.geopose_stamped.pose.position.latitude,
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

            return terrain_altitude_amsl

        return _terrain_altitude_amsl(self.geopose, self.home_geopoint)

    @property
    def terrain_altitude_ellipsoid(self) -> Optional[float]:
        """
        Altitude of terrain directly under drone above WGS84 ellipsoid in meters,
        or None if not available
        """

        @enforce_types(
            self.get_logger().warn, "Cannot generate terrain ellipsoidal altitude"
        )
        def _terrain_altitude_ellipsoid(
            terrain_altitude_amsl: float, geopose_stamped: GeoPoseStamped
        ):
            terrain_altitude_ellipsoid = terrain_altitude_amsl + self._egm96.height(
                self.geopose_stamped.pose.position.latitude,
                self.geopose_stamped.pose.position.longitude,
            )
            return terrain_altitude_ellipsoid

        return _terrain_altitude_ellipsoid(
            self.terrain_altitude_amsl, self.geopose_stamped
        )

    @property
    def home_altitude_amsl(self) -> Optional[float]:
        """
        Home position altitude above mean sea level (AMLS) in meters, or None
        if not available
        """

        @enforce_types(self.get_logger().warn, "Cannot generate home altitude AMSL")
        def _home_altitude_amsl(home_geopoint: GeoPointStamped):
            home_altitude_amsl = (
                self.home_geopoint.position.altitude
                - self._egm96.height(
                    self.home_geopoint.position.latitude,
                    self.home_geopoint.position.longitude,
                )
            )
            return home_altitude_amsl

        return _home_altitude_amsl(self.home_geopoint)

    @property
    def terrain_altitude_at_home_amsl(self) -> Optional[float]:
        """
        Terrain altitude above mean sea level (AMSL) at home position, or None
        if not available

        # TODO: check if this definition is correct
        """

        @enforce_types(
            self.get_logger().warn,
            "Cannot generate terrain altitude AMSL at home position",
        )
        def _terrain_altitude_at_home_amsl(
            home_altitude_amsl: float, terrain_altitude_amsl: float
        ):
            # TODO: this is wrong? Need terrain altitude AMSL at home position as input?
            terrain_altitude_at_home_amsl = (
                terrain_altitude_amsl - home_altitude_amsl
                if terrain_altitude_amsl is not None and home_altitude_amsl is not None
                else None
            )
            return terrain_altitude_at_home_amsl

        return _terrain_altitude_at_home_amsl(
            self.home_altitude_amsl, self.terrain_altitude_amsl
        )

    def _get_map(
        self, layers, styles, srs, bbox, size, format_, transparency, grayscale=False
    ) -> Optional[np.ndarray]:
        """Sends WM GetMap request and returns numpy array"""
        self.get_logger().info(
            f"Requesting image for\n"
            f"bbox: {bbox},\n"
            f"layers: {layers},\n"
            f"styles: {styles},\n"
            # f"SRS: {srs},\n"
            # f"size: {size},\n"
            # f"transparency: {transparency},\n"
            # f"format: {format_}."
        )
        try:
            # Do not handle possible requests library related exceptions here
            # (see class docstring)
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
                f"GetMap request for image ran into an unexpected exception: {se}"
            )
            return None
        finally:
            self.get_logger().debug("Image request complete.")

        def _read_img(img: IO, grayscale: bool = False) -> np.ndarray:
            """Reads image bytes and returns numpy array

            :return: Image as np.ndarray
            """
            img = np.frombuffer(img.read(), np.uint8)  # TODO: make DEM uint16?
            img = (
                cv2.imdecode(img, cv2.IMREAD_UNCHANGED)
                if not grayscale
                else cv2.imdecode(img, cv2.IMREAD_GRAYSCALE)
            )
            assert_type(img, np.ndarray)
            return img

        img_ = _read_img(img, grayscale)

        return img_

    def _request_new_map_data(self) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        Sends GetMap request to WMS for image and DEM layers and updates saved
        :attr:`.map_data` attribute.

        Assumes zero raster as DEM if no DEM layer is available.

        TODO: Currently no support for separate arguments for imagery and height
        layers. Assumes height layer is available at same CRS as imagery layer.
        """

        @enforce_types(self.get_logger().warm, "Cannot update map data")
        def _map(
            bbox: BBox, size: Tuple[int, int]
        ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
            layers, styles = (
                self.get_parameter("layers").get_parameter_value().string_array_value,
                self.get_parameter("styles").get_parameter_value().string_array_value,
            )
            assert_len(styles, len(layers))
            assert all(isinstance(x, str) for x in layers)
            assert all(isinstance(x, str) for x in styles)

            dem_layers, dem_styles = (
                self.get_parameter("dem_layers")
                .get_parameter_value()
                .string_array_value,
                self.get_parameter("dem_styles")
                .get_parameter_value()
                .string_array_value,
            )
            assert_len(dem_styles, len(dem_layers))
            assert all(isinstance(x, str) for x in dem_layers)
            assert all(isinstance(x, str) for x in dem_styles)

            srs = self.get_parameter("srs").get_parameter_value().string_value
            format_ = self.get_parameter("format").get_parameter_value().string_value
            transparency = (
                self.get_parameter("transparency").get_parameter_value().bool_value
            )

            self.get_logger().info("Requesting new orthoimage")
            img = self._get_map(layers, styles, srs, bbox, size, format_, transparency)
            if img is None:
                self.get_logger().error("Could not get orthoimage from GIS server")
                return

            dem = None
            if len(dem_layers) > 0 and dem_layers[0]:
                self.get_logger().info("Requesting new DEM")
                dem = self._get_map(
                    layers,
                    styles,
                    srs,
                    bbox,
                    size,
                    format_,
                    transparency,
                    grayscale=True,
                )
            else:
                # Assume flat (:=zero) terrain if no DEM layer provided
                self.get_logger().debug(
                    "No DEM layer provided, assuming flat (=zero) elevation model."
                )
                dem = np.zeros_like(img)

            return img, dem

        bbox = self.bbox
        map = _map(bbox, self.map_size_with_padding)
        if map is not None:
            img, dem = map
            self._map_data = MapData(bbox, Img(img), Img(dem))

    @property
    def _publish_timer(self) -> Timer:
        """:class:`gisnav_msgs.msg.OrthoImage3D` publish and map update timer"""
        return self.__publish_timer

    @_publish_timer.setter
    def _publish_timer(self, value: Timer) -> None:
        assert_type(value, Timer)
        self.__publish_timer = value

    @property
    def map_data(self) -> Optional[MapData]:
        """
        Stored orthoimage and DEM rasters together with associated bounding box,
        or None if not available
        """
        return self._map_data

    @map_data.setter
    def map_data(self, value: MapData) -> None:
        self._map_data = value

    @property
    def _origin_dem_altitude(self) -> Optional[float]:
        """Elevation layer (DEM) altitude at local frame origin"""
        return self.__origin_dem_altitude

    @_origin_dem_altitude.setter
    def _origin_dem_altitude(self, value: Optional[float]) -> None:
        assert_type(value, get_args(Optional[float]))
        self.__origin_dem_altitude = value

    # region rclpy subscriber callbacks

    def image_callback(self, msg: Image) -> None:
        """Receives :class:`sensor_msgs.msg.Image` message"""
        # Do nothing - enforced by CameraSubscriberNode parent

    # endregion rclpy subscriber callbacks

    def _should_request_new_map_data(self, bbox: BBox) -> bool:
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
        if self.bounding_box is not None:
            if self.orthoimage_3d is not None:
                bbox_previous = messaging.bounding_box_to_bbox(self.orthoimage_3d.bbox)
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

    def _terrain_altitude_at_position(
        self, position: Optional[GeoPt], local_origin: bool = False
    ) -> Optional[float]:
        """Raw terrain altitude from DEM if available, or None if not available

        :param position: Position to query
        :param local_origin: True to use :py:attr:`._home_dem` (retrieved
            specifically for local frame origin)
        :return: Raw altitude in DEM coordinate frame and units
        """
        map_data = self.map_data if not local_origin else self._home_dem
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
            of :py:attr:`.map_data`
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
