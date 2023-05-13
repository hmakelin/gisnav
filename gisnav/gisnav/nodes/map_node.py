"""Contains :class:`.Node` that provides :class:`OrthoImage3D`s"""
import time
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

from ..assertions import ROS, assert_len, assert_type, cache_if, enforce_types
from ..data import BBox, Img
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

    Downloads and stores orthophoto and optional DEM (together called orthoimage)
    from WMS for location of projected camera field of view.

    Subscribes to :class:`.CameraInfo` and :class:`.FOV` messages to determine
    bounding box for next orthoimage to be cached. Requests new map whenever
    ground-projected FOV overlap with bounding box of current cached orthoimage
    gets too small. Publishes a :class:`.OrthoImage3D`with a high-resolution
    orthophoto and an optional digital elevation model (DEM) that can be used
    for pose estimation.

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

    _WMS_CONNECTION_ATTEMPT_DELAY_SEC = 10
    """
    Delay in seconds until a new WMS connection is attempted in case of
    connection error
    """

    # TODO: remove once all nodes have @ROS.parameters decoration
    ROS_PARAM_DEFAULTS = []
    """List containing ROS parameter name, default value and read_only flag tuples"""

    def __init__(self, name: str):
        """Class initializer

        :param name: Node name
        """
        super().__init__(name)

        # Calling these decorated properties the first time will setup
        # subscriptions to the appropriate ROS topics
        self.bounding_box
        self.geopose
        self.home_geopoint

        publish_rate = (
            self.get_parameter("publish_rate").get_parameter_value().integer_value
        )
        self._publish_timer = self._create_publish_timer(publish_rate)

        # TODO: make configurable / use shared folder home path instead
        self._egm96 = GeoidPGM("/usr/share/GeographicLib/geoids/egm96-5.pgm", kind=-3)

        self._cv_bridge = CvBridge()

        # TODO: WMS connection handle disconnects, declare property
        self._connected = False
        self._connect_wms()

    def _connect_wms(self) -> None:
        """Connects to WMS server"""
        url = self.get_parameter("url").get_parameter_value().string_value
        version = self.get_parameter("version").get_parameter_value().string_value
        timeout = self.get_parameter("timeout").get_parameter_value().integer_value
        while not self.connected:
            try:
                self.get_logger().info("Connecting to WMS endpoint...")
                self._wms_client = WebMapService(url, version=version, timeout=timeout)
                self.connected = True
            except requests.exceptions.ConnectionError as _:  # noqa: F841
                self.get_logger().error(
                    f"Could not connect to WMS endpoint, trying again in "
                    f"{self._WMS_CONNECTION_ATTEMPT_DELAY_SEC} seconds..."
                )
                time.sleep(self._WMS_CONNECTION_ATTEMPT_DELAY_SEC)

        # TODO: any other errors that might prevent connection?
        #  handle disconnect & reconnect
        self.get_logger().info("WMS client setup complete.")

    @property
    def connected(self) -> bool:
        """True if connected to WMS server"""
        return self._connected

    @connected.setter
    def connected(self, value: bool) -> bool:
        self._connected = value

    def _should_update_dem_height_at_local_origin(self):
        # TODO: Update if local frame changes
        if self._dem_height_at_local_origin is None:
            return True

    @property
    @cache_if(_should_update_dem_height_at_local_origin)
    def dem_height_at_local_origin(self) -> Optional[float]:
        """
        Elevation layer (DEM) height in meters at local frame origin

        .. note::
            Assumes home GeoPoint is also the local frame origin.
        """
        return self._dem_height_at_geopoint(self.home_geopoint)

    def bounding_box_cb(self, msg: BoundingBox) -> None:
        """Callback for :class:`geographic_msgs.msg.BoundingBox` messages"""

        # Request new if needed and publish
        self.orthoimage_3d

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

    @property
    @ROS.publish(
        messaging.ROS_TOPIC_TERRAIN_ALTITUDE,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def terrain_altitude(self) -> Optional[Altitude]:
        """Altitude of terrain directly under drone, or None if not available"""

        @enforce_types(
            self.get_logger().warn, "Cannot generate terrain altitude message"
        )
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

    @property
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
        def _terrain_altitude_geopoint_stamped(
            geopose_stamped: GeoPoseStamped, terrain_altitude_ellipsoid: float
        ):
            geopoint_stamped = GeoPointStamped(
                header=messaging.create_header("base_link"),
                position=GeoPoint(
                    latitude=geopose_stamped.pose.position.latitude,
                    longitude=geopose_stamped.pose.position.longitude,
                    altitude=terrain_altitude_ellipsoid,
                ),
            )
            return geopoint_stamped

        return _terrain_altitude_geopoint_stamped(
            self.geopose_stamped, self.terrain_altitude_ellipsoid
        )

    @property
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

    def request_orthoimage_for_bounding_box(self, bounding_box: BoundingBox):
        """
        Sends GetMap request to GIS WMS for image and DEM layers and returns
        :attr:`.orthoimage_3d` attribute.

        Assumes zero raster as DEM if no DEM layer is available.

        TODO: Currently no support for separate arguments for imagery and height
        layers. Assumes height layer is available at same CRS as imagery layer.

        :param bounding_box: BoundingBox to request the orthoimage for
        """

        @enforce_types(self.get_logger().warm, "Cannot request orthoimage")
        def _request_orthoimage_for_bounding_box(
            bounding_box: BoundingBox, size: Tuple[int, int]
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

            bbox = messaging.bounding_box_to_bbox(bounding_box)

            self.get_logger().info("Requesting new orthoimage")
            img: np.ndarray = self._get_map(
                layers, styles, srs, bbox, size, format_, transparency
            )
            if img is None:
                self.get_logger().error("Could not get orthoimage from GIS server")
                return

            dem: Optional[np.ndarray] = None
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

            assert img.ndim == dem.ndim == 3
            return img, dem

        return _request_orthoimage_for_bounding_box(bounding_box, self.map_size_with_padding)

    def _should_request_orthoimage(self) -> bool:
        """Returns True if a new orthoimage (including DEM) should be requested
        from onboard GIS

        This check is made to avoid retrieving a new orthoimage that is almost
        the same as the previous orthoimage. Relaxing orthoimage update constraints
        should not improve accuracy of position estimates unless the orthoimage
        is so old that the field of view either no longer completely fits inside
        (vehicle has moved away or camera is looking in other direction) or is
        too small compared to the size of the orthoimage (vehicle altitude has
        significantly decreased).

        :return: True if new orthoimage should be requested from onboard GIS
        """
        @enforce_types(self.get_logger().warn, "Cannot determine if orthoimage should be updated")
        def _should_request_orthoimage(new_bounding_box: BoundingBox, old_orthoimage: OrthoImage3D):
            bbox = messaging.bounding_box_to_bbox(new_bounding_box)
            bbox_previous = messaging.bounding_box_to_bbox(old_orthoimage.bbox)
            threshold = (
                self.get_parameter("map_overlap_update_threshold")
                .get_parameter_value()
                .double_value
            )
            bbox1, bbox2 = box(*bbox), box(*bbox_previous)
            ratio1 = bbox1.intersection(bbox2).area / bbox1.area
            ratio2 = bbox2.intersection(bbox1).area / bbox2.area
            ratio = min(ratio1, ratio2)
            if ratio > threshold:
                return False

            return True

        # Access self._orthoimage_3d directly to prevent recursion since this is
        # used as @cache_if predicate for self.orthoimage_3d
        return _should_request_orthoimage(self.bounding_box, self._orthoimage_3d)

    @property
    @ROS.publish(
        messaging.ROS_TOPIC_ORTHOIMAGE,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    @cache_if(_should_request_orthoimage)
    def orthoimage_3d(self) -> Optional[OrthoImage3D]:
        """Outgoing orthoimage and elevation raster pair"""

        bounding_box = self.bounding_box
        map = self._request_orthoimage(bounding_box, self.map_size_with_padding)
        if map is not None:
            img, dem = map
            return OrthoImage3D(
                bbox=bounding_box,
                img=self._cv_bridge.cv2_to_imgmsg(img, encoding="passthrough"),
                dem=self._cv_bridge.cv2_to_imgmsg(dem, encoding="passthrough")
                # dem=self._cv_bridge.cv2_to_imgmsg(dem, encoding="mono8")
            )  # TODO: need to use mono16? 255 meters max?

    def _should_request_orthoimage_3d_at_local_origin(self) -> bool:
        """
        True if an OrthoImage3D should be requested for the local frame origin
        GeoPoint
        """
        # TODO: need to re-request if local origin changes, see also
        # _should_update_dem_height_at_local_origin for same issue
        return self._orthoimage_3d_at_local_origin is not None

    @property
    @cache_if(_should_request_orthoimage_3d_at_local_origin)
    def orthoimage_3d_at_local_origin(self) -> Optional[OrthoImage3D]:
        """
        Orthoimage for local frame origin

        Used to get DEM height for local origin, which is then used to determine
        DEM AMSL height since MapNode is ignorant of DEM vertical datum
        """
        bounding_box = # TODO - get bbox for local frame origin - make it up from scratch or translate from self.bounding_box?
        map = self._request_orthoimage(bounding_box, self.map_size_with_padding)
        if map is not None:
            img, dem = map
            return OrthoImage3D(
                bbox=bounding_box,
                img=self._cv_bridge.cv2_to_imgmsg(img, encoding="passthrough"),
                dem=self._cv_bridge.cv2_to_imgmsg(dem, encoding="passthrough")
                # dem=self._cv_bridge.cv2_to_imgmsg(dem, encoding="mono8")
            )  # TODO: need to use mono16? 255 meters max?

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
            terrain_altitude_amsl = self._terrain_altitude_amsl_at_geopoint(
                geopose_stamped.position
            )

            if terrain_altitude_amsl is None:
                # Probably have not received bbox yet so no map_data, try the
                # data that was retrieved for local position origin instead
                # (assume we are at starting position)
                self.get_logger().warn(
                    "Could not get terrain altitude amsl for position from map "
                    "data for publishing geopoint, trying DEM which is intended "
                    "for local origin..."
                )
                terrain_altitude_amsl = self._terrain_altitude_amsl_at_geopoint(
                    geopose_stamped.position,
                    True,  # TODO: ground track orthoimage, not local origin orthoimage?
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
                geopose_stamped.pose.position.latitude,
                geopose_stamped.pose.position.longitude,
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
            home_altitude_amsl = home_geopoint.position.altitude - self._egm96.height(
                home_geopoint.position.latitude,
                home_geopoint.position.longitude,
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
            f"Sending GetMap request for bbox: {bbox}, layers: {layers}."
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

            :param img: Image bytes buffer
            :param grayscale: True if buffer represents grayscale image
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

        return _read_img(img, grayscale)

    @property
    def _publish_timer(self) -> Timer:
        """:class:`gisnav_msgs.msg.OrthoImage3D` publish and map update timer"""
        return self.__publish_timer

    @_publish_timer.setter
    def _publish_timer(self, value: Timer) -> None:
        assert_type(value, Timer)
        self.__publish_timer = value

    def image_callback(self, msg: Image) -> None:
        """Receives :class:`sensor_msgs.msg.Image` message"""
        # Do nothing - enforced by CameraSubscriberNode parent

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

    def _dem_height_at_geopoint(
        self, geopoint: Optional[GeoPoint], local_origin: bool = False
    ) -> Optional[float]:
        """
        Raw DEM height in meters from DEM if available, or None if not
        available

        DEM values are expected to be requested for (1) the vehicle's current
        position, and (2) the local frame origin. DEM value for the local frame
        origin is needed to determine the AMSL height of the DEM, which may
        be expressed in some other vertical datum. Since the local frame origin
        generally is not inside :attr:`.orthoimage_3d`, a separate flag can be
        given as input argument to specifically request retrieving DEM height
        from :attr:`.orthoimage_3d_local_origin`.

        .. note::
            The vertical datum for the USGS DEM is NAVD 88. Other DEMs may
            have other vertical data, this method is agnostic to but assumes
            the units are meters. **Within the flight mission area the vertical
            datum of the DEM is assumed to be flat**.

        :param geopoint: Position to query
        :param local_origin: True to use :py:attr:`._home_dem` (retrieved
            specifically for local frame origin).
        :return: Raw altitude in DEM coordinate frame and units
        """

        @enforce_types(
            self.get_logger().warn, "Cannot determin if geopoint is inside bounding box"
        )
        def _is_geopoint_inside_bounding_box(
            geopoint: GeoPoint, bounding_box: BoundingBox
        ) -> bool:
            if (
                geopoint.latitude <= bounding_box.top_left.latitude
                and geopoint.latitude >= bounding_box.bottom_right.latitude
                and geopoint.longitude >= bounding_box.top_left.longitude
                and geopoint.longitude <= bounding_box.top_right.longitude
            ):
                return True
            else:
                return False

        @enforce_types(
            self.get_logger().warn, "Cannot determine terrain altitude at position"
        )
        def _dem_height_at_geopoint(
            orthoimage: OrthoImage3D, geopoint: GeoPoint
        ) -> Optional[float]:
            dem = self._cv_bridge.imgmsg_to_cv2(
                orthoimage.dem, desired_encoding="passthrough"
            )  # TODO: 255 meters max? use np.uint16

            if _is_geopoint_inside_bounding_box(geopoint, orthoimage.bbox):
                # Normalized coordinates of the GeoPoint in the BoundingBox
                bounding_box = orthoimage.bbox
                u = (geopoint.longitude - bounding_box.min_corner.longitude) / (
                    bounding_box.max_corner.longitude
                    - bounding_box.min_corner.longitude
                )
                v = (geopoint.latitude - bounding_box.min_corner.latitude) / (
                    bounding_box.max_corner.latitude - bounding_box.min_corner.latitude
                )

                # Pixel coordinates in the DEM image
                x_pixel = int(u * (orthoimage.dem.width - 1))
                y_pixel = int(v * (orthoimage.dem.height - 1))

                # DEM height at the pixel coordinates
                dem_altitude = dem[y_pixel, x_pixel]
                return dem_altitude
            else:
                return None

        orthoimage = (
            self.orthoimage_3d if not local_origin else self.orthoimage_3d_at_local_origin
        )
        return _dem_height_at_geopoint(orthoimage, geopoint)

    def _terrain_altitude_amsl_at_geopoint(
        self, geopoint: Optional[GeoPoint], local_origin: bool = False
    ) -> Optional[float]:
        """Terrain altitude in meters AMSL according to DEM if available, or
        None if not available

        :param geopoint: GeoPoint to query
        :param local_origin: True to use :py:attr:`._home_dem` (retrieved
            specifically for local frame origin).
        :return: Terrain altitude AMSL in meters at position
        """

        @enforce_types(
            self.get_logger().warn, "Cannot compute terrain altitude AMSL at position"
        )
        def _terrain_altitude_amsl_at_position(
            dem_meters_position: float,
            dem_meters_origin: float,
            home_geopoint: GeoPointStamped,
        ):
            elevation_relative = dem_meters_position - dem_meters_origin
            elevation_amsl = (
                elevation_relative
                + home_geopoint.position.altitude
                - self._egm96.height(
                    home_geopoint.position.latitude,
                    home_geopoint.position.longitude,
                )
            )
            return float(elevation_amsl)

        dem_elevation = self._dem_height_at_geopoint(geopoint, local_origin)
        return _terrain_altitude_amsl_at_position(
            dem_elevation, self._dem_height_at_local_origin, self.home_geopoint
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
        if (
            self._should_request_dem_for_local_frame_origin()
        ):  # TODO: should request ground track orthoimage? but local oriin is needed too?
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
                    if self._dem_height_at_local_origin is None:
                        if self._home_geopoint is not None:
                            self._dem_height_at_local_origin = self._dem_height_at_geopoint(
                                xy, local_origin=True
                            )
                else:
                    self.get_logger().warn(
                        "Required map size unknown, skipping requesting DEM for home."
                    )

        if self._ortho_image_3d_msg is not None:
            self._ortho_image_3d_pub.publish(self._ortho_image_3d_msg)

        self._publish_terrain_altitude()
