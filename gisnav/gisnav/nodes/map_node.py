"""Contains :class:`.Node` that provides :class:`OrthoImage3D`s"""
import time
import xml.etree.ElementTree as ET
from collections import deque
from typing import IO, Final, List, Optional, Tuple

import cv2
import numpy as np
import requests
from cv_bridge import CvBridge
from geographic_msgs.msg import (
    BoundingBox,
    GeoPoint,
    GeoPointStamped,
    GeoPose,
    GeoPoseStamped,
)
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_msgs.msg import Altitude, GimbalDeviceAttitudeStatus, HomePosition
from owslib.util import ServiceException
from owslib.wms import WebMapService
from pygeodesy.ellipsoidalVincenty import LatLon
from pygeodesy.geoids import GeoidPGM
from pyproj import Transformer
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.timer import Timer
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, NavSatFix
from shapely.geometry import box

from gisnav_msgs.msg import OrthoImage3D  # type: ignore

from ..assertions import ROS, assert_len, assert_type, cache_if, narrow_types
from ..geo import get_dynamic_map_radius
from . import messaging


class MapNode(Node):
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

    _WMS_CONNECTION_ATTEMPT_DELAY_SEC = 10
    """
    Delay in seconds until a new WMS connection is attempted in case of
    connection error
    """

    _DELAY_SLOW_MS = 10000
    """
    Max delay for messages where updates are not needed nor expected often,
    e.g. home position
    """

    _DELAY_NORMAL_MS = 2000
    """Max delay for things like global position"""

    _DELAY_FAST_MS = 500
    """
    Max delay for messages with fast dynamics that go "stale" quickly, e.g.
    local position and attitude. The delay can be a bit higher than is
    intuitive because the vehicle EKF should be able to fuse things with
    fast dynamics with higher lags as long as the timestamps are accurate.
    """

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
    """Required overlap ratio between suggested new :term:`bounding box` and current
    :term:`orthoimage` bounding box, under which a new map will be requested.
    """

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

    #: Publishing topic for :class:`nav_msgs.msg.Path` messages for rviz
    ROS_TOPIC_PATH = "~/path"

    #: Max limit for held :class:`geometry_msgs.msg.PoseStamped` messages
    _MAX_POSE_STAMPED_MESSAGES = 100

    _ROS_PARAM_DESCRIPTOR_READ_ONLY: Final = ParameterDescriptor(read_only=True)
    """A read only ROS parameter descriptor"""

    @ROS.setup_node([])
    def __init__(self, name: str):
        """Class initializer

        :param name: Node name
        """
        # super().__init__(name)  # Handled by setup_node decorator

        # Calling these decorated properties the first time will setup
        # subscriptions to the appropriate ROS topics
        # self.bounding_box
        self.pose
        self.camera_info
        self.nav_sat_fix
        self.home_position
        self.gimbal_device_attitude_status

        self._pose_stamped_queue: deque = deque(maxlen=self._MAX_POSE_STAMPED_MESSAGES)

        # TODO: use throttling in publish decorator, remove timer
        publish_rate = self.publish_rate
        assert publish_rate is not None
        self._publish_timer = self._create_publish_timer(publish_rate)

        # TODO: make configurable / use shared folder home path instead
        self._egm96 = GeoidPGM("/usr/share/GeographicLib/geoids/egm96-5.pgm", kind=-3)
        self._transformer = Transformer.from_crs("EPSG:4326", "EPSG:3857")

        # TODO: refactor out CvBridge and use np.frombuffer instead
        self._cv_bridge = CvBridge()

        # TODO: WMS connection handle disconnects, declare property
        self._connected = False
        self._connect_wms(self.wms_url, self.wms_version, self.wms_timeout)

    @property
    @ROS.parameter(ROS_D_URL, descriptor=_ROS_PARAM_DESCRIPTOR_READ_ONLY)
    def wms_url(self) -> Optional[str]:
        """WMS client endpoint URL"""

    @property
    @ROS.parameter(ROS_D_VERSION, descriptor=_ROS_PARAM_DESCRIPTOR_READ_ONLY)
    def wms_version(self) -> Optional[str]:
        """Used WMS protocol version"""

    @property
    @ROS.parameter(ROS_D_TIMEOUT, descriptor=_ROS_PARAM_DESCRIPTOR_READ_ONLY)
    def wms_timeout(self) -> Optional[int]:
        """WMS request timeout in seconds"""

    @property
    @ROS.parameter(ROS_D_LAYERS)
    def wms_layers(self) -> Optional[List[str]]:
        """WMS request layers for :term:`orthophoto` :term:`GetMap` requests"""

    @property
    @ROS.parameter(ROS_D_DEM_LAYERS)
    def wms_dem_layers(self) -> Optional[List[str]]:
        """WMS request layers for :term:`DEM` :term:`GetMap` requests"""

    @property
    @ROS.parameter(ROS_D_STYLES)
    def wms_styles(self) -> Optional[List[str]]:
        """WMS request styles for :term:`orthophoto` :term:`GetMap` requests"""

    @property
    @ROS.parameter(ROS_D_DEM_STYLES)
    def wms_dem_styles(self) -> Optional[List[str]]:
        """WMS request styles for :term:`DEM` :term:`GetMap` requests"""

    @property
    @ROS.parameter(ROS_D_SRS)
    def wms_srs(self) -> Optional[str]:
        """WMS request :term:`SRS` for all :term:`GetMap` requests"""

    @property
    @ROS.parameter(ROS_D_IMAGE_TRANSPARENCY)
    def wms_transparency(self) -> Optional[bool]:
        """WMS request transparency for all :term:`GetMap` requests"""

    @property
    @ROS.parameter(ROS_D_IMAGE_FORMAT)
    def wms_format(self) -> Optional[str]:
        """WMS request format for all :term:`GetMap` requests"""

    @property
    @ROS.parameter(ROS_D_MAP_OVERLAP_UPDATE_THRESHOLD)
    def min_map_overlap_update_threshold(self) -> Optional[float]:
        """Required :term:`bounding box` overlap ratio for new :term:`GetMap`
        requests

        If the overlap between the candidate new bounding box and the current
        :term:`orthoimage` bounding box is below this value, a new map will be
        requested.
        """

    @property
    @ROS.parameter(ROS_D_MAX_MAP_RADIUS)
    def max_map_radius(self) -> Optional[int]:
        """Max radius in meters for circle inside the requested :term:`bounding box`
        (half map side length)
        """

    @property
    @ROS.parameter(ROS_D_PUBLISH_RATE, descriptor=_ROS_PARAM_DESCRIPTOR_READ_ONLY)
    def publish_rate(self) -> Optional[int]:
        """Publish rate in Hz for the :attr:`.orthoimage_3d` :term:`message`"""

    @narrow_types
    def _create_publish_timer(self, publish_rate: int) -> Timer:
        """
        Returns a timer to publish :attr:`.orthoimage_3d` to ROS

        :param publish_rate: Publishing rate for the timer (in Hz)
        :return: The :class:`.Timer` instance
        """
        if publish_rate <= 0:
            error_msg = (
                f"Map update rate must be positive ({publish_rate} Hz provided)."
            )
            self.get_logger().error(error_msg)
            raise ValueError(error_msg)
        timer = self.create_timer(1 / publish_rate, self.publish)
        return timer

    @property
    def _publish_timer(self) -> Timer:
        """:class:`gisnav_msgs.msg.OrthoImage3D` publish and map update timer"""
        return self.__publish_timer

    @_publish_timer.setter
    def _publish_timer(self, value: Timer) -> None:
        self.__publish_timer = value

    def publish(self):
        """
        Publish :attr:`.orthoimage_3d` (:attr:`.terrain_altitude` and
        :attr:`.terrain_geopoint_stamped` are also published but that
        publish is triggered by callbacks since the messages are smaller and
        can be published more often)
        """
        self.orthoimage_3d

    @narrow_types
    def _connect_wms(self, url: str, version: str, timeout: int) -> None:
        """Connects to WMS server"""
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
    def connected(self, value: bool) -> None:
        self._connected = value

    @narrow_types
    def _bounding_box_with_padding_for_latlon(
        self, latitude: float, longitude: float, padding: float = 100.0
    ):
        """Adds 100 meters of padding to coordinates on both sides"""
        meters_in_degree = 111045.0  # at 0 latitude
        lat_degree_meter = meters_in_degree
        lon_degree_meter = meters_in_degree * np.cos(np.radians(latitude))

        delta_lat = padding / lat_degree_meter
        delta_lon = padding / lon_degree_meter

        bounding_box = BoundingBox()

        bounding_box.min_pt = GeoPoint()
        bounding_box.min_pt.latitude = latitude - delta_lat
        bounding_box.min_pt.longitude = longitude - delta_lon

        bounding_box.max_pt = GeoPoint()
        bounding_box.max_pt.latitude = latitude + delta_lat
        bounding_box.max_pt.longitude = longitude + delta_lon

        return bounding_box

    def _should_update_dem_height_at_local_origin(self):
        # TODO: Update if local frame changes
        if self._dem_height_meters_at_local_origin is None:
            return True

    @narrow_types
    def _dem_height_meters_at_latlon_wms(
        self,
        lat: float,
        lon: float,
        srs: str,
        format_: str,
        dem_layers: List[str],
        dem_styles: List[str],
    ) -> Optional[float]:
        """Digital elevation model (DEM) height in meters at given coordinates

        Uses WMS GetFeatureInfo to get the value frome the onboard GIS server.
        """
        assert_len(dem_styles, len(dem_layers))

        bounding_box = self._bounding_box_with_padding_for_latlon(lat, lon)
        bbox = messaging.bounding_box_to_bbox(bounding_box)

        height = self._get_feature_info(
            dem_layers, dem_styles, srs, bbox, (3, 3), format_, xy=(1, 1)
        )
        if height is None:
            self.get_logger().error("Could not get DEM height from GIS server")
            return None

        return height

    @property
    @cache_if(_should_update_dem_height_at_local_origin)
    def dem_height_meters_at_local_origin(self) -> Optional[float]:
        """
        Digital elevation model (DEM) height in meters at local frame origin

        DEM values are expected to be requested for (1) the vehicle's current
        position, and (2) the local frame origin. DEM value for the local frame
        origin is needed to determine the AMSL height of the DEM, which may
        be expressed in some other vertical datum. Since the local frame origin
        generally is not inside :attr:`.orthoimage_3d`, a separate request
        retrieving DEM height for local frame origin using a WMS GetFeatureInfo
        request is implemnted here (this has a bit of latency but is OK for the
        given use case of retrieving it typically only once and then caching it).

        .. note::
            Assumes HomePosition is also the local frame origin.
        """
        home_position = self.home_position
        if home_position is not None:
            return self._dem_height_meters_at_latlon_wms(
                home_position.geo.latitude,
                home_position.geo.longitude,
                self.wms_srs,
                self.wms_format,
                self.wms_dem_layers,
                self.wms_dem_styles,
            )
        else:
            self.get_logger().error("Home geopoint none, cannot get bbox wit padding)")
            return None

    def nav_sat_fix_cb(self, msg: NavSatFix) -> None:
        """Callback for :class:`mavros_msgs.msg.NavSatFix` message

        Publishes vehicle :class:`.geographic_msgs.msg.GeoPoseStamped` and
        :class:`mavros_msgs.msg.Altitude` because the contents of those messages
        are affected by this update.

        :param msg: :class:`mavros_msgs.msg.NavSatFix` message from MAVROS
        """
        self.ground_track_altitude
        self.ground_track_geopoint_stamped
        self.altitude
        self.geopose

        # TODO: temporarily assuming static camera so publishing gimbal quat here
        # self.gimbal_quaternion

    @property
    @ROS.max_delay_ms(_DELAY_NORMAL_MS)
    @ROS.subscribe(
        "/mavros/global_position/global",
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=nav_sat_fix_cb,
    )
    def nav_sat_fix(self) -> Optional[NavSatFix]:
        """Vehicle GPS fix, or None if unknown or too old"""

    @property
    @ROS.publish(
        messaging.ROS_TOPIC_VEHICLE_GEOPOSE, QoSPresetProfiles.SENSOR_DATA.value
    )
    def geopose(self) -> Optional[GeoPoseStamped]:
        """Vehicle pose as :class:`geographic_msgs.msg.GeoPoseStamped` message
        or None if not available"""

        @narrow_types(self)
        def _geopose(nav_sat_fix: NavSatFix, pose_stamped: PoseStamped):
            # Position
            latitude, longitude = (
                nav_sat_fix.latitude,
                nav_sat_fix.longitude,
            )
            altitude = nav_sat_fix.altitude

            # Convert ENU->NED + re-center yaw
            enu_to_ned = Rotation.from_euler("XYZ", np.array([np.pi, 0, np.pi / 2]))
            attitude_ned = (
                Rotation.from_quat(
                    messaging.as_np_quaternion(pose_stamped.pose.orientation)
                )
                * enu_to_ned.inv()
            )
            rpy = attitude_ned.as_euler("XYZ", degrees=True)
            rpy[0] = (rpy[0] + 180) % 360
            attitude_ned = Rotation.from_euler("XYZ", rpy, degrees=True)
            attitude_ned = attitude_ned.as_quat()
            orientation = messaging.as_ros_quaternion(attitude_ned)

            return GeoPoseStamped(
                header=messaging.create_header("base_link"),
                pose=GeoPose(
                    position=GeoPoint(
                        latitude=latitude, longitude=longitude, altitude=altitude
                    ),
                    orientation=orientation,  # TODO: is this NED or ENU?
                ),
            )

        return _geopose(self.nav_sat_fix, self.pose)

    def pose_stamped_cb(self, msg: PoseStamped) -> None:
        """Callback for :class:`geometry_msgs.msg.Pose` message

        Publishes :class:`nav_msgs.msg.Path` of :class:`.Pose` messages for
        visualization in RViz.

        :param msg: :class:`.PoseStamped` message from MAVROS
        """
        # Update visualization if some time has passed, but not too soon. This
        # is mainly to prevent the Paths being much shorter in time for nodes
        # that would otherwise publish at much higher frequency (e.g. actual
        # GPS at 10 Hz vs GISNav mock GPS at 1 Hz)

        # if len(self._pose_stamped_queue) > 0:
        #    if (
        #        msg.header.stamp.sec
        #        - self._pose_stamped_queue[-1].header.stamp.sec
        #        > 1.0
        #    ):
        #        self._pose_stamped_queue.append(msg)
        #    else:
        #        # Observation is too recent, return
        #        return
        # else:
        #    # Queue is empty
        #    self._pose_stamped_queue.append(msg)

        # Publish to rviz
        # assert len(self._pose_stamped_queue) > 0
        self.pose_stamped

    @property
    # @ROS.max_delay_ms(_DELAY_FAST_MS)  # TODO:
    @ROS.subscribe(
        "/mavros/local_position/pose",
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=pose_stamped_cb,
    )
    def pose(self) -> Optional[PoseStamped]:
        """Vehicle local position, or None if not available or too old"""

    @property
    @ROS.publish(
        "~/pose",
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def pose_stamped(self) -> Optional[PoseStamped]:
        @narrow_types(self)
        def _pose_stamped(
            geopose_stamped: GeoPoseStamped, altitude: Altitude
        ) -> Optional[PoseStamped]:
            """
            Publish :class:`geometry_msgs.msg.PoseStamped` and
            :class:`nav_msgs.msg.Path` messages

            :param geopose_stamped: Vehicle geopose
            :param alt_agl: Vehicle altitude above ground level (RViz config is
                assumed to define a planar ground surface at z=0)
            """
            alt_agl = altitude.terrain
            # Convert latitude, longitude, and altitude to Cartesian coordinates
            x, y = self._transformer.transform(
                geopose_stamped.pose.position.latitude,
                geopose_stamped.pose.position.longitude,
            )
            z = alt_agl

            # Create a PoseStamped message
            pose_stamped = PoseStamped()
            pose_stamped.header = geopose_stamped.header
            pose_stamped.header.frame_id = "map"

            # TODO: x should be easting but re-centered to 0 for ksql_airport.world
            pose_stamped.pose.position.x = x + 13609376

            # TODO: y should be northing but re-centered to 0 for ksql_airport.world
            pose_stamped.pose.position.y = y - 4512349
            pose_stamped.pose.position.z = z
            pose_stamped.pose.orientation = geopose_stamped.pose.orientation

            # Update visualization if some time has passed, but not too soon. This
            # is mainly to prevent the Paths being much shorter in time for nodes
            # that would otherwise publish at much higher frequency (e.g. actual
            # GPS at 10 Hz vs GISNav mock GPS at 1 Hz)
            if len(self._pose_stamped_queue) > 0:
                if (
                    pose_stamped.header.stamp.sec
                    - self._pose_stamped_queue[-1].header.stamp.sec
                    > 1.0
                ):
                    self._pose_stamped_queue.append(pose_stamped)
                else:
                    # Observation is too recent, return
                    return None
            else:
                # Queue is empty
                self._pose_stamped_queue.append(pose_stamped)

            return pose_stamped

        return _pose_stamped(self.geopose, self.altitude)

    @property
    # @ROS.max_delay_ms(2000) - camera info has no header (?)
    @ROS.subscribe(messaging.ROS_TOPIC_CAMERA_INFO, QoSPresetProfiles.SENSOR_DATA.value)
    def camera_info(self) -> Optional[CameraInfo]:
        """Camera info for determining appropriate :attr:`.orthoimage_3d` resolution"""

    @property
    def _orthoimage_size(self) -> Optional[Tuple[int, int]]:
        """
        Padded map size tuple (height, width) or None if the information
        is not available.

        Because the deep learning models used for predicting matching keypoints
        or poses between camera image frames and map rasters are not assumed to
        be rotation invariant in general, the orthoimage rasters are rotated
        based on camera yaw so that they align with the camera images. To keep
        the scale of the raster after rotation unchanged, black corners would
        appear unless padding is used. Retrieved maps therefore have to be
        squares with the side lengths matching the diagonal of the camera frames
        so that scale is preserved and no black corners appear in the rasters
        after arbitrary 2D rotation. The height and width will both be equal to
        the diagonal of the declared camera frame dimensions.
        """

        @narrow_types(self)
        def _orthoimage_size(camera_info: CameraInfo):
            diagonal = int(
                np.ceil(np.sqrt(camera_info.width**2 + camera_info.height**2))
            )
            return diagonal, diagonal

        return _orthoimage_size(self.camera_info)

    @property
    @ROS.publish(
        messaging.ROS_TOPIC_TERRAIN_ALTITUDE,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def ground_track_altitude(self) -> Optional[Altitude]:
        """Altitude of vehicle ground track, or None if not available"""

        @narrow_types(self)
        def _ground_track_altitude(
            ground_track_altitude_amsl: float,
            ground_track_altitude_at_home_amsl: float,
        ):
            # Define local == -relative, and terrain == bottom_clearance
            return Altitude(
                header=messaging.create_header("base_link"),
                amsl=ground_track_altitude_amsl,
                local=ground_track_altitude_amsl - ground_track_altitude_at_home_amsl,
                relative=-(
                    ground_track_altitude_amsl - ground_track_altitude_at_home_amsl
                ),
                terrain=0.0,
                bottom_clearance=0.0,
            )

        return _ground_track_altitude(
            self._ground_track_altitude_amsl, self._ground_track_altitude_at_home_amsl
        )

    @property
    @ROS.publish(
        messaging.ROS_TOPIC_TERRAIN_GEOPOINT,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def ground_track_geopoint_stamped(self) -> Optional[GeoPointStamped]:
        """
        Vehicle ground track as :class:`geographic_msgs.msg.GeoPointStamped`
        message, or None if not available

        Complementary to the Altitude message, includes lat and lon in atomic
        message
        """

        @narrow_types(self)
        def _ground_track_geopoint_stamped(
            geopose: GeoPoseStamped, ground_track_altitude_ellipsoid: float
        ):
            return GeoPointStamped(
                header=messaging.create_header("base_link"),
                position=GeoPoint(
                    latitude=geopose.pose.position.latitude,
                    longitude=geopose.pose.position.longitude,
                    altitude=ground_track_altitude_ellipsoid,
                ),
            )

        return _ground_track_geopoint_stamped(
            self.geopose, self._ground_track_altitude_ellipsoid
        )

    @property
    @ROS.publish(
        messaging.ROS_TOPIC_VEHICLE_ALTITUDE, QoSPresetProfiles.SENSOR_DATA.value
    )
    def altitude(self) -> Optional[Altitude]:
        """Vehicle altitude, or None if unknown or too old"""

        @narrow_types(self)
        def _altitude(
            geopose: GeoPoseStamped,
            egm96_height: float,  # Float32,
            ground_track_altitude: Altitude,
            altitude_local: Optional[float],
        ):
            altitude_amsl = geopose.pose.position.altitude - egm96_height
            local = altitude_local if altitude_local is not None else np.nan

            # Define local == -relative, terrain == bottom_clearance
            altitude = Altitude(
                header=messaging.create_header("base_link"),
                amsl=altitude_amsl,
                local=local,  # TODO: home altitude ok?
                relative=-local,  # TODO: check sign
                terrain=altitude_amsl - ground_track_altitude.amsl,
                bottom_clearance=altitude_amsl - ground_track_altitude.amsl,
            )
            return altitude

        return _altitude(
            self.geopose,
            self.egm96_height,
            self.ground_track_altitude,
            self._altitude_local,
        )

    @property
    # @ROS.publish(
    #    messaging.ROS_TOPIC_EGM96_HEIGHT,
    #    QoSPresetProfiles.SENSOR_DATA.value,
    # )
    def egm96_height(self) -> float:  # Optional[Float32]:
        """EGM96 geoid height at current location, or None if not available"""

        @narrow_types(self)
        def _egm96_height(geopose: GeoPoseStamped) -> float:
            return self._egm96.height(
                geopose.pose.position.latitude,
                geopose.pose.position.longitude,
            )

        # return Float32(data=_egm96_height(self.geopose))
        return _egm96_height(self.geopose)

    @narrow_types
    def _request_orthoimage_for_bounding_box(
        self,
        bounding_box: BoundingBox,
        size: Tuple[int, int],
        srs: str,
        format_: str,
        transparency: bool,
        layers: List[str],
        dem_layers: List[str],
        styles: List[str],
        dem_styles: List[str],
    ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        Sends GetMap request to GIS WMS for image and DEM layers and returns
        :attr:`.orthoimage_3d` attribute.

        Assumes zero raster as DEM if no DEM layer is available.

        TODO: Currently no support for separate arguments for imagery and height
        layers. Assumes height layer is available at same CRS as imagery layer.

        :param bounding_box: BoundingBox to request the orthoimage for
        :param size: Orthoimage resolution (height, width)
        :return: Orthophoto and dem tuple for bounding box
        """
        assert_len(styles, len(layers))
        assert_len(dem_styles, len(dem_layers))

        bbox = messaging.bounding_box_to_bbox(bounding_box)

        self.get_logger().info("Requesting new orthoimage")
        img: np.ndarray = self._get_map(
            layers, styles, srs, bbox, size, format_, transparency
        )
        if img is None:
            self.get_logger().error("Could not get orthoimage from GIS server")
            return None

        dem: Optional[np.ndarray] = None
        if len(dem_layers) > 0 and dem_layers[0]:
            self.get_logger().info("Requesting new DEM")
            dem = self._get_map(
                dem_layers,
                dem_styles,
                srs,
                bbox,
                size,
                format_,
                transparency,
                grayscale=True,
            )
            if dem is not None and dem.ndim == 2:
                dem = np.expand_dims(dem, axis=2)
        else:
            # Assume flat (:=zero) terrain if no DEM layer provided
            self.get_logger().debug(
                "No DEM layer provided, assuming flat (=zero) elevation model."
            )
            dem = np.zeros_like(img)

        # TODO: handle dem is None from _get_map call
        assert img is not None and dem is not None
        assert img.ndim == dem.ndim == 3
        return img, dem

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

        @narrow_types(self)
        def _orthoimage_overlap_is_too_low(
            new_bounding_box: BoundingBox,
            old_orthoimage: OrthoImage3D,
            min_map_overlap_update_threshold: float,
        ) -> bool:
            bbox = messaging.bounding_box_to_bbox(new_bounding_box)
            bbox_previous = messaging.bounding_box_to_bbox(old_orthoimage.bbox)
            bbox1, bbox2 = box(*bbox), box(*bbox_previous)
            ratio1 = bbox1.intersection(bbox2).area / bbox1.area
            ratio2 = bbox2.intersection(bbox1).area / bbox2.area
            ratio = min(ratio1, ratio2)
            if ratio > min_map_overlap_update_threshold:
                return False

            return True

        # Access self._orthoimage_3d directly to prevent recursion since this is
        # used as @cache_if predicate for self.orthoimage_3d
        # Cast None to False (assume bounding box not yet available)
        return (
            bool(
                _orthoimage_overlap_is_too_low(
                    self.bounding_box,
                    self._orthoimage_3d,
                    self.min_map_overlap_update_threshold,
                )
            )
            or not self._orthoimage_3d
        )

    @property
    def _altitude_local(self) -> Optional[float]:
        """Returns z coordinate from :class:`sensor_msgs.msg.PoseStamped` message
        or None if not available"""

        @narrow_types(self)
        def _altitude_local(pose_stamped: PoseStamped):
            return pose_stamped.pose.position.z

        return _altitude_local(self.pose)

    @property
    def bounding_box(self) -> Optional[BoundingBox]:
        """Geographical bounding box of area to retrieve a map for, or None if not
        available
        """

        @narrow_types(self)
        def _bounding_box(
            latlon: LatLon,
            camera_info: CameraInfo,
            altitude: Altitude,
            max_map_radius: int,
        ):
            # TODO: log messages for what's going on, or split into multiple methods
            # bounding_box = self.bounding_box
            map_radius = get_dynamic_map_radius(
                camera_info, max_map_radius, altitude.terrain
            )
            return self._bounding_box_with_padding_for_latlon(
                latlon.lat, latlon.lon, map_radius
            )

        latlon = self._principal_point_on_ground_plane
        bounding_box = _bounding_box(
            latlon, self.camera_info, self.altitude, self.max_map_radius
        )

        if bounding_box is None:
            geopose = self.geopose
            if geopose is None:
                if self.home_position is not None:
                    geopoint = self.home_position.geo
                else:
                    return None
            else:
                geopoint = geopose.pose.position

            bounding_box = self._bounding_box_with_padding_for_latlon(
                geopoint.latitude, geopoint.longitude
            )

        return bounding_box

    @property
    @ROS.publish(
        messaging.ROS_TOPIC_ORTHOIMAGE,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    @cache_if(_should_request_orthoimage)
    def orthoimage_3d(self) -> Optional[OrthoImage3D]:
        """Outgoing orthoimage and elevation raster pair"""
        # TODO: if FOV projection is large, this BoundingBox can be too large
        # and the WMS server will choke? Should get a BoundingBox for center
        # of this BoundingBox instead, with limited width and height (in meters)
        bounding_box = self.bounding_box
        map = self._request_orthoimage_for_bounding_box(
            bounding_box,
            self._orthoimage_size,
            self.wms_srs,
            self.wms_format,
            self.wms_transparency,
            self.wms_layers,
            self.wms_dem_layers,
            self.wms_styles,
            self.wms_dem_styles,
        )
        if map is not None:
            img, dem = map
            # TODO: use np.frombuffer, not CvBridge
            return OrthoImage3D(
                bbox=bounding_box,
                img=self._cv_bridge.cv2_to_imgmsg(img, encoding="passthrough"),
                dem=self._cv_bridge.cv2_to_imgmsg(dem, encoding="passthrough")
                # dem=self._cv_bridge.cv2_to_imgmsg(dem, encoding="mono8")
            )  # TODO: need to use mono16? 255 meters max?
        else:
            return None

    @property
    def _ground_track_altitude_amsl(self) -> Optional[float]:
        """
        Altitude of vehicle ground track above mean sea level (AMSL) in
        meters, or None if not available
        """

        @narrow_types(self)
        def _ground_track_altitude_amsl(navsatfix: NavSatFix):
            return self._ground_track_altitude_amsl_at_latlon(
                navsatfix.latitude, navsatfix.longitude
            )

        # Use self.latlon instead of self.geopose (geopose needs vehicle ellipsoide
        # altitude, which possibly creates a circular dependency), should not need
        # vehicle altitude to get ground track altitude at vehicle global position
        return _ground_track_altitude_amsl(self.nav_sat_fix)

    @property
    def _ground_track_altitude_ellipsoid(self) -> Optional[float]:
        """
        Vehicle ground track altitude above WGS84 ellipsoid in meters,
        or None if not available
        """

        @narrow_types(self)
        def _ground_track_altitude_ellipsoid(
            ground_track_altitude_amsl: float, navsatfix: NavSatFix
        ):
            return ground_track_altitude_amsl + self._egm96.height(
                navsatfix.latitude,
                navsatfix.longitude,
            )

        return _ground_track_altitude_ellipsoid(
            self._ground_track_altitude_amsl, self.nav_sat_fix
        )

    @property
    def _home_altitude_amsl(self) -> Optional[float]:
        """
        Home position altitude above mean sea level (AMLS) in meters, or None
        if not available
        """

        @narrow_types(self)
        def _home_altitude_amsl(home_position: HomePosition):
            home_geopoint = home_position.geo
            return home_geopoint.altitude - self._egm96.height(
                home_geopoint.latitude,
                home_geopoint.longitude,
            )

        return _home_altitude_amsl(self.home_position)

    @property
    def _ground_track_altitude_at_home_amsl(self) -> Optional[float]:
        """
        Vehicle ground track altitude above mean sea level (AMSL) at home
        position, or None if not available

        # TODO: check if this definition is correct
        """
        # Home defined as local origin, and local origin defined as part of
        # ground track, so this should be just home altitude AMSL
        return self._home_altitude_amsl

    def _get_map(
        self, layers, styles, srs, bbox, size, format_, transparency, grayscale=False
    ) -> Optional[np.ndarray]:
        """Sends WMS GetMap request and returns numpy array"""
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

    def _get_feature_info(
        self, layers, styles, srs, bbox, size, format_, xy
    ) -> Optional[float]:
        @narrow_types(self)
        def _extract_dem_height_from_gml(gml_bytes: bytes) -> Optional[float]:
            """Extracts DEM height value from GML returned from WMS endpoint"""
            # Split the binary string into lines using '\n\n' as a separator
            binary_lines = gml_bytes.split(b"\n\n")

            # Decode each binary line into a regular string
            decoded_lines = [line.decode("utf-8") for line in binary_lines]

            # Join the decoded lines back into a single string
            gml_data = "\n".join(decoded_lines)

            root = ET.fromstring(gml_data)

            # Find the "value_0" element and extract its value
            # TODO: handle not finding the value
            height_element = root.find(".//value_0")
            if height_element is not None and height_element.text is not None:
                return float(height_element.text)
            else:
                self.get_logger().error(
                    'DEM height ("value_0" element) not found in GetFeatureInfo '
                    "response"
                )
                return None

        """Sends WMS GetFeatureInfo request and returns float value"""
        self.get_logger().info(
            f"Sending GetFeatureInfo request for xy: {bbox}, xy {xy}, layers: {layers}."
        )
        try:
            # Do not handle possible requests library related exceptions here
            # (see class docstring)
            feature_info: IO = self._wms_client.getfeatureinfo(
                layers=layers,
                styles=styles,
                srs=srs,
                bbox=bbox,
                size=size,
                format=format_,
                query_layers=layers,
                info_format="application/vnd.ogc.gml",
                xy=xy,
            )

        except ServiceException as se:
            self.get_logger().error(
                f"GetFeatureInfo request ran into an unexpected exception: {se}"
            )
            return None
        finally:
            self.get_logger().debug("GetFeatureInfo request complete.")

        return _extract_dem_height_from_gml(feature_info.read())

    def _dem_height_meters_at_latlon(
        self, latitude: float, longitude: float
    ) -> Optional[float]:
        """
        Raw digital elevation model (DEM) height in meters at geographic coordinates
        (WGS 84 latitude and longitude) from cached DEM if available, or None
        if not available

        .. note::
            The vertical datum for the USGS DEM is NAVD 88. Other DEMs may
            have other vertical data, this method is agnostic to the vertical
            datum but assumes the units are meters. **Within the flight mission
            area the vertical datum of the DEM is assumed to be flat**.

        :param latitude: Query latitude coordinate
        :param longitude: Query longitude coordinate
        :return: Raw altitude in DEM coordinate frame and units
        """

        @narrow_types(self)
        def _dem_height_meters_at_latlon(
            latitude: float, longitude: float, orthoimage: OrthoImage3D
        ) -> Optional[float]:
            def _is_latlon_inside_bounding_box(
                latitude: float, longitude: float, bounding_box: BoundingBox
            ) -> bool:
                return (
                    bounding_box.min_pt.latitude
                    <= latitude
                    <= bounding_box.max_pt.latitude
                    and bounding_box.min_pt.longitude
                    <= longitude
                    <= bounding_box.max_pt.longitude
                )

            dem = self._cv_bridge.imgmsg_to_cv2(
                orthoimage.dem, desired_encoding="passthrough"
            )  # TODO: 255 meters max? use np.uint16

            if _is_latlon_inside_bounding_box(latitude, longitude, orthoimage.bbox):
                # Normalized coordinates of the GeoPoint in the BoundingBox
                bounding_box = orthoimage.bbox
                u = (longitude - bounding_box.min_pt.longitude) / (
                    bounding_box.max_pt.longitude - bounding_box.min_pt.longitude
                )
                v = (latitude - bounding_box.min_pt.latitude) / (
                    bounding_box.max_pt.latitude - bounding_box.min_pt.latitude
                )

                # Pixel coordinates in the DEM image
                x_pixel = int(u * (orthoimage.dem.width - 1))
                y_pixel = int(v * (orthoimage.dem.height - 1))

                # DEM height at the pixel coordinates
                dem_altitude = dem[y_pixel, x_pixel]
                return float(dem_altitude)
            else:
                return None

        # Use cached orthoimage if available, do not try to recompute to avoid
        # circular dependencies
        dem_height_meters_at_latlon = _dem_height_meters_at_latlon(
            latitude, longitude, self._orthoimage_3d
        )
        if dem_height_meters_at_latlon is None:
            dem_height_meters_at_latlon = self._dem_height_meters_at_latlon_wms(
                latitude,
                longitude,
                self.wms_srs,
                self.wms_format,
                self.wms_dem_layers,
                self.wms_dem_styles,
            )

        return dem_height_meters_at_latlon

    def _ground_track_altitude_amsl_at_latlon(
        self,
        latitude: float,
        longitude: float,
    ) -> Optional[float]:
        """
        Vehicle ground track altitude in meters above mean sea level (AMSL)
        according to digital elevation model (DEM) if available, or None if
        not available.

        :param latitude: Vehicle global position latitude
        :param latitude: Vehicle global position longitude
        :return: Vehicle ground track altitude AMSL in meters at vehicle global
            position
        """

        @narrow_types(self)
        def _ground_track_altitude_amsl_at_latlon(
            dem_height_meters_at_latlon: float,
            dem_height_meters_at_local_origin: float,
            home_position: HomePosition,
        ):
            local_origin_position = home_position.geo
            local_origin_altitude_amsl = (
                local_origin_position.altitude
                - self._egm96.height(
                    local_origin_position.latitude, local_origin_position.longitude
                )
            )
            dem_elevation_relative_meters = (
                dem_height_meters_at_latlon - dem_height_meters_at_local_origin
            )
            elevation_amsl = dem_elevation_relative_meters + local_origin_altitude_amsl
            return float(elevation_amsl)

        dem_height_meters_at_latlon = self._dem_height_meters_at_latlon(
            latitude, longitude
        )
        return _ground_track_altitude_amsl_at_latlon(
            dem_height_meters_at_latlon,
            self.dem_height_meters_at_local_origin,
            self.home_position,
        )

    @staticmethod
    def _quaternion_to_yaw_degrees(q):
        yaw = np.arctan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        return np.degrees(yaw)

    @property
    def _principal_point_on_ground_plane(self) -> LatLon:
        @narrow_types(self)
        def _principal_point_on_ground_plane(
            geopose: GeoPoseStamped, altitude: Altitude, gimbal_quaternion: Quaternion
        ) -> LatLon:
            # Your off-nadir angle and camera yaw
            off_nadir_angle_deg = messaging.off_nadir_angle(gimbal_quaternion)

            camera_yaw = self._quaternion_to_yaw_degrees(gimbal_quaternion)

            # Convert the off-nadir angle to a distance on the ground.
            # This step assumes a simple spherical Earth model, not taking
            # into account ellipsoid shape or terrain altitude.
            ground_distance = altitude.terrain / np.cos(
                np.radians(off_nadir_angle_deg)
            )  # in meters

            # Use pygeodesy to calculate new position
            current_pos = LatLon(
                geopose.pose.position.latitude, geopose.pose.position.longitude
            )

            # Get the latitude and longitude of the principal point projected
            # on the ground
            principal_point_ground = current_pos.destination(
                ground_distance, camera_yaw
            )

            return principal_point_ground

        return _principal_point_on_ground_plane(
            self.geopose, self.altitude, self.gimbal_quaternion
        )

    @property
    @ROS.max_delay_ms(_DELAY_SLOW_MS)
    @ROS.subscribe(
        "/mavros/home_position/home",
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def home_position(self) -> Optional[HomePosition]:
        """Home position, or None if unknown or too old"""

    def gimbal_device_attitude_status_cb(self, msg: GimbalDeviceAttitudeStatus) -> None:
        """Callback for :class:`mavros_msgs.msg.GimbalDeviceAttitudeStatus` message

        Publishes gimbal :class:`.geometry_msgs.msg.Quaternion` because the
        content of that message is affected by this update.

        :param msg: :class:`mavros_msgs.msg.GimbalDeviceAttitudeStatus` message
            from MAVROS
        """
        self.gimbal_quaternion

    @property
    # @ROS.max_delay_ms(_DELAY_FAST_MS)  # TODO re-enable
    @ROS.subscribe(
        "/mavros/gimbal_control/device/attitude_status",
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=gimbal_device_attitude_status_cb,
    )
    def gimbal_device_attitude_status(self) -> Optional[GimbalDeviceAttitudeStatus]:
        """Gimbal attitude, or None if unknown or too old"""

    @staticmethod
    def _quaternion_multiply(q1, q2):
        w1, x1, y1, z1 = q1.w, q1.x, q1.y, q1.z
        w2, x2, y2, z2 = q2.w, q2.x, q2.y, q2.z

        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

        return Quaternion(w=w, x=x, y=y, z=z)

    @property
    @ROS.publish(
        messaging.ROS_TOPIC_GIMBAL_QUATERNION, QoSPresetProfiles.SENSOR_DATA.value
    )
    def gimbal_quaternion(self) -> Optional[Quaternion]:
        """Gimbal orientation as :class:`geometry_msgs.msg.Quaternion` message
        or None if not available

        .. note::
            Current implementation assumes camera is facing directly down from
            vehicle body if GimbalDeviceAttitudeStatus (MAVLink gimbal protocol v2)
            is not available. SHould therefore not be used for estimating
            vehicle pose from gimbal pose.
        """

        def _apply_vehicle_yaw(vehicle_q, gimbal_q):
            yaw_deg = self._quaternion_to_yaw_degrees(vehicle_q)
            yaw_rad = np.radians(yaw_deg)

            # Create a new quaternion with only yaw rotation
            yaw_q = Quaternion(
                w=np.cos(yaw_rad / 2), x=0.0, y=0.0, z=np.sin(yaw_rad / 2)
            )

            # Apply the vehicle yaw rotation to the gimbal quaternion
            gimbal_yaw_q = self._quaternion_multiply(yaw_q, gimbal_q)

            return gimbal_yaw_q

        # TODO check frame (e.g. base_link_frd/vehicle body in PX4 SITL simulation)
        @narrow_types(self)
        def _gimbal_quaternion(
            geopose: GeoPoseStamped,
        ):
            """Gimbal orientation quaternion in North-East-Down (NED) frame.

            Origin is defined as gimbal (camera) pointing directly down nadir
            with top of image facing north. This definition should avoid gimbal
            lock for realistic use cases where the camera is used mainly to look
            down at the terrain under the vehicle instead of e.g. at the horizon.
            """
            if self.gimbal_device_attitude_status is None:
                # Identity quaternion: assume nadir-facing camera if no
                # information received from autopilot bridge
                q = Quaternion(
                    x=0.0,
                    y=0.0,
                    z=0.0,
                    w=1.0,
                )
            else:
                # PX4 over MAVROS gives GimbalDeviceAttitudeStatus in vehicle
                # body FRD frame with origin pointing forward along vehicle nose.
                # To re-center origin to nadir need to adjust pitch by -90 degrees.
                q_pitch_minus_90_deg = messaging.as_ros_quaternion(
                    np.array([np.cos(-np.pi / 4), 0, np.sin(-np.pi / 4), 0])
                )
                q = self._quaternion_multiply(
                    self.gimbal_device_attitude_status.q, q_pitch_minus_90_deg
                )

            assert q is not None

            compound_q = _apply_vehicle_yaw(geopose.pose.orientation, q)

            return compound_q

        return _gimbal_quaternion(self.geopose)
