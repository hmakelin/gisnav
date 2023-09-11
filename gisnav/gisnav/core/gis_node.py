"""This module contains :class:`.GISNode`, a :term:`ROS` node for retrieving and
publishing geographic information and images.

:class:`.GISNode` manages geographic information for the system, including
downloading and storing the :term:`orthophoto` and optionally :term:`DEM`
:term:`raster`. These rasters are retrieved from an :term:`onboard` :term:`WMS`
based on the projected location of the :term:`camera` field of view.

:class:`.GISNode` publishes :class:`.OrthoImage3D` messages, which contain
high-resolution orthophotos and an optional DEM. It also publishes vehicle
:term:`geopose` and :term:`altitude`, and :term:`ground track` :term:`geopose` and
:term:`elevation`.

.. mermaid::
    :caption: :class:`.GISNode` computational graph

    graph LR

        subgraph Camera
            camera_info[camera/camera_info]
        end

        subgraph MAVROS
            pose[mavros/local_position/pose]
            global[mavros/global_position/global]
            home[mavros/home_position/home]
            attitude[mavros/gimbal_control/device/attitude_status]
        end

        subgraph GISNode
            geopose[gisnav/vehicle/geopose]
            altitude[gisnav/vehicle/altitude]
            geopoint_track[gisnav/ground_track/geopoint]
            altitude_track[gisnav/ground_track/altitude]
            orthoimage[gisnav/orthoimage]
        end

        subgraph CVNode
            geopose_estimate[gisnav/vehicle/geopose/estimate]
            altitude_estimate[gisnav/vehicle/altitude/estimate]
        end

        pose -->|geometry_msgs/Pose| GISNode
        global -->|sensor_msgs/NavSatFix| GISNode
        home -->|mavros_msgs/HomePosition| GISNode
        attitude -->|mavros_msgs/GimbalDeviceAttitudeStatus| GISNode
        camera_info -->|sensor_msgs/CameraInfo| GISNode

        geopose -->|geographic_msgs/GeoPose| CVNode
        altitude -->|mavros_msgs/Altitude| CVNode
        orthoimage -->|gisnav_msgs/OrthoImage3D| CVNode
        altitude_track -->|mavros_msgs/Altitude| CVNode
        geopoint_track -->|geographic_msgs/GeoPoint| CVNode

"""
import xml.etree.ElementTree as ET
from typing import IO, Final, List, Optional, Tuple

import cv2
import numpy as np
import requests
from cv_bridge import CvBridge
from geographic_msgs.msg import BoundingBox, GeoPoint, GeoPose, GeoPoseStamped
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

from .. import messaging
from .._assertions import assert_len, assert_type
from .._decorators import ROS, cache_if, narrow_types
from ..static_configuration import (
    ROS_TOPIC_RELATIVE_CAMERA_GEOPOSE,
    ROS_TOPIC_RELATIVE_CAMERA_QUATERNION,
    ROS_TOPIC_RELATIVE_GROUND_TRACK_ELEVATION,
    ROS_TOPIC_RELATIVE_GROUND_TRACK_GEOPOSE,
    ROS_TOPIC_RELATIVE_ORTHOIMAGE,
    ROS_TOPIC_RELATIVE_VEHICLE_ALTITUDE,
    ROS_TOPIC_RELATIVE_VEHICLE_GEOPOSE,
)


class GISNode(Node):
    """Publishes :class:`.OrthoImage3D` of approximate location to a topic

    This node generates both :term:`vehicle` :term:`altitude` and
    :term:`ground track` :term:`elevation` :term:`messages`, resulting in a
    complex interplay of class properties. The workflows to publish specific
    messages are triggered by subscription callbacks.

    GISNode implements three data flows representing its main outputs, which are
    defined as its :term:`ROS` publishers. The data flows use shared inputs and
    intermediates and are therefore all included in the same node. The data
    flows are 1. the orthoimage data flow, 2. vehicle data flow, 3. and the
    ground track data flow. Diagrams are provided for each below.

    .. note::
        Diamonds represent conditionals, often reverting to cached values. Boxed
        items can be ROS messages or private computed properties used as
        intermediates.

    **1. Orthoimage data flow**: The diagram below shows the dependency between
    the properties of GISNode when generating the :attr:`orthoimage` message:

    .. mermaid::
        :caption: Orthoimage data flow diagram

        graph TB
            subgraph callbacks
                CB[_nav_sat_fix_cb]
            end

            subgraph subscribe
                CI[camera_info]
                HP[home_position]
                GDAS[gimbal_device_attitude_status]
            end

            subgraph publish
                Z[vehicle_altitude]
                V[vehicle_geopose]
                L[orthoimage]
            end

            subgraph OWSLib
                K[_get_map]
            end

            Q --> X[camera_quaternion]
            X --> V
            Q --> Z
            Q --> V
            P{_should_request_orthoimage} --> L
            CB --> P
            L --> H[_request_orthoimage_for_bounding_box]
            L --> OS[_orthoimage_size]
            H --> K
            L ----> O[_bounding_box]
            O --> HP
            O --> Q[_principal_point_on_ground_plane]
            O --> V
            OS ----> CI
            O --> CI
            X --> GDAS

    .. note::
        :attr:`.vehicle_geopose` is a dependency to the :attr:`._bounding_box`
        both directly and via :attr:`._principal_point_on_ground_plane`. If the
        principal point projection on ground is not available, vehicle
        :term:`geopose` is used for an approximate position guess directly.

    **2. Vehicle data flow**: The diagram below shows the dependency between
    the properties of GISNode when generating the :attr:`vehicle_geopose` and
    :attr:`vehicle_altitude` messages:

    .. mermaid::
        :caption: Vehicle data flow diagram

        graph TB

            subgraph _callbacks
                NSFCB[_nav_sat_fix_cb]
            end _callbacks

            subgraph subscribe
                I[nav_sat_fix]
                T[vehicle_pose]
            end subscribe

            subgraph publish
                E[ground_track_elevation]
                V[vehicle_geopose]
                Z[vehicle_altitude]
            end publish

            NSFCB --> Z
            NSFCB --> V
            V --> I
            V --> T
            Z --> EGM[_ground_track_geoid_separation_egm96]
            Z --> E
            Z --> V
            Z --> AL[_altitude_local]
            AL --> T
            EGM[_ground_track_geoid_separation_egm96] --> EGM2[_egm96]
            EGM --> I

    **3. Ground track data flow**: The diagram below shows he dependency between
    the properties of GISNode when generating the :attr:`ground_track_geopose` and
    :attr:`ground_track_elevation` messages:

    .. mermaid::
        :caption: Ground track data flow diagram

        graph TB

            subgraph callbacks
                NSFCB[_nav_sat_fix_cb]
            end callbacks

            subgraph subscribe
                I[nav_sat_fix]
                HP[home_position]
            end subscribe

            subgraph publish
                E[ground_track_elevation]
                Y[ground_track_geopose]
            end publish

            subgraph OWSLib
                GFI[_get_feature_info]
            end OWSLib

            NSFCB[_nav_sat_fix_cb] --> E
            E --> F[_ground_track_elevation_at_home_amsl]
            E --> G[_ground_track_elevation_amsl]
            F --> N[_home_elevation_amsl]
            G --> J[_ground_track_elevation_amsl_at_latlon]
            J --> D[_dem_elevation_meters_at_home]
            D --> R{_should_update_dem_elevation_at_home}
            R --> C[_dem_elevation_meters_at_latlon_wms]
            C --> GFI
            D --> HP
            J --> I[nav_sat_fix]
            Y --> M[_ground_track_elevation_ellipsoid]
            NSFCB --> Y
            EGM2 --> I
            EGM2[_ground_track_geoid_separation_egm96] --> EGM[_egm96]
            M --> G[_ground_track_elevation_amsl]
            M --> EGM2
            N --> HP[home_position]

    .. warning::
        ``OWSLib`` *as of version 0.25.0* uses the Python ``requests`` library
        under the hood but does not seem to document the various exceptions it
        raises that are passed through by ``OWSLib`` as part of its public API.
        The :meth:`.get_map` method is therefore expected to raise `errors and exceptions
        <https://requests.readthedocs.io/en/latest/user/quickstart/#errors-and-exceptions>`_
        that are specific to the ``requests`` library.

        These errors and exceptions are not handled by the :class:`.GISNode`
        to avoid a direct dependency to ``requests``. They are therefore
        handled as unexpected errors.
    """  # noqa: E501

    ROS_D_URL = "http://127.0.0.1:80/wms"
    """Default WMS URL"""

    ROS_D_VERSION = "1.3.0"
    """Default WMS version"""

    ROS_D_TIMEOUT = 10
    """Default WMS GetMap request timeout in seconds"""

    ROS_D_PUBLISH_RATE = 1.0
    """Default publish rate for :class:`.OrthoImage3D` messages in Hz"""

    ROS_D_WMS_POLL_RATE = 0.1
    """Default WMS connection status poll rate in Hz"""

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
        raster with pixel values corresponding meters relative to vertical datum.
        Vertical datum can be whatever system is used (e.g. USGS DEM uses NAVD 88),
        although it is assumed to be flat across the flight mission area.
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

    .. todo::
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

    _ROS_PARAM_DESCRIPTOR_READ_ONLY: Final = ParameterDescriptor(read_only=True)
    """A read only ROS parameter descriptor"""

    def __init__(self, *args, **kwargs):
        """Class initializer

        :param args: Positional arguments to parent :class:`.Node` constructor
        :param kwargs: Keyword arguments to parent :class:`.Node` constructor
        """
        super().__init__(*args, **kwargs)

        # Calling these decorated properties the first time will setup
        # subscriptions to the appropriate ROS topics
        # self.bounding_box
        self.vehicle_pose
        self.camera_info
        self.nav_sat_fix
        self.home_position
        self.gimbal_device_attitude_status

        # TODO: use throttling in publish decorator, remove timer
        publish_rate = self.publish_rate
        assert publish_rate is not None
        self._publish_timer = self._create_publish_timer(publish_rate)

        # TODO: make configurable / use shared folder home path instead
        self._egm96 = GeoidPGM("/usr/share/GeographicLib/geoids/egm96-5.pgm", kind=-3)
        self._transformer = Transformer.from_crs("EPSG:4326", "EPSG:3857")

        # TODO: refactor out CvBridge and use np.frombuffer instead
        self._cv_bridge = CvBridge()

        wms_poll_rate = self.wms_poll_rate
        assert wms_poll_rate is not None
        self._wms_client = None  # TODO add type hint if possible
        self._connect_wms_timer: Optional[Timer] = self._create_connect_wms_timer(
            wms_poll_rate
        )

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
    @ROS.parameter(ROS_D_WMS_POLL_RATE, descriptor=_ROS_PARAM_DESCRIPTOR_READ_ONLY)
    def wms_poll_rate(self) -> Optional[float]:
        """:term:`WMS` connection status poll rate in Hz"""

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
    def publish_rate(self) -> Optional[float]:
        """Publish rate in Hz for the :attr:`.orthoimage` :term:`message`"""

    @narrow_types
    def _create_publish_timer(self, publish_rate: float) -> Timer:
        """
        Returns a timer to publish :attr:`.orthoimage` to ROS

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

    @narrow_types
    def _create_connect_wms_timer(self, poll_rate: float) -> Timer:
        """Returns a timer that reconnects :term:`WMS` client when needed

        :param poll_rate: WMS connection status poll rate for the timer (in Hz)
        :return: The :class:`.Timer` instance
        """
        if poll_rate <= 0:
            error_msg = (
                f"WMS connection status poll rate must be positive ("
                f"{poll_rate} Hz provided)."
            )
            self.get_logger().error(error_msg)
            raise ValueError(error_msg)
        timer = self.create_timer(1 / poll_rate, self._try_wms_client_instantiation)
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
        Publish :attr:`.orthoimage` (:attr:`.ground_track_elevation` and
        :attr:`.terrain_geopoint_stamped` are also published but that
        publish is triggered by callbacks since the messages are smaller and
        can be published more often)
        """
        self.orthoimage

    def _try_wms_client_instantiation(self) -> None:
        """Attempts to instantiate :attr:`._wms_client`

        Destroys :attr:`._connect_wms_timer` if instantiation is successful
        """

        @narrow_types(self)
        def _connect_wms(url: str, version: str, timeout: int, poll_rate: float):
            try:
                assert self._wms_client is None
                self.get_logger().info("Connecting to WMS endpoint...")
                self._wms_client = WebMapService(url, version=version, timeout=timeout)
                self.get_logger().info("WMS client connection established.")

                # We have the WMS client instance - we can now destroy the timer
                assert self._connect_wms_timer is not None
                self._connect_wms_timer.destroy()
            except requests.exceptions.ConnectionError as _:  # noqa: F841
                # Expected error if no connection
                self.get_logger().error(
                    f"Could not instantiate WMS client due to connection error, "
                    f"trying again in {1 / poll_rate} seconds..."
                )
                assert self._wms_client is None
            except Exception as e:
                # TODO: handle other exception types
                self.get_logger().error(
                    f"Could not instantiate WMS client due to unexpected exception "
                    f"type ({type(e)}), trying again in {1 / poll_rate} seconds..."
                )
                assert self._wms_client is None

        if self._wms_client is None:
            _connect_wms(
                self.wms_url, self.wms_version, self.wms_timeout, self.wms_poll_rate
            )

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

    def _should_update_dem_elevation_meters_at_home(self):
        # TODO: Update if local frame changes
        return (
            not hasattr(self, "__dem_elevation_meters_at_home")
            or getattr(self, "__dem_elevation_meters_at_home") is None
        )

    @narrow_types
    def _dem_elevation_meters_at_latlon_wms(
        self,
        lat: float,
        lon: float,
        srs: str,
        format_: str,
        dem_layers: List[str],
        dem_styles: List[str],
    ) -> Optional[float]:
        """:term:`DEM` :term:`elevation` in meters at :term:`WGS 84` coordinates,
        or None if not available

        Uses WMS :term:`GetFeatureInfo` to get the value from the onboard
        :term:`GIS`.
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
    @cache_if(_should_update_dem_elevation_meters_at_home)
    def _dem_elevation_meters_at_home(self) -> Optional[float]:
        """:term:`DEM` :term:`elevation` in meters at :term:`home`, or None if
        not available

        DEM elevation vlaues are expected to be requested for (1) the
        :term:`vehicle` :term:`global position`, and (2) home global position.
        DEM elevation for the home global position is needed to determine the
        :term:`AMSL` elevation of the DEM, which may be expressed in some other
        vertical datum. Since the home global position generally is not inside
        :attr:`.orthoimage`, a separate :term:`GetFeatureInfo` request retrieving
        DEM elevation for home global position is implemented here.

        .. note::
            Assumes :term:`home` is also the local frame :term:`origin`.
        """
        home_position = self.home_position
        if home_position is not None:
            return self._dem_elevation_meters_at_latlon_wms(
                home_position.geo.latitude,
                home_position.geo.longitude,
                self.wms_srs,
                self.wms_format,
                self.wms_dem_layers,
                self.wms_dem_styles,
            )
        else:
            self.get_logger().error(
                "Home position is None, cannot get bounding box with padding."
            )
            return None

    def _nav_sat_fix_cb(self, msg: NavSatFix) -> None:
        """Callback for the :term:`global position` message from the
        :term:`navigation filter`

        Publishes :term:`vehicle` and :term:`ground track` :term:`geopose` and
        :term:`altitude` or :term:`elevation` because they are affected by the
        updated global position.

        :param msg: :class:`mavros_msgs.msg.NavSatFix` message from MAVROS
        """
        self.ground_track_elevation
        self.ground_track_geopose
        self.vehicle_altitude
        self.vehicle_geopose

        # TODO: temporarily assuming static camera so publishing gimbal quat here
        # self.camera_quaternion

    @property
    @ROS.max_delay_ms(messaging.DELAY_DEFAULT_MS)
    @ROS.subscribe(
        "/mavros/global_position/global",
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=_nav_sat_fix_cb,
    )
    def nav_sat_fix(self) -> Optional[NavSatFix]:
        """Vehicle GPS fix, or None if unknown or too old"""

    @property
    @ROS.publish(
        ROS_TOPIC_RELATIVE_VEHICLE_GEOPOSE, QoSPresetProfiles.SENSOR_DATA.value
    )
    def vehicle_geopose(self) -> Optional[GeoPoseStamped]:
        """Published :term:`vehicle` :term:`geopose`, or None if not available"""

        @narrow_types(self)
        def _vehicle_geopose(nav_sat_fix: NavSatFix, pose_stamped: PoseStamped):
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

        return _vehicle_geopose(self.nav_sat_fix, self.vehicle_pose)

    @property
    # @ROS.max_delay_ms(messaging.DELAY_FAST_MS)  # TODO:
    @ROS.subscribe(
        "/mavros/local_position/pose",
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def vehicle_pose(self) -> Optional[PoseStamped]:
        """Vehicle local position, or None if not available or too old"""

    @property
    # @ROS.max_delay_ms(messaging.DELAY_DEFAULT_MS) - camera info has no header (?)
    @ROS.subscribe(messaging.ROS_TOPIC_CAMERA_INFO, QoSPresetProfiles.SENSOR_DATA.value)
    def camera_info(self) -> Optional[CameraInfo]:
        """Camera info for determining appropriate :attr:`.orthoimage` resolution"""

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
        ROS_TOPIC_RELATIVE_GROUND_TRACK_ELEVATION,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def ground_track_elevation(self) -> Optional[Altitude]:
        """
        Published :term:`ground track` :term:`elevation`, or None if not available

        .. note::
            The term "elevation" is more appropriate for the ground track than
            "altitude". However, :class:`.mavros_msgs.msg.Altitude` is used to
            model ground track elevation the same way it is used to model
            :term:`vehicle` :term:`altitude`.
        """

        @narrow_types(self)
        def _ground_track_elevation(
            ground_track_elevation_amsl: float,
            ground_track_elevation_at_home_amsl: float,
        ):
            # Define local == -relative, and terrain == bottom_clearance
            return Altitude(
                header=messaging.create_header("base_link"),
                amsl=ground_track_elevation_amsl,
                local=ground_track_elevation_amsl - ground_track_elevation_at_home_amsl,
                relative=-(
                    ground_track_elevation_amsl - ground_track_elevation_at_home_amsl
                ),
                terrain=0.0,
                bottom_clearance=0.0,
            )

        return _ground_track_elevation(
            self._ground_track_elevation_amsl, self._ground_track_elevation_at_home_amsl
        )

    @property
    @ROS.publish(
        ROS_TOPIC_RELATIVE_GROUND_TRACK_GEOPOSE,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def ground_track_geopose(self) -> Optional[GeoPoseStamped]:
        """Published :term:`ground track` :term:`geopose`, or None if not available

        Complementary to the :attr:`ground_track_elevation`, including horizontal
        and vertical :term:`global position` in one (atomic) :term:`message`.

        .. note::
            The :term:`orientation` part of the :term:`geopose` contained in the
            path is not defined for the ground track and should be ignored.
        """

        @narrow_types(self)
        def _ground_track_geopose(
            geopose: GeoPoseStamped, ground_track_elevation_ellipsoid: float
        ):
            return GeoPoseStamped(
                header=messaging.create_header("base_link"),
                pose=GeoPose(
                    position=GeoPoint(
                        latitude=geopose.pose.position.latitude,
                        longitude=geopose.pose.position.longitude,
                        altitude=ground_track_elevation_ellipsoid,
                    ),
                ),
            )

        return _ground_track_geopose(
            self.vehicle_geopose, self._ground_track_elevation_ellipsoid
        )

    @property
    @ROS.publish(
        ROS_TOPIC_RELATIVE_VEHICLE_ALTITUDE, QoSPresetProfiles.SENSOR_DATA.value
    )
    def vehicle_altitude(self) -> Optional[Altitude]:
        """Published :term:`vehicle` :term:`altitude`, or None if not available"""

        @narrow_types(self)
        def _vehicle_altitude(
            geopose: GeoPoseStamped,
            egm96_height: float,  # Float32,
            ground_track_elevation: Altitude,
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
                terrain=altitude_amsl - ground_track_elevation.amsl,
                bottom_clearance=altitude_amsl - ground_track_elevation.amsl,
            )
            return altitude

        return _vehicle_altitude(
            self.vehicle_geopose,
            self._ground_track_geoid_separation_egm96,
            self.ground_track_elevation,
            self._altitude_local,
        )

    @property
    def _ground_track_geoid_separation_egm96(self) -> float:
        """EGM96 geoid separation in meters at current location, or None if not
        available

        Add this value to :term:`AMSL` :term:`altitude` to get :term:`ellipsoid`
        altitude. Subtract this value from ellipsoid altitude to get AMSL altitude.

        .. seealso::
            See the "Avoiding Pitfalls Related to Ellipsoid Height and Height
            Above Mean Sea Level" section in the `MAVROS Plugins wiki`_

        .. _MAVROS wiki: http://wiki.ros.org/mavros/Plugins
        """

        @narrow_types(self)
        def _ground_track_geoid_separation_egm96(navsatfix: NavSatFix) -> float:
            return self._egm96.height(
                navsatfix.latitude,
                navsatfix.longitude,
            )

        return _ground_track_geoid_separation_egm96(self.nav_sat_fix)

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
        :attr:`.orthoimage` attribute.

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

        # Access self._orthoimage directly to prevent recursion since this is
        # used as @cache_if predicate for self.orthoimage
        # Cast None to False (assume bounding box not yet available)
        return (
            bool(
                _orthoimage_overlap_is_too_low(
                    self._bounding_box,
                    self._orthoimage,
                    self.min_map_overlap_update_threshold,
                )
            )
            or not self._orthoimage
        )

    @property
    def _altitude_local(self) -> Optional[float]:
        """Returns z coordinate from :class:`sensor_msgs.msg.PoseStamped` message
        or None if not available"""

        @narrow_types(self)
        def _altitude_local(pose_stamped: PoseStamped):
            return pose_stamped.pose.position.z

        return _altitude_local(self.vehicle_pose)

    @property
    def _bounding_box(self) -> Optional[BoundingBox]:
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
            def get_dynamic_map_radius(
                camera_info: CameraInfo, max_map_radius: int, elevation: float
            ) -> float:
                """Returns map radius that adjusts for camera altitude to be used
                for new map requests"""
                hfov = 2 * np.arctan(camera_info.width / (2 * camera_info.k[0]))
                map_radius = 1.5 * hfov * elevation  # Arbitrary padding of 50%
                return min(map_radius, max_map_radius)

            assert all((camera_info, max_map_radius, altitude.terrain))
            map_radius = get_dynamic_map_radius(
                camera_info, max_map_radius, altitude.terrain
            )
            return self._bounding_box_with_padding_for_latlon(
                latlon.lat, latlon.lon, map_radius
            )

        latlon = self._principal_point_on_ground_plane
        bounding_box = _bounding_box(
            latlon, self.camera_info, self.vehicle_altitude, self.max_map_radius
        )

        if bounding_box is None:
            geopose = self.vehicle_geopose
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
        ROS_TOPIC_RELATIVE_ORTHOIMAGE,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    @cache_if(_should_request_orthoimage)
    def orthoimage(self) -> Optional[OrthoImage3D]:
        """Outgoing orthoimage and elevation raster pair"""
        # TODO: if FOV projection is large, this BoundingBox can be too large
        # and the WMS server will choke? Should get a BoundingBox for center
        # of this BoundingBox instead, with limited width and height (in meters)
        bounding_box = self._bounding_box
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
    def _ground_track_elevation_amsl(self) -> Optional[float]:
        """:term:`Ground track` :term:`elevation` in meters :term:`AMSL`, or
        None if not available
        """

        @narrow_types(self)
        def _ground_track_elevation_amsl(navsatfix: NavSatFix):
            return self._ground_track_elevation_amsl_at_latlon(
                navsatfix.latitude, navsatfix.longitude
            )

        # Use self.latlon instead of self.geopose (geopose needs vehicle ellipsoide
        # altitude, which possibly creates a circular dependency), should not need
        # vehicle altitude to get ground track elevation at vehicle global position
        return _ground_track_elevation_amsl(self.nav_sat_fix)

    @property
    def _ground_track_elevation_ellipsoid(self) -> Optional[float]:
        """:term:`Ground track` :term:`elevation` above WGS 84 ellipsoid in meters,
        or None if not available
        """

        @narrow_types(self)
        def _ground_track_elevation_ellipsoid(
            ground_track_elevation_amsl: float,
            ground_track_geoid_separation_egm96: float,
        ):
            return ground_track_elevation_amsl + ground_track_geoid_separation_egm96

        return _ground_track_elevation_ellipsoid(
            self._ground_track_elevation_amsl, self._ground_track_geoid_separation_egm96
        )

    @property
    def _home_elevation_amsl(self) -> Optional[float]:
        """:term:`Home` :term:`elevation` in meters :term:`AMSL`, or None if not
        available
        """

        @narrow_types(self)
        def _home_elevation_amsl(home_position: HomePosition):
            home_geopoint = home_position.geo
            return home_geopoint.altitude - self._egm96.height(
                home_geopoint.latitude,
                home_geopoint.longitude,
            )

        return _home_elevation_amsl(self.home_position)

    @property
    def _ground_track_elevation_at_home_amsl(self) -> Optional[float]:
        """:term:`Ground track` :term:`elevation` in meters:term:`AMSL` at :term:`home`,
        or None if not available
        """
        # Home defined as local origin, and local origin defined as part of
        # ground track, so this should be just home elevation AMSL
        return self._home_elevation_amsl

    def _get_map(
        self, layers, styles, srs, bbox, size, format_, transparency, grayscale=False
    ) -> Optional[np.ndarray]:
        """Sends WMS :term:`GetMap` request and returns response :term:`raster`"""
        if self._wms_client is None:
            self.get_logger().warning(
                "WMS client not instantiated. Skipping sending GetMap request."
            )
            return None

        self.get_logger().info(
            f"Sending GetMap request for bbox: {bbox}, layers: {layers}."
        )
        try:
            # Do not handle possible requests library related exceptions here
            # (see class docstring)
            assert self._wms_client is not None
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
        """Sends WMS :term:`GetFeatureInfo` request and returns response value"""

        @narrow_types(self)
        def _extract_dem_height_from_gml(gml_bytes: bytes) -> Optional[float]:
            """Extracts :term:`DEM` :term:`elevation` from returned :term:`GML`"""
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

        if self._wms_client is None:
            self.get_logger().warning(
                "WMS client not instantiated. Skipping sending GetFeatureInfo request."
            )
            return None

        self.get_logger().info(
            f"Sending GetFeatureInfo request for xy: {bbox}, xy {xy}, layers: {layers}."
        )
        try:
            # Do not handle possible requests library related exceptions here
            # (see class docstring)
            assert self._wms_client is not None
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

    def _dem_elevation_meters_at_latlon(
        self, latitude: float, longitude: float
    ) -> Optional[float]:
        """
        Raw :term:`DEM` :term:`elevation` in meters at :term:`WGS 84` coordinates
        from (1) cached DEM if available, or (2) via :term:`GetFeatureInfo`
        request if no earlier cached DEM, or (3) None if not available

        .. note::
            The vertical datum for the USGS DEM is NAVD 88. Other DEMs may
            have other vertical data, this method is agnostic to the vertical
            datum but assumes the units are meters. **Within the flight mission
            area the vertical datum of the DEM is assumed to be flat**.

        :param latitude: Query latitude coordinate
        :param longitude: Query longitude coordinate
        :return: Raw elevation in DEM coordinate frame and units (assumed meters)
        """

        @narrow_types(self)
        def _dem_elevation_meters_at_latlon(
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

                # DEM elevation in meters at the pixel coordinates
                dem_elevation = dem[y_pixel, x_pixel]
                return float(dem_elevation)
            else:
                return None

        # Use cached orthoimage if available, do not try to recompute to avoid
        # circular dependencies
        dem_height_meters_at_latlon = (
            _dem_elevation_meters_at_latlon(latitude, longitude, self._orthoimage)
            if hasattr(self, "_orthoimage")
            else None
        )
        if dem_height_meters_at_latlon is None:
            dem_height_meters_at_latlon = self._dem_elevation_meters_at_latlon_wms(
                latitude,
                longitude,
                self.wms_srs,
                self.wms_format,
                self.wms_dem_layers,
                self.wms_dem_styles,
            )

        return dem_height_meters_at_latlon

    def _ground_track_elevation_amsl_at_latlon(
        self,
        latitude: float,
        longitude: float,
    ) -> Optional[float]:
        """:term:`Ground track` :term:`elevation` in meters :term:`AMSL` according to
        :term:`DEM` if available, or None if not available

        :param latitude: Vehicle global position latitude
        :param latitude: Vehicle global position longitude
        :return: Vehicle ground track elevation AMSL in meters
        """

        @narrow_types(self)
        def _ground_track_elevation_amsl_at_latlon(
            dem_elevation_meters_at_latlon: float,
            dem_elevation_meters_at_home: float,
            home_position: HomePosition,
        ):
            home_elevation_amsl = home_position.geo.altitude - self._egm96.height(
                home_position.geo.latitude, home_position.geo.longitude
            )
            dem_elevation_relative_meters = (
                dem_elevation_meters_at_latlon - dem_elevation_meters_at_home
            )
            elevation_amsl = dem_elevation_relative_meters + home_elevation_amsl
            return float(elevation_amsl)

        dem_height_meters_at_latlon = self._dem_elevation_meters_at_latlon(
            latitude, longitude
        )
        return _ground_track_elevation_amsl_at_latlon(
            dem_height_meters_at_latlon,
            self._dem_elevation_meters_at_home,
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
        """Projects :term:`camera` principal point on ground plane

        .. note::
            Assumes ground is a flat plane, does not take :term:`DEM` into account
        """

        @narrow_types(self)
        def _principal_point_on_ground_plane(
            geopose: GeoPoseStamped,
            vehicle_altitude: Altitude,
            camera_quaternion: Quaternion,
        ) -> LatLon:
            # Your off-nadir angle and camera yaw
            off_nadir_angle_deg = messaging.off_nadir_angle(camera_quaternion)

            camera_yaw = self._quaternion_to_yaw_degrees(camera_quaternion)

            # Convert the off-nadir angle to a distance on the ground.
            # This step assumes a simple spherical Earth model, not taking
            # into account ellipsoid shape or ground track elevation.
            ground_distance = vehicle_altitude.terrain / np.cos(
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
            self.vehicle_geopose, self.vehicle_altitude, self.camera_quaternion
        )

    @property
    @ROS.max_delay_ms(messaging.DELAY_SLOW_MS)
    @ROS.subscribe(
        "/mavros/home_position/home",
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def home_position(self) -> Optional[HomePosition]:
        """:term:`Home` :term:`global position`, or None if unknown or too old"""

    def _gimbal_device_attitude_status_cb(
        self, msg: GimbalDeviceAttitudeStatus
    ) -> None:
        """Callback for :class:`mavros_msgs.msg.GimbalDeviceAttitudeStatus` message

        Publishes :term:`camera` :class:`.geometry_msgs.msg.Quaternion` because
        its content is affected by this update.

        :param msg: :class:`mavros_msgs.msg.GimbalDeviceAttitudeStatus` message
            from MAVROS
        """
        self.camera_quaternion

    @property
    # @ROS.max_delay_ms(messaging.DELAY_FAST_MS)  # TODO re-enable
    @ROS.subscribe(
        "/mavros/gimbal_control/device/attitude_status",
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=_gimbal_device_attitude_status_cb,
    )
    def gimbal_device_attitude_status(self) -> Optional[GimbalDeviceAttitudeStatus]:
        """:term:`Camera` :term:`FRD` :term:`orientation`, or None if not available
        or too old
        """

    @staticmethod
    def _quaternion_multiply(q1: Quaternion, q2: Quaternion) -> Quaternion:
        w1, x1, y1, z1 = q1.w, q1.x, q1.y, q1.z
        w2, x2, y2, z2 = q2.w, q2.x, q2.y, q2.z

        w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
        x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
        y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
        z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

        return Quaternion(w=w, x=x, y=y, z=z)

    @property
    @ROS.publish(
        ROS_TOPIC_RELATIVE_CAMERA_QUATERNION, QoSPresetProfiles.SENSOR_DATA.value
    )
    def camera_quaternion(self) -> Optional[Quaternion]:
        """:term:`Camera` :term:`orientation` or None if not available

        .. note::
            Current implementation assumes camera is :term:`nadir` facing
            if GimbalDeviceAttitudeStatus :term:`message` (:term:`MAVLink` gimbal
            protocol v2) is not available. Should therefore not be used for
            estimating :term:`vehicle` :term:`orientation`.
        """

        def _apply_vehicle_yaw(vehicle_q, camera_q):
            yaw_deg = self._quaternion_to_yaw_degrees(vehicle_q)
            yaw_rad = np.radians(yaw_deg)

            # Create a new quaternion with only yaw rotation
            yaw_q = Quaternion(
                w=np.cos(yaw_rad / 2), x=0.0, y=0.0, z=np.sin(yaw_rad / 2)
            )

            # Apply the vehicle yaw rotation to the camera (gimbal) quaternion
            gimbal_yaw_q = self._quaternion_multiply(yaw_q, camera_q)

            return gimbal_yaw_q

        # TODO check frame (e.g. base_link_frd/vehicle body in PX4 SITL simulation)
        @narrow_types(self)
        def _camera_quaternion(
            geopose: GeoPoseStamped,
        ):
            """:term:`Camera` :term:`orientation` quaternion in :term:`NED` frame

            Origin is defined as camera (gimbal) pointing directly down :term:`nadir`
            with top of image facing north. This definition should avoid gimbal
            lock for realistic use cases where the camera is used mainly to look
            down at the ground under the :term:`vehicle` instead of e.g. at the horizon.
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

        return _camera_quaternion(self.vehicle_geopose)

    @property
    @ROS.publish(ROS_TOPIC_RELATIVE_CAMERA_GEOPOSE, QoSPresetProfiles.SENSOR_DATA.value)
    def camera_geopose(self) -> Optional[Quaternion]:
        """:term:`Camera` :term:`geopose` or None if not available"""

        @narrow_types(self)
        def _camera_geopose(
            vehicle_geopose: GeoPoseStamped, camera_quaternion: Quaternion
        ):
            camera_geopose = vehicle_geopose
            camera_geopose.pose.orientation = camera_quaternion
            return camera_geopose

        return _camera_geopose(self.vehicle_geopose, self.camera_quaternion)
