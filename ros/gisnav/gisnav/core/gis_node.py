"""This module contains :class:`.GISNode`, a ROS node that requests
orthoimagery from the GIS and publishes it to ROS.
"""
from copy import deepcopy
from typing import IO, Final, List, Optional, Tuple

import cv2
import numpy as np
import requests
from cv_bridge import CvBridge
from geographic_msgs.msg import BoundingBox, GeoPoint
from gisnav_msgs.msg import OrthoImage  # type: ignore[attr-defined]
from owslib.util import ServiceException
from owslib.wms import WebMapService
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.timer import Timer
from sensor_msgs.msg import CameraInfo
from shapely.geometry import box
from std_msgs.msg import String

from .. import _transformations as tf_
from .._decorators import ROS, cache_if, narrow_types
from ..constants import (
    BBOX_NODE_NAME,
    ROS_NAMESPACE,
    ROS_TOPIC_CAMERA_INFO,
    ROS_TOPIC_RELATIVE_FOV_BOUNDING_BOX,
    ROS_TOPIC_RELATIVE_ORTHOIMAGE,
    FrameID,
)


class GISNode(Node):
    """Publishes the orthoimage, DEM, and their CRS as a single, atomic ROS message.

    > [!WARNING] OWSLib exception handling
    > ``OWSLib``, *as of version 0.25.0*, uses the Python ``requests`` library under
    > the hood but does not document the various exceptions it raises that are passed
    > through by ``OWSLib`` as part of its public API. The private method that calls the
    > ``OWSLib`` method that implements the GetMap call is therefore expected to raise > `errors and exceptions <https://requests.readthedocs.io/en/latest/user/quickstart/#errors-and-exceptions>`_
    > specific to the ``requests`` library.
    >
    > These errors and exceptions are not handled by the :class:`.GISNode` to avoid a
    > direct dependency on ``requests``. They are therefore handled as unexpected
    > errors.
    """  # noqa: E501

    ROS_D_URL = "http://127.0.0.1:80/wms"
    """Default value for :attr:`.wms_url`

    When :ref:`deploying Docker Compose services <Deploy with Docker Compose>` the
    Docker DNS host name of the MapServer container ``gisnav-mapserver-1`` should be
    used in the URL. This should already be configured in the `default launch parameter
    file <https://github.com/hmakelin/gisnav/blob/master/gisnav/launch/params/gis_node.
    yaml>`_ which overrides this default value.

    Alternatively, if the service is on a different network, use the Docker host URL,
    or the URL of the reverse proxy.
    """

    ROS_D_VERSION = "1.3.0"
    """Default value for :attr:`.wms_version`"""

    ROS_D_TIMEOUT = 10
    """Default value for :attr:`.wms_timeout`"""

    ROS_D_PUBLISH_RATE = 1.0
    """Default value for :attr:`.publish_rate`"""

    ROS_D_WMS_POLL_RATE = 0.1
    """Default value for :attr:`.wms_poll_rate`"""

    ROS_D_LAYERS = ["imagery"]
    """Default value for :attr:`.wms_layers`

    > [!TIP]
    > The combined layers should cover the flight area of the vehicle at high
    > resolution. Typically this list would have just one layer for high resolution
    > aerial or satellite imagery.
    """

    ROS_D_DEM_LAYERS = ["dem"]
    """Default value for :attr:`.wms_dem_layers`

    > [!TIP]
    > This is an optional elevation layer that makes the pose estimation more accurate
    > especially when flying at low altitude. It should be a grayscale raster with
    > pixel values corresponding meters relative to vertical datum. Vertical datum can
    > be whatever system is used (e.g. USGS DEM uses NAVD 88), although it is assumed
    > to be flat across the flight mission area.
    """

    ROS_D_STYLES = [""]
    """Default value for :attr:`.wms_styles`

    > [!TIP]
    > Must be same length as :py:attr:`.ROS_D_LAYERS`. Use empty strings for server
    > default styles.
    """

    ROS_D_DEM_STYLES = [""]
    """Default value for :attr:`.wms_dem_styles`

    > [!TIP]
    > Must be same length as :py:attr:`.ROS_D_DEM_LAYERS`. Use empty strings for server
    > default styles.
    """

    ROS_D_SRS = "EPSG:4326"
    """Default value for :attr:`.wms_srs`"""

    ROS_D_IMAGE_FORMAT = "image/jpeg"
    """Default value for :attr:`.wms_format`"""

    ROS_D_IMAGE_TRANSPARENCY = False
    """Default value for :attr:`.wms_transparency`

    > [!NOTE]
    > This parameter is not supported by jpeg format
    """

    ROS_D_MAP_OVERLAP_UPDATE_THRESHOLD = 0.85
    """The default ROS parameter for the minimum required overlap ratio between the
    bounding boxes of the current and new orthoimages. If the overlap is below this
    threshold, a new map request is triggered.
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
        self.bounding_box
        self.camera_info

        # TODO: use throttling in publish decorator, remove timer
        publish_rate = self.publish_rate
        assert publish_rate is not None
        self._publish_timer = self._create_publish_timer(publish_rate)

        # TODO: refactor out CvBridge and use np.frombuffer instead
        self._cv_bridge = CvBridge()

        wms_poll_rate = self.wms_poll_rate
        assert wms_poll_rate is not None
        self._wms_client = None  # TODO add type hint if possible
        self._connect_wms_timer: Optional[Timer] = self._create_connect_wms_timer(
            wms_poll_rate
        )

        self.old_bounding_box: Optional[BoundingBox] = None

    @property
    @ROS.parameter(ROS_D_URL, descriptor=_ROS_PARAM_DESCRIPTOR_READ_ONLY)
    def wms_url(self) -> Optional[str]:
        """ROS WMS client endpoint URL parameter"""

    @property
    @ROS.parameter(ROS_D_VERSION, descriptor=_ROS_PARAM_DESCRIPTOR_READ_ONLY)
    def wms_version(self) -> Optional[str]:
        """ROS WMS protocol version parameter"""

    @property
    @ROS.parameter(ROS_D_TIMEOUT, descriptor=_ROS_PARAM_DESCRIPTOR_READ_ONLY)
    def wms_timeout(self) -> Optional[int]:
        """ROS WMS request timeout [seconds] parameter"""

    @property
    @ROS.parameter(ROS_D_LAYERS)
    def wms_layers(self) -> Optional[List[str]]:
        """ROS parameter for WMS request layers for orthoimagery GetMap requests"""

    @property
    @ROS.parameter(ROS_D_DEM_LAYERS)
    def wms_dem_layers(self) -> Optional[List[str]]:
        """ROS WMS parameter for WMS request layers for DEM GetMap requests"""

    @property
    @ROS.parameter(ROS_D_STYLES)
    def wms_styles(self) -> Optional[List[str]]:
        """ROS parameter for WMS request styles for orthoimegry GetMap requests"""

    @property
    @ROS.parameter(ROS_D_DEM_STYLES)
    def wms_dem_styles(self) -> Optional[List[str]]:
        """ROS parameter for WMS request styles for :DEM GetMap requests"""

    @property
    @ROS.parameter(ROS_D_SRS)
    def wms_srs(self) -> Optional[str]:
        """ROS parameter for WMS request CRS for all GetMap requests"""

    @property
    @ROS.parameter(ROS_D_IMAGE_TRANSPARENCY)
    def wms_transparency(self) -> Optional[bool]:
        """ROS parameter for WMS request transparency for all GetMap requests"""

    @property
    @ROS.parameter(ROS_D_IMAGE_FORMAT)
    def wms_format(self) -> Optional[str]:
        """ROS parameter for WMS request format for all GetMap requests"""

    @property
    @ROS.parameter(ROS_D_WMS_POLL_RATE, descriptor=_ROS_PARAM_DESCRIPTOR_READ_ONLY)
    def wms_poll_rate(self) -> Optional[float]:
        """ROS parameter for WMS connection status poll rate in Hz"""

    @property
    @ROS.parameter(ROS_D_MAP_OVERLAP_UPDATE_THRESHOLD)
    def min_map_overlap_update_threshold(self) -> Optional[float]:
        """ROS parameter for the minimum required overlap ratio between the bounding
        boxes of the current and new orthoimages. If the overlap is below this
        threshold, a new map request is triggered.
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
        timer = self.create_timer(1 / publish_rate, self._publish)
        return timer

    @narrow_types
    def _create_connect_wms_timer(self, poll_rate: float) -> Timer:
        """Returns a timer that reconnects the WMS client when needed

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
        """:attr:`.orthoimage` publish and map update timer"""
        return self.__publish_timer

    @_publish_timer.setter
    def _publish_timer(self, value: Timer) -> None:
        self.__publish_timer = value

    def _publish(self):
        """Publishes :attr:`.orthoimage`"""
        self.orthoimage

    def _try_wms_client_instantiation(self) -> None:
        """Attempts to instantiate :attr:`._wms_client`

        Destroys :attr:`._connect_wms_timer` if instantiation is successful
        """

        @narrow_types(self)
        def _connect_wms(url: str, version: str, timeout: int, poll_rate: float):
            try:
                assert self._wms_client is None
                self.get_logger().info(f"Connecting to WMS endpoint at {url}...")
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
        self, latitude: float, longitude: float, padding: float = 600.0
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

    @property
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_FOV_BOUNDING_BOX.replace("~", BBOX_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def bounding_box(self) -> Optional[BoundingBox]:
        """Subscribed bounding box of the camera's ground-projected FOV, or None if
        unknown
        """

    @property
    @ROS.subscribe(
        ROS_TOPIC_CAMERA_INFO,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def camera_info(self) -> Optional[CameraInfo]:
        """Subscribed camera info for determining appropriate :attr:`.orthoimage`
        resolution, or None if unknown
        """

    @property
    def _orthoimage_size(self) -> Optional[Tuple[int, int]]:
        """Padded map size tuple (height, width) or None if the information
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
        """Sends GetMap request to GIS WMS for image and DEM layers and returns
        :attr:`.orthoimage` attribute.

        Assumes zero raster as DEM if no DEM layer is available.

        TODO: Currently no support for separate arguments for imagery and elevation
         layers. Assumes elevation layer is available at same CRS as imagery layer.

        :param bounding_box: BoundingBox to request the orthoimage for
        :param size: Orthoimage resolution (height, width)
        :return: Orthophoto and dem tuple for bounding box
        """
        assert len(styles) == len(layers)
        assert len(dem_styles) == len(dem_layers)

        bbox = tf_.bounding_box_to_bbox(bounding_box)

        self.get_logger().debug("Requesting new orthoimage")
        img: np.ndarray = self._get_map(
            layers, styles, srs, bbox, size, format_, transparency
        )
        if img is None:
            self.get_logger().error("Could not get orthoimage from GIS server")
            return None

        dem: Optional[np.ndarray] = None
        if len(dem_layers) > 0 and dem_layers[0]:
            self.get_logger().debug("Requesting new DEM")
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
            old_bounding_box: BoundingBox,
            min_map_overlap_update_threshold: float,
        ) -> bool:
            bbox = tf_.bounding_box_to_bbox(new_bounding_box)
            bbox_previous = tf_.bounding_box_to_bbox(old_bounding_box)
            bbox1, bbox2 = box(*bbox), box(*bbox_previous)
            ratio1 = bbox1.intersection(bbox2).area / bbox1.area
            ratio2 = bbox2.intersection(bbox1).area / bbox2.area
            ratio = min(ratio1, ratio2)
            if ratio > min_map_overlap_update_threshold:
                return False

            return True

        return self.old_bounding_box is None or _orthoimage_overlap_is_too_low(
            self.bounding_box,
            self.old_bounding_box,
            self.min_map_overlap_update_threshold,
        )

    @property
    @ROS.publish(
        ROS_TOPIC_RELATIVE_ORTHOIMAGE,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    @cache_if(_should_request_orthoimage)
    def orthoimage(self) -> Optional[OrthoImage]:
        """Outgoing orthoimage and DEM raster"""
        bounding_box = deepcopy(self.bounding_box)
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

            assert (
                dem.shape[2] == 1
            ), f"DEM shape was {dem.shape}, expected 1 channel only."
            assert (
                img.shape[2] == 3
            ), f"Image shape was {img.shape}, expected 3 channels."
            assert dem.dtype == np.uint8  # todo get 16 bit dems?

            # Convert image to grayscale (color not needed)
            dem_msg = self._cv_bridge.cv2_to_imgmsg(dem, encoding="mono8")
            img_msg = self._cv_bridge.cv2_to_imgmsg(img, encoding="passthrough")
            orthoimage_msg = OrthoImage(image=img_msg, dem=dem_msg)

            # Set old bounding box
            self.old_bounding_box = bounding_box

            height, width = img.shape[:2]
            M = self._calculate_affine_transformation_matrix(
                height, width, bounding_box
            )
            if M is not None:
                proj_str: FrameID = tf_.affine_to_proj(M)

            # Assign timestamp
            orthoimage_msg.image.header.stamp = self.get_clock().now().to_msg()
            orthoimage_msg.dem.header.stamp = orthoimage_msg.image.header.stamp

            orthoimage_msg.crs = String(data=proj_str)

            return orthoimage_msg
        else:
            return None

    @narrow_types
    def _calculate_affine_transformation_matrix(
        self, height: int, width: int, bbox: BoundingBox
    ) -> Optional[np.ndarray]:
        """Calculates the 3x3 affine transformation matrix that transforms orthoimage
        frame pixel coordinates to WGS84 lon, lat coordinates.

        :param height: Height in pixels of the :term:`reference` image
        :param width: Width in pixels of the :term:`reference` image (most
            likely same as height)
        :param bbox: :term:`WGS 84` :term:`bounding box` of the reference image
        :return: 3x3 affine transformation matrix
        """

        def _boundingbox_to_geo_coords(
            bounding_box: BoundingBox,
        ) -> List[Tuple[float, float]]:
            """Extracts the geo coordinates from a ROS
            geographic_msgs/BoundingBox and returns them as a list of tuples.

            Returns corners in order: top-left, bottom-left, bottom-right,
            top-right.

            :param bbox: (geographic_msgs/BoundingBox): The bounding box.
            :return: The geo coordinates as a list of (longitude, latitude)
                tuples.
            """
            min_lon = bounding_box.min_pt.longitude
            min_lat = bounding_box.min_pt.latitude
            max_lon = bounding_box.max_pt.longitude
            max_lat = bounding_box.max_pt.latitude

            return [
                (min_lon, max_lat),
                (min_lon, min_lat),
                (max_lon, min_lat),
                (max_lon, max_lat),
            ]

        def _haversine_distance(lat1, lon1, lat2, lon2) -> float:
            R = 6371000  # Radius of the Earth in meters
            lat1_rad, lon1_rad = np.radians(lat1), np.radians(lon1)
            lat2_rad, lon2_rad = np.radians(lat2), np.radians(lon2)

            delta_lat = lat2_rad - lat1_rad
            delta_lon = lon2_rad - lon1_rad

            a = (
                np.sin(delta_lat / 2) ** 2
                + np.cos(lat1_rad) * np.cos(lat2_rad) * np.sin(delta_lon / 2) ** 2
            )
            c = 2 * np.arctan2(np.sqrt(a), np.sqrt(1 - a))

            return R * c

        def _bounding_box_perimeter_meters(bounding_box: BoundingBox) -> float:
            """Returns the length of the bounding box perimeter in meters"""
            width_meters = _haversine_distance(
                bounding_box.min_pt.latitude,
                bounding_box.min_pt.longitude,
                bounding_box.min_pt.latitude,
                bounding_box.max_pt.longitude,
            )
            height_meters = _haversine_distance(
                bounding_box.min_pt.latitude,
                bounding_box.min_pt.longitude,
                bounding_box.max_pt.latitude,
                bounding_box.min_pt.longitude,
            )
            return 2 * width_meters + 2 * height_meters

        pixel_coords = self._create_src_corners(height, width)
        geo_coords = _boundingbox_to_geo_coords(bbox)

        pixel_coords = np.float32(pixel_coords).squeeze()
        geo_coords = np.float32(geo_coords).squeeze()
        M = cv2.getPerspectiveTransform(pixel_coords, geo_coords)

        # Insert z dimensions and scale
        aff = np.eye(4)
        aff[:2, :2] = M[:2, :2]
        aff[:2, 3] = M[:2, 2]

        # TODO: extract scaling from M - do not calculate it separately
        bounding_box_perimeter_native = 2 * height + 2 * width
        bounding_box_perimeter_meters = _bounding_box_perimeter_meters(bbox)

        # The reference raster image plane defined by the bounding box is "
        # East-South-Down", while WGS 84 is ENU, so we invert the z coordinate sign
        aff[2, 2] = -bounding_box_perimeter_meters / bounding_box_perimeter_native

        return aff

    def _get_map(
        self, layers, styles, srs, bbox, size, format_, transparency, grayscale=False
    ) -> Optional[np.ndarray]:
        """Sends WMS GetMap request and returns response raster"""
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
                f"GetMap request failed likely because of a connection error: {se}"
            )
            return None
        except requests.exceptions.ConnectionError as ce:  # noqa: F841
            # Expected error if no connection
            self.get_logger().error(
                f"GetMap request failed because of a connection error: {ce}"
            )
            return None
        except Exception as e:
            # TODO: handle other exception types
            self.get_logger().error(
                f"GetMap request for image ran into an unexpected exception: {e}"
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
            return img

        return _read_img(img, grayscale)

    @staticmethod
    def _create_src_corners(h: int, w: int) -> np.ndarray:
        """Helper function that returns image corner pixel coordinates in a
        numpy array.

        Returns corners in following order: top-left, bottom-left, bottom-right,
        top-right.

        :param h: Source image height
        :param w: Source image width
        :return: Source image corner pixel coordinates
        """
        assert (
            h > 0 and w > 0
        ), f"Height {h} and width {w} are both expected to be positive."
        return np.float32([[0, 0], [w - 1, 0], [w - 1, h - 1], [0, h - 1]]).reshape(
            -1, 1, 2
        )
