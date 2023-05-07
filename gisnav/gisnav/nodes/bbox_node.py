"""A node that publishes bounding box of field of view projected to ground
from vehicle approximate location"""
from typing import Optional

import numpy as np
from geographic_msgs.msg import BoundingBox, GeoPoint, GeoPointStamped, GeoPoseStamped
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import Altitude
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image

from gisnav.assertions import enforce_types

from ..data import (
    Attitude,
    BBox,
    CameraData,
    ContextualMapData,
    DataValueError,
    Dim,
    FixedCamera,
    ImageData,
    ImagePair,
    Img,
    MapData,
    Pose,
)
from ..geo import GeoPt as GeoPt
from ..geo import GeoSquare, get_dynamic_map_radius
from . import messaging
from .base.camera_subscriber_node import CameraSubscriberNode


class BBoxNode(CameraSubscriberNode):
    """A node that publishes bounding box that matches camera field of view
    projected to ground

    This is the suggested bounding box for the next map update, or "approximate
    location" of what the camera is seeing.

    .. note::
        :class:`.KeypointPoseEstimator` takes elevation data into account if
        DEMs are provided, but the projection logic here currently assumes
        planar terrain regardless of whether a DEM is provided via :class:`.MapNode`.
    """

    ROS_D_GIMBAL_PROJECTION = True
    """Default flag to enable map updates based on expected center of field of
    view (FOV) projected onto ground

    When this flag is enabled, map rasters are retrieved for the expected camera
    FOV instead of the expected position of the vehicle, which increases the
    chances that the FOV is fully contained in the map raster. This again increases
    the chances of getting a good pose estimate.
    """

    ROS_D_MAX_MAP_RADIUS = 400
    """Default maximum map radius for used maps"""

    ROS_PARAM_DEFAULTS = [
        ("gimbal_projection", ROS_D_GIMBAL_PROJECTION, False),
        ("max_map_radius", ROS_D_MAX_MAP_RADIUS, False),
    ]
    """List containing ROS parameter name, default value and read_only flag tuples"""

    def __init__(self, name: str):
        """Class initializer

        :param name: Node name
        """
        super().__init__(name)

        # Subscribers
        self._vehicle_geopose = None
        self._vehicle_geopose_sub = self.create_subscription(
            GeoPoseStamped,
            messaging.ROS_TOPIC_VEHICLE_GEOPOSE,
            self._vehicle_geopose_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

        self._vehicle_altitude = None
        self._vehicle_altitude_sub = self.create_subscription(
            Altitude,
            messaging.ROS_TOPIC_VEHICLE_ALTITUDE,
            self._vehicle_altitude_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

        self._terrain_geopoint = None
        self._terrain_geopoint_sub = self.create_subscription(
            GeoPointStamped,
            messaging.ROS_TOPIC_TERRAIN_GEOPOINT,
            self._terrain_geopoint_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self._home_geopoint = None
        self._home_geopoint_sub = self.create_subscription(
            GeoPointStamped,
            messaging.ROS_TOPIC_HOME_GEOPOINT,
            self._home_geopoint_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

        self._terrain_altitude = None
        self._terrain_altitude_sub = self.create_subscription(
            Altitude,
            messaging.ROS_TOPIC_TERRAIN_ALTITUDE,
            self._terrain_altitude_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

        self._gimbal_quaternion = None
        self._gimbal_quaternion_sub = self.create_subscription(
            Quaternion,
            messaging.ROS_TOPIC_GIMBAL_QUATERNION,
            self._gimbal_quaternion_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

        # Publishers
        self._bounding_box_pub = self.create_publisher(
            BoundingBox,
            messaging.ROS_TOPIC_BOUNDING_BOX,
            QoSPresetProfiles.SENSOR_DATA.value,
        )

    @property
    def _is_gimbal_projection_enabled(self) -> bool:
        """True if map rasters should be retrieved for projected field of view
        instead of vehicle position

        If this is set to false, map rasters are retrieved for the vehicle's
        global position instead. This is typically fine as long as the camera
        is not aimed too far in to the horizon and has a relatively wide field
        of view. For best results, this should be enabled to ensure the field
        of view is fully contained within the area of the retrieved map raster.

        .. note::
            If you know your camera will be nadir-facing, disabling
            ``gimbal_projection`` may improve performance
        """
        gimbal_projection_flag = (
            self.get_parameter("gimbal_projection").get_parameter_value().bool_value
        )
        if type(gimbal_projection_flag) is bool:
            return gimbal_projection_flag
        else:
            # Default behavior (safer)
            self.get_logger().warn(
                f"Could not read gimbal projection flag: "
                f"{gimbal_projection_flag}. Assume False."
            )
            return False

    def _vehicle_geopose_callback(self, msg: GeoPoseStamped) -> None:
        """Receives vehicle :class:`geographic_msgs.msg.GeoPoseStamped` message"""
        self._vehicle_geopose = msg

        # Vehicle GeoPose changes affect bbox estimate -> call _publish here
        self._publish()

    def _vehicle_altitude_callback(self, msg: Altitude) -> None:
        """Receives vehicle :class:`mavros_msgs.msg.Altitude` message"""
        self._vehicle_altitude = msg

        # No need to call _publish here:
        # Vehicle altitude changes do affect bbox estimate but _publish is
        # already called in GeoPose callback Altitude message is only
        # subscribed to because it provides altitude AGL and AMSL unlike GeoPose
        # self._publish()

    def _terrain_geopoint_callback(self, msg: GeoPointStamped) -> None:
        """Receives terrain :class:`geographic_msgs.msg.GeoPointStamped` message"""
        self._terrain_geopoint = msg

    def _home_geopoint_callback(self, msg) -> None:
        """Receives home :class:`geographic_msgs.msg.GeoPointStamped` message"""
        self._home_geopoint = msg

    def _terrain_altitude_callback(self, msg: Altitude) -> None:
        """Receives terrain :class:`mavros_msgs.msg.Altitude` message"""
        self._terrain_altitude = msg

    def _gimbal_quaternion_callback(self, msg: Quaternion) -> None:
        """Receives gimbal :class:`geometry_msgs.msg.Quaternion` message"""
        self._gimbal_quaternion = msg

    def image_callback(self, msg: Image) -> None:
        """Receives :class:`sensor_msgs.msg.Image` message"""
        # Do nothing - implementation enforced by parent class

    # region Mock Image Pair
    def _mock_image_pair(self, xy: GeoPt) -> Optional[ImagePair]:
        """Creates mock :class:`.ImagePair` for guessing projected FOV needed
        for map requests, or None if not available

        The mock image pair will be paired with a pose guess to compute the
        expected field of view. The expected field of view is used to request
        a new map that overlaps with what the camera is looking at.

        .. seealso:
            :meth:`._mock_map_data` and :meth:`._mock_image_data`

        :param xy: Vehicle position
        :return: Mock image pair that can be paired with a pose guess to generate
            a FOV guess, or None if not available
        """

        @enforce_types(self.get_logger().warn, "Cannot generate mock image pair")
        def _mock_image_pair(image_data: ImageData, map_data: MapData):
            contextual_map_data = ContextualMapData(
                rotation=0, crop=image_data.image.dim, map_data=map_data, mock_data=True
            )
            return ImagePair(image_data, contextual_map_data)

        return _mock_image_pair(self._mock_image_data(), self._mock_map_data(xy))

    # TODO: make property
    def _mock_image_data(self) -> Optional[ImageData]:
        """Creates mock :class:`.ImageData` for guessing projected FOV for map
        requests, or None if not available

        .. seealso:
            :meth:`._mock_map_data` and :meth:`._mock_image_pair`
        """

        @enforce_types(self.get_logger().warn, "Cannot generate mock image data")
        def _mock_image_data(img_dim: Dim, camera_data: CameraData, usec: int):
            image_data = ImageData(
                image=Img(np.zeros(img_dim)),
                frame_id="mock_image_data",
                timestamp=usec,
                camera_data=camera_data,
            )
            return image_data

        return _mock_image_data(self.img_dim, self.camera_data, self.usec)

    def _mock_map_data(self, xy: GeoPt) -> Optional[MapData]:
        """Creates mock :class:`.MapData` for guessing projected FOV needed for
        map requests, or None if not available

        The mock image pair will be paired with a pose guess to compute the
        expected field of view. The expected field of view is used to request a
        new map that overlaps with what the camera is looking at.

        .. seealso:
            :meth:`._mock_image_pair` and :meth:`._mock_image_data`

        :param xy: Vehicle position
        :return: Mock map data with mock images but with real expected bbox,
            or None if not available
        """

        @enforce_types(self.get_logger().warn, "Cannot generate mock map data")
        def _mock_map_data(
            camera_data: CameraData,
            map_size_with_padding: Dim,
            vehicle_altitude: Altitude,
        ):
            altitude_agl = vehicle_altitude.terrain
            if altitude_agl < 0:
                self.get_logger().warn(
                    f"Altitude AGL {altitude_agl} was negative, skipping mock map data."
                )
                return

            # Scaling factor of image pixels := terrain_altitude
            scaling = (map_size_with_padding[0] / 2) / camera_data.fx
            radius = scaling * altitude_agl

            bbox = GeoSquare(xy, radius)
            zeros = np.zeros(map_size_with_padding)
            map_data = MapData(
                bbox=BBox(*bbox.bounds), image=Img(zeros), elevation=Img(zeros)
            )
            return map_data

        return _mock_map_data(
            self.camera_data, self.map_size_with_padding, self._vehicle_altitude
        )

    # endregion

    def _guess_fov_center(self, xy: GeoPt) -> Optional[GeoPt]:
        """Guesses WGS84 coordinates of camera field of view (FOV) projected on
        ground from given origin

        :param xy: Camera position
        :return: Center of the projected FOV, or None if not available
        """

        @enforce_types(self.get_logger().warn, "Cannot project FOV center")
        def _fov_center(
            mock_image_pair: ImagePair,
            gimbal_quaternion: Quaternion,
            camera_data: CameraData,
            terrain_altitude: Altitude,
            terrain_geopoint: GeoPointStamped,
            home_geopoint: GeoPointStamped,
            usec: int,
        ):
            gimbal_attitude = Attitude(q=messaging.as_np_quaternion(gimbal_quaternion))
            gimbal_attitude = (
                gimbal_attitude.to_esd()
            )  # Need coordinates in image frame, not NED

            assert gimbal_attitude is not None

            translation = -gimbal_attitude.r @ np.array(
                [camera_data.cx, camera_data.cy, -camera_data.fx]
            )
            try:
                pose = Pose(gimbal_attitude.r, translation.reshape((3, 1)))
            except DataValueError as e:
                raise Exception(
                    f"Pose input values: {gimbal_attitude.r}, "
                    f"{translation} were invalid: {e}."
                ) from e

            try:
                mock_fixed_camera = FixedCamera(
                    pose=pose,
                    image_pair=mock_image_pair,
                    terrain_altitude_amsl=terrain_altitude.amsl,
                    terrain_altitude_ellipsoid=terrain_geopoint.position.altitude,
                    home_position=home_geopoint.position,
                    timestamp=usec,
                )
            except DataValueError:
                self.get_logger().warn(
                    "Could not create a valid mock projection of FOV."
                )
                return None

            return mock_fixed_camera.fov.fov.to_crs("epsg:4326").center

        try:
            return _fov_center(
                self._mock_image_pair(xy),
                self._gimbal_quaternion,
                self.camera_data,
                self._terrain_altitude,
                self._terrain_geopoint,
                self._home_geopoint,
                self.usec,
            )
        except DataValueError as e:
            self.get_logger().warn(str(e))
            return None

    def _publish(self) -> None:
        """Publishes :class:`geographic_msgs.msg.BoundingBox` message

        If :py:attr:`._is_gimbal_projection_enabled` is True, the center of the
        projected camera field of view is used instead of the vehicle position
        to ensure the field of view is best contained in the new map raster.
        """

        @enforce_types(self.get_logger().warn, "Cannot publish bounding box")
        def _publish_bbox(
            vehicle_geopose: GeoPoseStamped,
            vehicle_altitude: Altitude,
            camera_data: CameraData,
            gimbal_projection_enabled: bool,
        ):
            geopt = messaging.geopoint_to_geopt(vehicle_geopose.pose.position)

            if gimbal_projection_enabled:
                projected_center = self._guess_fov_center(geopt)
                if projected_center is None:
                    self.get_logger().warn(
                        "Could not project field of view center. Using vehicle "
                        "position for map center instead."
                    )
            else:
                projected_center = None

            map_update_altitude_agl = vehicle_altitude.terrain
            if map_update_altitude_agl is None:
                self.get_logger().warn(
                    "Cannot determine altitude AGL, skipping map update."
                )
                return None
            if map_update_altitude_agl <= 0:
                self.get_logger().warn(
                    f"Map update altitude {map_update_altitude_agl} should be > 0, "
                    f"skipping map update."
                )
                return None
            max_map_radius = (
                self.get_parameter("max_map_radius").get_parameter_value().integer_value
            )
            map_radius = get_dynamic_map_radius(
                camera_data, max_map_radius, map_update_altitude_agl
            )
            map_candidate = GeoSquare(
                projected_center if projected_center is not None else geopt, map_radius
            )

            bbox = BBox(*map_candidate.bounds)
            bbox = BoundingBox(
                min_pt=GeoPoint(
                    latitude=bbox.bottom, longitude=bbox.left, altitude=np.nan
                ),
                max_pt=GeoPoint(
                    latitude=bbox.top, longitude=bbox.right, altitude=np.nan
                ),
            )
            self._bounding_box_pub.publish(bbox)

        _publish_bbox(
            self._vehicle_geopose,
            self._vehicle_altitude,
            self.camera_data,
            self._is_gimbal_projection_enabled,
        )
