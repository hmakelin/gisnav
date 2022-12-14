"""A node that publishes bounding box of field of view projected to ground from vehicle approximate location"""
from typing import Optional

import numpy as np
from scipy.spatial.transform import Rotation
from rclpy.qos import QoSPresetProfiles
from geographic_msgs.msg import BoundingBox, GeoPoint, GeoPointStamped, GeoPoseStamped
from mavros_msgs.msg import Altitude
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import Image

from . import messaging
from .base.camera_subscriber_node import _CameraSubscriberNode
from ..data import BBox, Pose, FixedCamera, Attitude, DataValueError, MapData, ImagePair, ImageData, Img, \
    ContextualMapData
from ..geo import GeoPt as GeoPt, GeoSquare, get_dynamic_map_radius


class BBoxNode(_CameraSubscriberNode):
    """A node that publishes bounding box of field of view projected to ground

    This is the suggested bounding box for the next map update
    """
    ROS_D_GIMBAL_PROJECTION = True
    """Default flag to enable map updates based on expected center of field of view (FOV) projected onto ground

    When this flag is enabled, map rasters are retrieved for the expected center of the camera FOV instead of the
    expected position of the vehicle, which increases the chances that the FOV is fully contained in the map raster.
    This again increases the chances of getting a good pose estimate.

    .. seealso::
        :py:attr:`.ROS_D_MISC_MAX_PITCH`
        :py:attr:`.ROS_D_MAP_UPDATE_MAX_PITCH`
    """

    ROS_D_MAX_MAP_RADIUS = 400
    """Default maximum map radius for used maps"""

    ROS_D_DEBUG_EXPORT_PROJECTION = ''  # 'projection.json'
    """Default filename for exporting GeoJSON containing projected field of view (FOV) and FOV center

    Set to '' to disable
    """

    _ROS_PARAM_DEFAULTS = [
        ('gimbal_projection', ROS_D_GIMBAL_PROJECTION),
        ('max_map_radius', ROS_D_MAX_MAP_RADIUS),
        ('export_projection', ROS_D_DEBUG_EXPORT_PROJECTION),
    ]
    """ROS parameters used by this node to declare"""

    def __init__(self, name: str):
        """Class initializer

        :param name: Node name
        """
        super().__init__(name, ros_param_defaults=self._ROS_PARAM_DEFAULTS)

        # Subscribers
        self._vehicle_geopose = None
        self._vehicle_geopose_sub = self.create_subscription(GeoPoseStamped,
                                                             messaging.ROS_TOPIC_VEHICLE_GEOPOSE,
                                                             self._vehicle_geopose_callback,
                                                             QoSPresetProfiles.SENSOR_DATA.value)

        self._vehicle_altitude = None
        self._vehicle_altitude_sub = self.create_subscription(Altitude,
                                                              messaging.ROS_TOPIC_VEHICLE_ALTITUDE,
                                                              self._vehicle_altitude_callback,
                                                              QoSPresetProfiles.SENSOR_DATA.value)

        self._terrain_geopoint = None
        self._terrain_geopoint_sub = self.create_subscription(GeoPointStamped,
                                                              messaging.ROS_TOPIC_TERRAIN_GEOPOINT,
                                                              self._terrain_geopoint_callback,
                                                              QoSPresetProfiles.SENSOR_DATA.value)
        self._home_geopoint = None
        self._home_geopoint_sub = self.create_subscription(GeoPointStamped,
                                                           messaging.ROS_TOPIC_HOME_GEOPOINT,
                                                           self._home_geopoint_callback,
                                                           QoSPresetProfiles.SENSOR_DATA.value)

        self._terrain_altitude = None
        self._terrain_altitude_sub = self.create_subscription(Altitude,
                                                              messaging.ROS_TOPIC_TERRAIN_ALTITUDE,
                                                              self._terrain_altitude_callback,
                                                              QoSPresetProfiles.SENSOR_DATA.value)

        self._gimbal_quaternion = None
        self._gimbal_quaternion_sub = self.create_subscription(Quaternion,
                                                               messaging.ROS_TOPIC_GIMBAL_QUATERNION,
                                                               self._gimbal_quaternion_callback,
                                                               QoSPresetProfiles.SENSOR_DATA.value)

        # Publishers
        self._bounding_box_pub = self.create_publisher(BoundingBox,
                                                       messaging.ROS_TOPIC_BOUNDING_BOX,
                                                       QoSPresetProfiles.SENSOR_DATA.value)

    @property
    def _is_gimbal_projection_enabled(self) -> bool:
        """True if map rasters should be retrieved for projected field of view instead of vehicle position

        If this is set to false, map rasters are retrieved for the vehicle's global position instead. This is typically
        fine as long as the camera is not aimed too far in to the horizon and has a relatively wide field of view. For
        best results, this should be on to ensure the field of view is fully contained within the area of the retrieved
        map raster.

        .. note::
            If you know your camera will be nadir-facing, disabling ``map_update.gimbal_projection`` may improve
            performance by a small amount.
        """
        gimbal_projection_flag = self.get_parameter('gimbal_projection').get_parameter_value().bool_value
        if type(gimbal_projection_flag) is bool:
            return gimbal_projection_flag
        else:
            # Default behavior (safer)
            self.get_logger().warn(f'Could not read gimbal projection flag: {gimbal_projection_flag}. Assume False.')
            return False

    def _vehicle_geopose_callback(self, msg: GeoPoseStamped) -> None:
        """Receives vehicle :class:`geographic_msgs.msg.GeoPointStamped` message"""
        self._vehicle_geopose = msg

        # Vehicle GeoPose changes affect bbox estimate -> call _publish here
        self._publish()

    def _vehicle_altitude_callback(self, msg: Altitude) -> None:
        """Receives vehicle :class:`mavros_msgs.msg.Altitude` message"""
        self._vehicle_altitude = msg

        # No need to call _publish here:
        # Vehicle altitude changes do affect bbox estimate but _publish is already called in GeoPose callback
        # Altitude message is only subscribed to because it provides altitude AGL and AMSL unlike GeoPose
        #self._publish()

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
        # Do nothing - required by _SubscriberNode parent
        pass

    # region Mock Image Pair
    def _mock_image_pair(self, xy: GeoPt) -> Optional[ImagePair]:
        """Creates mock :class:`.ImagePair` for guessing projected FOV needed for map requests, or None if not available

        The mock image pair will be paired with a pose guess to compute the expected field of view. The expected field
        of view is used to request a new map that overlaps with what the camera is looking at.

        .. seealso:
            :meth:`._mock_map_data` and :meth:`._mock_image_data`

        :param xy: Vehicle position
        :return: Mock image pair that can be paired with a pose guess to generate a FOV guess, or None if not available
        """
        image_data = self._mock_image_data()
        map_data = self._mock_map_data(xy)
        if image_data is None or map_data is None:
            self.get_logger().warn('Missing required inputs for generating mock image PAIR.')
            return None
        contextual_map_data = ContextualMapData(rotation=0, crop=image_data.image.dim, map_data=map_data,
                                                mock_data=True)
        image_pair = ImagePair(image_data, contextual_map_data)
        return image_pair

    # TODO: make property?
    def _mock_image_data(self) -> Optional[ImageData]:
        """Creates mock :class:`.ImageData` for guessing projected FOV for map requests, or None if not available

        .. seealso:
            :meth:`._mock_map_data` and :meth:`._mock_image_pair`
        """
        if self.img_dim is None or self.camera_data is None:
            self.get_logger().warn('Missing required camera data or img dim for generating mock image data.')
            return None

        image_data = ImageData(image=Img(np.zeros(self.img_dim)),
                               frame_id='mock_image_data',
                               timestamp=self.usec,
                               camera_data=self.camera_data)
        return image_data

    def _mock_map_data(self, xy: GeoPt) -> Optional[MapData]:
        """Creates mock :class:`.MapData` for guessing projected FOV needed for map requests, or None if not available

        The mock image pair will be paired with a pose guess to compute the expected field of view. The expected field
        of view is used to request a new map that overlaps with what the camera is looking at.

        .. seealso:
            :meth:`._mock_image_pair` and :meth:`._mock_image_data`

        :param xy: Vehicle position
        :return: Mock map data with mock images but with real expected bbox, or None if not available
        """
        if self.camera_data is None or self.map_size_with_padding is None:
            self.get_logger().warn('Missing required inputs for generating mock MAP DATA.')
            return None

        if self._vehicle_altitude is None:
            self.get_logger().warn('Cannot determine vehicle altitude AGL, skipping creating mock map data')
            return None
        altitude_agl = self._vehicle_altitude.terrain
        if altitude_agl < 0:
            self.get_logger().warn(f'Altitude AGL {altitude_agl} was negative, skipping mock map data.')
            return

        # Scaling factor of image pixels := terrain_altitude
        scaling = (self.map_size_with_padding[0]/2) / self.camera_data.fx
        radius = scaling * altitude_agl

        bbox = GeoSquare(xy, radius)
        map_data = MapData(bbox=BBox(*bbox.bounds), image=Img(np.zeros(self.map_size_with_padding)))
        return map_data
    # endregion

    def _guess_fov_center(self, xy: GeoPt) -> Optional[GeoPt]:
        """Guesses WGS84 coordinates of camera field of view (FOV) projected on ground from given origin

        :param xy: Camera position
        :return: Center of the projected FOV, or None if not available
        """
        if self._gimbal_quaternion is None:
            self.get_logger().warn('Gimbal quaternion not available, cannot project gimbal FOV.')
            return None
        else:
            gimbal_attitude = Attitude(q=messaging.as_np_quaternion(self._gimbal_quaternion))
            gimbal_attitude = gimbal_attitude.to_esd()  # Need coordinates in image frame, not NED

        assert gimbal_attitude is not None

        if self.camera_data is None:
            self.get_logger().warn('Camera data not available, cannot create a mock pose to generate a FOV guess.')
            return None

        if self._terrain_altitude is None:
            self.get_logger().warn('Terrain altitude not available, cannot create a mock pose to generate a FOV guess.')
            return None

        if self._terrain_geopoint is None:
            self.get_logger().warn('Terrain geopoint not available, cannot create a mock pose to generate a FOV guess.')
            return None

        if self._home_geopoint is None:
            self.get_logger().warn('Home geopoint not available, cannot create a mock pose to generate a FOV guess.')
            return None

        translation = -gimbal_attitude.r @ np.array([self.camera_data.cx, self.camera_data.cy, -self.camera_data.fx])
        try:
            pose = Pose(gimbal_attitude.r, translation.reshape((3, 1)))
        except DataValueError as e:
            self.get_logger().warn(f'Pose input values: {gimbal_attitude.r}, {translation} were invalid: {e}.')
            return None

        try:
            assert self._terrain_altitude is not None
            assert self._terrain_geopoint is not None
            assert self._home_geopoint is not None
            mock_fixed_camera = FixedCamera(pose=pose,
                                            image_pair=self._mock_image_pair(xy),
                                            terrain_altitude_amsl=self._terrain_altitude.amsl,
                                            terrain_altitude_ellipsoid=self._terrain_geopoint.position.altitude,
                                            home_position=self._home_geopoint.position,
                                            timestamp=self.usec)
        except DataValueError as _:
            self.get_logger().warn(f'Could not create a valid mock projection of FOV.')
            return None

        if __debug__:
            export_projection = self.get_parameter('export_projection').get_parameter_value().string_value
            if export_projection != '':
                self._export_position(mock_fixed_camera.fov.c, mock_fixed_camera.fov.fov, export_projection)

        return mock_fixed_camera.fov.fov.to_crs('epsg:4326').center

    def _publish(self) -> None:
        """Publishes :class:`.BoundingBox` message

        If :py:attr:`._is_gimbal_projection_enabled`, the center of the projected camera field of view is used instead
        of vehicle position to ensure the field of view is best contained in the new map raster.
        """
        if self._vehicle_geopose is None:
            self.get_logger().warn('Cannot determine vehicle geopose, skipping publishing bbox')
            return

        if self._vehicle_altitude is None:
            self.get_logger().warn('Cannot determine vehicle altitude above ground, skipping publishing bbox')
            return None

        geopt = messaging.geopoint_to_geopt(self._vehicle_geopose.pose.position)

        if self._is_gimbal_projection_enabled:
            assert self._vehicle_geopose is not None
            projected_center = self._guess_fov_center(geopt)
            if projected_center is None:
                self.get_logger().warn('Could not project field of view center. Using vehicle position for map center '
                                       'instead.')
        else:
            projected_center = None

        assert self._vehicle_altitude is not None
        map_update_altitude_agl = self._vehicle_altitude.terrain
        if map_update_altitude_agl is None:
            self.get_logger().warn('Cannot determine altitude AGL, skipping map update.')
            return None
        if map_update_altitude_agl <= 0:
            self.get_logger().warn(f'Map update altitude {map_update_altitude_agl} should be > 0, skipping map update.')
            return None
        max_map_radius = self.get_parameter('max_map_radius').get_parameter_value().integer_value
        map_radius = get_dynamic_map_radius(self.camera_data, max_map_radius, map_update_altitude_agl)
        map_candidate = GeoSquare(projected_center if projected_center is not None else geopt, map_radius)

        # TODO: redundant with request_dem contents
        bbox = BBox(*map_candidate.bounds)
        bbox = BoundingBox(min_pt=GeoPoint(latitude=bbox.bottom, longitude=bbox.left, altitude=np.nan),
                           max_pt=GeoPoint(latitude=bbox.top, longitude=bbox.right, altitude=np.nan))
        self._bounding_box_pub.publish(bbox)
