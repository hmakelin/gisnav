"""Extends :class:`.BaseNode` to publish mock GPS (GNSS) messages that can substitute real GPS"""
import sys
import io
import pstats
import numpy as np
import cProfile
import rclpy
import importlib
import traceback
import math

from typing import Optional, Tuple, Union, Callable, get_args

from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.exceptions import ParameterAlreadyDeclaredException
from rclpy.qos import QoSPresetProfiles
from rclpy.timer import Timer

from scipy.spatial.transform import Rotation

from geographic_msgs.msg import BoundingBox, GeoPoint as ROSGeoPoint

from gisnav.data import BBox, Position, Pose, FixedCamera, Attitude, Altitude, DataValueError, MapData, ImagePair, \
    ImageData, Dim, Img, ContextualMapData
from gisnav.geo import GeoPoint, GeoSquare
from gisnav.assertions import assert_type
from gisnav.autopilots.autopilot import Autopilot
from gisnav.nodes.map_node import MapNode


class BBoxNode(Node):
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

    DEFAULT_PUBLISH_RATE = 1
    """Default publish rate for :class:`.BoundingBox` messages in Hz"""

    _ROS_PARAMS = [
        ('gimbal_projection', ROS_D_GIMBAL_PROJECTION),
        ('max_map_radius', ROS_D_MAX_MAP_RADIUS),
        ('export_projection', ROS_D_DEBUG_EXPORT_PROJECTION),
    ]
    """ROS parameters used by this node to declare"""

    def __init__(self, name: str, px4_micrortps: bool = True, publish_rate: int = DEFAULT_PUBLISH_RATE):
        """Class initializer

        :param name: Node name
        :param px4_micrortps: Set False to use MAVROS, PX4 microRTPS by default
        """
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self._declare_ros_params()
        self._bbox_pub = self.create_publisher(BoundingBox, 'bbox', QoSPresetProfiles.SENSOR_DATA.value)
        self._geopoint = None
        self._geopoint_sub = self.create_subscription(ROSGeoPoint, 'ground_geopoint', self._geopoint_callback,
                                                      QoSPresetProfiles.SENSOR_DATA.value)
        self._home_position = None
        self._home_position_sub = self.create_subscription(ROSGeoPoint, MapNode.DEFAULT_HOME_POSITION_TOPIC,
                                                           self._home_position_callback,
                                                           QoSPresetProfiles.SENSOR_DATA.value)
        if px4_micrortps:
            ap: Autopilot = self._load_autopilot('gisnav.autopilots.px4_micrortps.PX4microRTPS')
            self._bridge = ap(self, lambda x: None)  # dummy callback for image_raw topic (not needed here)
        else:
            ap: Autopilot = self._load_autopilot('gisnav.autopilots.ardupilot_mavros.ArduPilotMAVROS')
            self._bridge = ap(self, lambda x: None)  # dummy callback for image_raw topic (not needed here)
        self._timer = self._setup_bbox_update_timer(publish_rate)

    # TODO redundant with basenode
    @property
    def _img_dim(self) -> Optional[Dim]:
        """Image resolution from latest :class:`px4_msgs.msg.CameraInfo` message, None if not available"""
        if self._bridge.camera_data is not None:
            return self._bridge.camera_data.dim
        else:
            self.get_logger().warn('Camera data was not available, returning None as declared image size.')
            return None

    # TODO redundant with basenode, map node
    @property
    def _map_size_with_padding(self) -> Optional[Tuple[int, int]]:
        """Padded map size tuple (height, width) or None if the information is not available.

        Because the deep learning models used for predicting matching keypoints or poses between camera image frames
        and map rasters are not assumed to be rotation invariant in general, the map rasters are rotated based on
        camera yaw so that they align with the camera images. To keep the scale of the map after rotation the same,
        black corners would appear unless padding is used. Retrieved maps therefore have to be squares with the side
        lengths matching the diagonal of the camera frames so that scale is preserved and no black corners appear in
        the map rasters after rotation. The height and width will both be equal to the diagonal of the declared
        (:py:attr:`._bridge.camera_data`) camera frame dimensions.
        """
        if self._img_dim is None:
            self.get_logger().warn(f'Dimensions not available - returning None as map size.')
            return None
        diagonal = int(np.ceil(np.sqrt(self._img_dim.width ** 2 + self._img_dim.height ** 2)))
        assert_type(diagonal, int)
        return diagonal, diagonal

    @property
    def _geopoint(self) -> Optional[ROSGeoPoint]:
        """GeoPoint of ground directly below drone"""
        return self.__geopoint

    @_geopoint.setter
    def _geopoint(self, value: Optional[GeoPoint]) -> None:
        self.__geopoint = value

    # TODO: redundant implementation in base_node
    @property
    def _vehicle_position(self) -> Optional[Position]:
        """Vehicle position guess in WGS84 coordinates and altitude in meters above ground, None if not available"""
        if self._bridge.global_position is not None:
            assert_type(self._bridge.global_position, get_args(Optional[GeoPoint]))

            crs = 'epsg:4326'
            if self._bridge.attitude is None:
                self.get_logger().warn('Vehicle attitude not yet available, cannot determine vehicle Position.')
                return None

            try:
                position = Position(
                    xy=self._bridge.global_position,
                    altitude=Altitude(
                        agl=self._bridge.altitude_agl(self.terrain_altitude_amsl) if self.terrain_altitude_amsl is not None else None,
                        amsl=self._bridge.altitude_amsl,
                        ellipsoid=self._bridge.altitude_ellipsoid,
                        home=self._bridge.altitude_home
                    ),
                    attitude=self._bridge.attitude,
                    timestamp=self._bridge.synchronized_time
                )
                return position
            except DataValueError as dve:
                self.get_logger().warn(f'Error determining vehicle position:\n{dve},\n{traceback.print_exc()}.')
                return None
        else:
            return None

    # TODO Redundant implementation in map_node
    def _setup_bbox_update_timer(self, publish_rate: int) -> Timer:
        """Returns a timer to publish :class:`.BoundingBox`

        :param publish_rate: Publishing rate for the timer (in Hz)
        :return: The :class:`.Timer` instancen
        """
        if publish_rate <= 0:
            error_msg = f'Update rate must be positive ({publish_rate} Hz provided).'
            self.get_logger().error(error_msg)
            raise ValueError(error_msg)
        timer = self.create_timer(1 / publish_rate, self._publish)
        return timer

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

    # TODO: redundant implementation in BaseNode
    def _load_autopilot(self, autopilot: str) -> Callable:
        """Returns :class:`.Autopilot` instance from provided class path

        :param autopilot: Full path of :class:`.Autopilot` adapter class
        :return: Initialized :class:`.Autopilot` instance
        """
        assert_type(autopilot, str)
        module_name, class_name = autopilot.rsplit('.', 1)
        class_ = self._import_class(class_name, module_name)
        return class_

    # TODO: redundant implementation in BaseNode
    def _import_class(self, class_name: str, module_name: str) -> type:
        """Dynamically imports class from given module if not yet imported

        :param class_name: Name of the class to import
        :param module_name: Name of module that contains the class
        :return: Imported class
        """
        if module_name not in sys.modules:
            self.get_logger().info(f'Importing module {module_name}.')
            importlib.import_module(module_name)
        imported_class = getattr(sys.modules[module_name], class_name, None)
        assert imported_class is not None, f'{class_name} was not found in module {module_name}.'
        return imported_class

    # TODO: redundant implementation in map_node, base_node
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

    def _geopoint_callback(self, msg) -> None:
        """Receives ground geopoint"""
        self._geopoint = msg

    def _home_position_callback(self, msg) -> None:
        """Receives home position"""
        self._home_position = msg

    # TODO: redundant implementation in basenode
    def _get_dynamic_map_radius(self, altitude: Union[int, float]) -> int:
        """Returns map radius that adjusts for camera altitude to be used for new map requests

        :param altitude: Altitude of camera in meters
        :return: Suitable map radius in meters
        """
        assert_type(altitude, get_args(Union[int, float]))
        max_map_radius = self.get_parameter('max_map_radius').get_parameter_value().integer_value

        if self._bridge.camera_data is not None:
            hfov = 2 * math.atan(self._bridge.camera_data.dim.width / (2 * self._bridge.camera_data.fx))
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

    # region Mock Image Pair
    def _mock_image_pair(self, origin: Position) -> Optional[ImagePair]:
        """Creates mock :class:`.ImagePair` for guessing projected FOV needed for map requests, or None if not available

        The mock image pair will be paired with a pose guess to compute the expected field of view. The expected field
        of view is used to request a new map that overlaps with what the camera is looking at.

        .. seealso:
            :meth:`._mock_map_data` and :meth:`._mock_image_data`

        :param origin: Vehicle position
        :return: Mock image pair that can be paired with a pose guess to generate a FOV guess, or None if not available
        """
        image_data = self._mock_image_data()
        map_data = self._mock_map_data(origin)
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
        if self._img_dim is None or self._bridge.synchronized_time is None or self._bridge.camera_data is None:
            self.get_logger().warn('Missing required inputs for generating mock image DATA.')
            return None

        image_data = ImageData(image=Img(np.zeros(self._img_dim)),
                               frame_id='mock_image_data',  # TODO
                               timestamp=self._bridge.synchronized_time,
                               camera_data=self._bridge.camera_data)
        return image_data

    def _mock_map_data(self, origin: Position) -> Optional[MapData]:
        """Creates mock :class:`.MapData` for guessing projected FOV needed for map requests, or None if not available

        The mock image pair will be paired with a pose guess to compute the expected field of view. The expected field
        of view is used to request a new map that overlaps with what the camera is looking at.

        .. seealso:
            :meth:`._mock_image_pair` and :meth:`._mock_image_data`

        :param origin: Vehicle position
        :return: Mock map data with mock images but with real expected bbox, or None if not available
        """
        assert_type(origin, Position)
        if self._bridge.camera_data is None or self._map_size_with_padding is None:
            self.get_logger().warn('Missing required inputs for generating mock MAP DATA.')
            return None

        if self.terrain_altitude_amsl is None:
            self.get_logger().warn('Cannot determine altitude above ground, skipping creating mock map data')
            return None

        # Scaling factor of image pixels := terrain_altitude
        scaling = (self._map_size_with_padding[0]/2) / self._bridge.camera_data.fx
        altitude = self._bridge.altitude_agl(self.terrain_altitude_amsl)  # TODO: does not use origin!
        if altitude is None:
            self.get_logger().warn('Cannot determine altitude AGL, skipping mock map data.')
            return
        if altitude < 0:
            self.get_logger().warn(f'Altitude AGL {altitude} was negative, skipping mock map data.')
            return
        radius = scaling * altitude

        assert_type(origin.xy, GeoPoint)
        bbox = GeoSquare(origin.xy, radius)
        #map_data = MapData(bbox=bbox, image=Img(np.zeros(self._map_size_with_padding)))
        map_data = MapData(bbox=BBox(*bbox.bounds), image=Img(np.zeros(self._map_size_with_padding)))
        return map_data
    # endregion

    def _guess_fov_center(self, origin: Position) -> Optional[GeoPoint]:
        """Guesses WGS84 coordinates of camera field of view (FOV) projected on ground from given origin

        Triggered by :meth:`._map_update_timer_callback` when gimbal projection is enabled to determine center
        coordinates for next WMS GetMap request.

        :param origin: Camera position
        :return: Center of the projected FOV, or None if not available
        """
        static_camera = self.get_parameter('static_camera').get_parameter_value().bool_value

        if self._bridge.gimbal_set_attitude is None:
            if not static_camera:
                self.get_logger().warn('Gimbal set attitude not available, cannot project gimbal FOV.')
                return None
            else:
                if self._bridge.attitude is not None:
                    attitude = self._bridge.attitude.as_rotation()
                    attitude *= Rotation.from_euler('XYZ', [0, -np.pi/2, 0])
                    gimbal_set_attitude = Attitude(attitude.as_quat()).to_esd().r
                else:
                    self.get_logger().warn('Vehicle attitude not available, will not provide pose guess for static '
                                           'camera.')
                    return None
        else:
            assert_type(self._bridge.gimbal_set_attitude, Attitude)
            gimbal_set_attitude = self._bridge.gimbal_set_attitude.to_esd()  # Need coordinates in image frame, not NED

        assert gimbal_set_attitude is not None

        if self._bridge.camera_data is None:
            self.get_logger().warn('Camera data not available, could not create a mock pose to generate a FOV guess.')
            return None

        translation = -gimbal_set_attitude.r @ np.array([self._bridge.camera_data.cx, self._bridge.camera_data.cy,
                                                         -self._bridge.camera_data.fx])

        try:
            pose = Pose(gimbal_set_attitude.r, translation.reshape((3, 1)))
        except DataValueError as e:
            self.get_logger().warn(f'Pose input values: {gimbal_set_attitude.r}, {translation} were invalid: {e}.')
            return None

        try:
            snapshot = self._bridge.snapshot(self.terrain_altitude_amsl)
            mock_fixed_camera = FixedCamera(pose=pose, image_pair=self._mock_image_pair(origin),
                                            #snapshot=self._bridge.snapshot(self.terrain_altitude_amsl),
                                            terrain_altitude_amsl=snapshot.terrain_altitude.amsl,
                                            terrain_altitude_ellipsoid=snapshot.terrain_altitude.ellipsoid,
                                            home_position=self._home_position,
                                            timestamp=self._bridge.synchronized_time)
        except DataValueError as _:
            self.get_logger().warn(f'Could not create a valid mock projection of FOV.')
            return None

        if __debug__:
            export_projection = self.get_parameter('export_projection').get_parameter_value().string_value
            if export_projection != '':
                self._export_position(mock_fixed_camera.fov.c, mock_fixed_camera.fov.fov, export_projection)

        return mock_fixed_camera.fov.fov.to_crs('epsg:4326').center

    @property
    def terrain_altitude_amsl(self) -> Optional[float]:
        if self._geopoint is None:
            self.get_logger().warn('Cannot determine altitude above ground, skipping publishing bbox')
            return None
        terrain_altitude_amsl = self._geopoint.altitude - self._bridge._egm96.height(self._geopoint.latitude,
                                                                                     self._geopoint.longitude)
        return terrain_altitude_amsl

    def _publish(self) -> None:
        """Publishes :class:`.BoundingBox` message"""
        if self._vehicle_position is None:
            self.get_logger().warn('Vehicle position unknown - will not publish')
            return

        if self._is_gimbal_projection_enabled:
            assert self._vehicle_position is not None
            projected_center = self._guess_fov_center(self._vehicle_position)
            if projected_center is None:
                self.get_logger().warn('Could not project field of view center. Using vehicle position for map center '
                                       'instead.')
        else:
            projected_center = None

        if self.terrain_altitude_amsl is None:
            self.get_logger().warn('Cannot determine altitude above ground, skipping publishing bbox')
            # TODO: use assumption instead? E.g. first bbox needs to be published before dem is retrieved
            return None

        map_update_altitude = self._bridge.altitude_agl(self.terrain_altitude_amsl)
        if map_update_altitude is None:
            self.get_logger().warn('Cannot determine altitude AGL, skipping map update.')
            return None
        if map_update_altitude <= 0:
            self.get_logger().warn(f'Map update altitude {map_update_altitude} should be > 0, skipping map update.')
            return None
        map_radius = self._get_dynamic_map_radius(map_update_altitude)
        map_candidate = GeoSquare(projected_center if projected_center is not None else self._vehicle_position.xy,
                                  map_radius)

        # TODO: redundant with request_dem contents
        bbox = BBox(*map_candidate.bounds)
        bbox = BoundingBox(min_pt=ROSGeoPoint(latitude=bbox.bottom, longitude=bbox.left, altitude=np.nan),
                           max_pt=ROSGeoPoint(latitude=bbox.top, longitude=bbox.right, altitude=np.nan))
        self._bbox_pub.publish(bbox)

    def destroy(self) -> None:
        """Unsubscribes ROS topics and destroys timer"""
        if self._geopoint_sub is not None:
            self._geopoint_sub.destroy()
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

    bbox_node = None
    try:
        rclpy.init(args=args)
        bbox_node = BBoxNode('bbox_node', px4_micrortps='--mavros' not in sys.argv)
        rclpy.spin(bbox_node)
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
        if bbox_node is not None:
            bbox_node.destroy()
            bbox_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
