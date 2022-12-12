"""Module that contains the pose estimation node"""
import os
import sys
import yaml
import numpy as np
import traceback
import importlib
from typing import Optional, Union, get_args

import cv2
from scipy.spatial.transform import Rotation
from rclpy.qos import QoSPresetProfiles
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from sensor_msgs.msg import CameraInfo, Image
from mavros_msgs.msg import Altitude
from geometry_msgs.msg import Quaternion
from geographic_msgs.msg import GeoPoint, GeoPointStamped, GeoPose, GeoPoseStamped
from std_msgs.msg import Float32
from gisnav_msgs.msg import OrthoImage3D

from . import messaging
from .base.camera_subscriber_node import _CameraSubscriberNode
from ..pose_estimators.pose_estimator import PoseEstimator
from ..assertions import assert_type, assert_ndim, assert_shape
from ..data import Pose, FixedCamera, DataValueError, ImageData, Img, Attitude, ContextualMapData, BBox, MapData, \
    ImagePair, Position
from ..geo import GeoPt, GeoTrapezoid


class PoseEstimationNode(_CameraSubscriberNode):
    """Publishes pose between two images"""

    # Encoding of input video (input to CvBridge)
    # e.g. gscam2 only supports bgr8 so this is used to override encoding in image header
    _IMAGE_ENCODING = 'bgr8'

    ROS_D_DEBUG_EXPORT_POSITION = '' # 'position.json'
    """Default filename for exporting GeoJSON containing estimated field of view and position

    Set to '' to disable
    """

    ROS_D_MISC_MAX_PITCH = 30
    """Default maximum camera pitch from nadir in degrees for attempting to estimate pose against reference map

    .. seealso::
        :py:attr:`.ROS_D_MAP_UPDATE_MAX_PITCH` 
        :py:attr:`.ROS_D_MAP_UPDATE_GIMBAL_PROJECTION`
    """

    ROS_D_MISC_MIN_MATCH_ALTITUDE = 80
    """Default minimum ground altitude in meters under which matches against map will not be attempted"""

    ROS_D_MISC_ATTITUDE_DEVIATION_THRESHOLD = 10
    """Magnitude of allowed attitude deviation of estimate from expectation in degrees"""

    _ROS_PARAMS_DEFAULTS = [
        ('max_pitch', ROS_D_MISC_MAX_PITCH),
        ('min_match_altitude', ROS_D_MISC_MIN_MATCH_ALTITUDE),
        ('attitude_deviation_threshold', ROS_D_MISC_ATTITUDE_DEVIATION_THRESHOLD),
        ('export_position', ROS_D_DEBUG_EXPORT_POSITION),
    ]
    """ROS parameters used by this node to declare"""

    def __init__(self, name: str, params_file: str,  package_share_dir: str) -> None:
        """Node initializer

        :param name: Node name
        :param params_file: Pose estimator params file
        :param package_share_dir: Package shared directory
        """
        super().__init__(name, ros_param_defaults=self._ROS_PARAMS_DEFAULTS)
        self._package_share_dir = package_share_dir  # Needed for loading pose estimator params file
        params = self._load_config(params_file)
        module_name, class_name = params.get('class_name', '').rsplit('.', 1)
        pose_estimator: PoseEstimator = self._import_class(class_name, module_name)
        self._estimator = pose_estimator(*params.get('args', []))

        self._map = None

        # Converts image_raw to cv2 compatible image
        self._cv_bridge = CvBridge()

        # Subscribers
        self._orthoimage_3d = None
        self._orthoimage_3d_sub = self.create_subscription(OrthoImage3D,
                                                           messaging.ROS_TOPIC_ORTHOIMAGE,
                                                           self._orthoimage_3d_callback,
                                                           QoSPresetProfiles.SENSOR_DATA.value)
        self._terrain_altitude = None
        self._terrain_altitude_sub = self.create_subscription(Altitude,
                                                              messaging.ROS_TOPIC_TERRAIN_ALTITUDE,
                                                              self._terrain_altitude_callback,
                                                              QoSPresetProfiles.SENSOR_DATA.value)
        self._terrain_geopoint = None
        self._terrain_geopoint_sub = self.create_subscription(GeoPointStamped,
                                                              messaging.ROS_TOPIC_TERRAIN_GEOPOINT,
                                                              self._terrain_geopoint_callback,
                                                              QoSPresetProfiles.SENSOR_DATA.value)
        self._vehicle_altitude = None
        self._vehicle_altitude_sub = self.create_subscription(Altitude,
                                                              messaging.ROS_TOPIC_VEHICLE_ALTITUDE,
                                                              self._vehicle_altitude_callback,
                                                              QoSPresetProfiles.SENSOR_DATA.value)
        self._gimbal_quaternion = None
        self._gimbal_quaternion_sub = self.create_subscription(Quaternion,
                                                               messaging.ROS_TOPIC_GIMBAL_QUATERNION,
                                                               self._gimbal_quaternion_callback,
                                                               QoSPresetProfiles.SENSOR_DATA.value)
        self._vehicle_geopose = None
        self._vehicle_geopose_sub = self.create_subscription(GeoPoseStamped,
                                                             messaging.ROS_TOPIC_VEHICLE_GEOPOSE,
                                                             self._vehicle_geopose_callback,
                                                             QoSPresetProfiles.SENSOR_DATA.value)
        self._home_geopoint = None
        self._home_geopoint_sub = self.create_subscription(GeoPointStamped,
                                                           messaging.ROS_TOPIC_HOME_GEOPOINT,
                                                           self._home_geopoint_callback,
                                                           QoSPresetProfiles.SENSOR_DATA.value)

        # Publishers
        self._geopose_pub = self.create_publisher(GeoPoseStamped,
                                                  messaging.ROS_TOPIC_VEHICLE_GEOPOSE_ESTIMATE,
                                                  QoSPresetProfiles.SENSOR_DATA.value)
        self._altitude_pub = self.create_publisher(Altitude,
                                                   messaging.ROS_TOPIC_VEHICLE_ALTITUDE_ESTIMATE,
                                                   QoSPresetProfiles.SENSOR_DATA.value)

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

    def _load_config(self, yaml_file: str) -> dict:
        """Loads params from the provided YAML file

        :param yaml_file: Path to the yaml file
        :return: The loaded yaml file as dictionary
        """
        assert_type(yaml_file, str)
        with open(os.path.join(self._package_share_dir, yaml_file), 'r') as f:
            # noinspection PyBroadException
            try:
                config = yaml.safe_load(f)
                self.get_logger().info(f'Loaded params:\n{config}.')
                return config
            except Exception as e:
                self.get_logger().error(f'Could not load params file {yaml_file} because of unexpected exception.')
                raise

    @property
    def _altitude_scaling(self) -> Optional[float]:
        """Returns camera focal length divided by camera altitude in meters."""
        if self.camera_data is not None and self._vehicle_altitude is not None:
            return self.camera_data.fx / self._vehicle_altitude.terrain  # TODO: assumes fx == fy
        else:
            self.get_logger().warn('Could not estimate elevation scale because camera focal length and/or vehicle '
                                   'altitude is unknown.')
            return None

    @property
    def _r_guess(self) -> Optional[np.ndarray]:
        """Gimbal rotation matrix guess (based on :class:`px4_msgs.GimbalDeviceSetAttitude` message)

        .. note::
            Should be roughly same as rotation matrix stored in :py:attr:`._pose_guess`, even though it is derived via
            a different route. If gimbal is not stabilized to its set position, the rotation matrix will be different.
        """
        if self._gimbal_quaternion is None:
            self.get_logger().warn('Gimbal set attitude not available, will not provide pose guess.')
            return None
        else:
            gimbal_attitude = Attitude(q=messaging.as_np_quaternion(self._gimbal_quaternion))
            gimbal_attitude = gimbal_attitude.to_esd()  # Need coordinates in image frame, not NED
            return gimbal_attitude.r

    @property
    def _contextual_map_data(self) -> Optional[ContextualMapData]:
        """Returns contextual (rotated) map data for pose estimation

        :return: Rotated map with associated metadata, or None if not available
        """
        # Get cropped and rotated map
        if self._gimbal_quaternion is not None:
            gimbal_attitude = Attitude(q=messaging.as_np_quaternion(self._gimbal_quaternion), extrinsic=True)
            camera_yaw = gimbal_attitude.yaw
            assert_type(camera_yaw, float)
            assert -np.pi <= camera_yaw <= np.pi, f'Unexpected gimbal yaw value: {camera_yaw} ([-pi, pi] expected).'
        else:
            self.get_logger().warn(f'Camera yaw unknown, cannot estimate pose.')
            return None

        return ContextualMapData(rotation=camera_yaw, map_data=self._map, crop=self.img_dim,
                                 altitude_scaling=self._altitude_scaling)

    def _should_estimate(self) -> bool:
        """Determines whether :meth:`._estimate` should be called

        Match should be attempted if (1) a reference map has been retrieved, (2) camera roll or pitch is not too high
        (e.g. facing horizon instead of nadir), and (3) drone is not flying too low.

        :return: True if pose estimation be attempted
        """
        # Check condition (1) - that _map_data exists
        if self._map is None:
            self.get_logger().warn(f'No reference map available. Skipping pose estimation.')
            return False

        # Check condition (2) - whether camera roll/pitch is too large
        max_pitch = self.get_parameter('max_pitch').get_parameter_value().integer_value
        if self._camera_roll_or_pitch_too_high(max_pitch):
            self.get_logger().warn(f'Camera roll or pitch not available or above limit {max_pitch}. Skipping pose '
                                   f'estimation.')
            return False

        # Check condition (3) - whether vehicle altitude is too low
        min_alt = self.get_parameter('min_match_altitude').get_parameter_value().integer_value
        assert min_alt > 0
        if self._vehicle_altitude is None or self._vehicle_altitude.terrain is np.nan:
            self.get_logger().warn('Cannot determine altitude AGL, skipping map update.')
            return None

        if self._vehicle_altitude.terrain < min_alt:
            self.get_logger().warn(f'Assumed altitude {self._vehicle_altitude.terrain} was lower than minimum '
                                   f'threshold for matching ({min_alt}) or could not be determined. Skipping pose '
                                   f'estimation.')
            return False

        return True

    def image_callback(self, msg: Image) -> None:
        """Handles latest :class:`px4_msgs.msg.Image` message

        :param msg: The :class:`px4_msgs.msg.Image` message from the PX4-ROS 2 bridge
        """
        cv_image = self._cv_bridge.imgmsg_to_cv2(msg, self._IMAGE_ENCODING)

        # Check that image dimensions match declared dimensions
        if self.img_dim is not None:
            cv_img_shape = cv_image.shape[0:2]
            assert cv_img_shape == self.img_dim, f'Converted cv_image shape {cv_img_shape} did not match '\
                                                  f'declared image shape {self.img_dim}.'

        if self.camera_data is None:
            self.get_logger().warn('Camera data not yet available, skipping frame.')
            return None

        image_data = ImageData(image=Img(cv_image), frame_id=msg.header.frame_id, timestamp=self.usec,
                               camera_data=self.camera_data)

        if self._should_estimate():
            assert self._map is not None
            assert self.camera_data is not None
            assert hasattr(self._map, 'image'), 'Map data unexpectedly did not contain the image data.'

            image_pair = ImagePair(image_data, self._contextual_map_data)
            pose = self._estimator.estimate(image_data.image.arr, image_pair.ref.image.arr, self.camera_data.k)
            if pose is not None:
                pose = Pose(*pose)
                self._post_process_pose(pose, image_pair)
            else:
                self.get_logger().warn(f'Worker did not return a pose, skipping this frame.')
                return None

    def _orthoimage_3d_callback(self, msg: OrthoImage3D) -> None:
        """Handles latest :class:`gisnav_msgs.msg.OrthoImage3D` message

        :param msg: Latest :class:`gisnav_msgs.msg.OrthoImage3D` message
        """
        bbox = messaging.bounding_box_to_bbox(msg.bbox)
        img = self._cv_bridge.imgmsg_to_cv2(msg.img, desired_encoding='passthrough')
        dem = self._cv_bridge.imgmsg_to_cv2(msg.dem, desired_encoding='passthrough')
        assert_type(img, np.ndarray)
        assert_ndim(dem, 2)
        assert_shape(dem, img.shape[0:2])

        assert self.map_size_with_padding is not None

        # Should already have received camera info so map_size_with_padding should not be None
        assert img.shape[0:2] == self.map_size_with_padding, f'Decoded map {img.shape[0:2]} is not of specified ' \
                                                             f'size {self.map_size_with_padding}.'

        elevation = Img(dem) if dem is not None else None
        self._map = MapData(bbox=bbox, image=Img(img), elevation=elevation)

    def _terrain_altitude_callback(self, msg: Altitude) -> None:
        """Handles latest terrain altitude message

        :param msg: Latest :class:`mavros_msgs.msg.Altitude` message
        """
        self._terrain_altitude = msg

    def _home_geopoint_callback(self, msg: GeoPointStamped) -> None:
        """Receives home :class:`geographic_msgs.msg.GeoPointStamped` message

        :param msg: Latest :class:`geographic_msgs.msg.GeoPointStamped` message
        """
        self._home_geopoint = msg

    def _terrain_geopoint_callback(self, msg: GeoPointStamped) -> None:
        """Receives terrain :class:`geographic_msgs.msg.GeoPointStamped` message

        :param msg: Latest :class:`geographic_msgs.msg.GeoPointStamped` message
        """
        self._terrain_geopoint = msg

    def _vehicle_altitude_callback(self, msg: Altitude) -> None:
        """Receives vehicle :class:`mavros_msgs.msg.Altitude` message

        :param msg: Latest :class:`mavros_msgs.msg.Altitude` message
        """
        self._vehicle_altitude = msg

    def _gimbal_quaternion_callback(self, msg: Quaternion) -> None:
        """Receives gimbal :class:`geometry_msgs.msg.Quaternion` message

        .. note::
            This could be gimbal set attitude, not actual attitude

        :param msg: Latest :class:`geometry_msgs.msg.Quaternion` message
        """
        self._gimbal_quaternion = msg

    def _vehicle_geopose_callback(self, msg: GeoPoseStamped) -> None:
        """Receives vehicle :class:`geographic_msgs.msg.GeoPoseStamped` message

        :param msg: Latest :class:`geographic_msgs.msg.GeoPoseStamped` message
        """
        self._vehicle_geopose = msg

    def _post_process_pose(self, pose: Pose, image_pair: ImagePair) -> None:
        """Handles estimated pose

        :param pose: Pose result from pose estimation node worker
        :param image_pair: Image pair input from which pose was estimated
        """
        try:
            # Compute DEM value at estimated position
            # This is in camera intrinsic (pixel) units with origin at whatever the DEM uses
            # For example, the USGS DEM uses NAVD 88
            x, y = -pose.t.squeeze()[0:2]
            x, y = int(x), int(y)
            elevation = image_pair.ref.elevation.arr[y, x]
            pose = Pose(pose.r, pose.t - elevation)
        except DataValueError as _:
            self.get_logger().warn(f'Estimated pose was not valid, skipping this frame.')
            return None
        except IndexError as __:
            # TODO: might be able to handle this
            self.get_logger().warn(f'Estimated pose was not valid, skipping this frame.')
            return None

        try:
            assert self._terrain_geopoint is not None
            assert self._terrain_altitude is not None
            fixed_camera = FixedCamera(pose=pose, image_pair=image_pair,
                                       terrain_altitude_amsl=self._terrain_altitude.amsl,
                                       terrain_altitude_ellipsoid=self._terrain_geopoint.position.altitude,
                                       home_position=self._home_geopoint.position, #messaging.geopoint_to_geopt(self._home_geopoint),
                                       timestamp=image_pair.qry.timestamp)
        except DataValueError as _:
            self.get_logger().warn(f'Could not estimate a valid camera position, skipping this frame.')
            return None

        if not self._is_valid_estimate(fixed_camera, self._r_guess):
            self.get_logger().warn('Estimate did not pass post-estimation validity check, skipping this frame.')
            return None

        assert fixed_camera is not None
        # noinspection PyUnreachableCode
        if __debug__:
            # Visualize projected FOV estimate
            fov_pix = fixed_camera.fov.fov_pix
            ref_img = fixed_camera.image_pair.ref.image.arr
            map_with_fov = cv2.polylines(ref_img.copy(),
                                         [np.int32(fov_pix)], True,
                                         255, 3, cv2.LINE_AA)

            img = np.vstack((map_with_fov, fixed_camera.image_pair.qry.image.arr))
            cv2.imshow("Projected FOV", img)
            cv2.waitKey(1)

            # Export GeoJSON
            export_geojson = self.get_parameter('export_position').get_parameter_value().string_value
            if export_geojson != '':
                self._export_position(fixed_camera.position.xy, fixed_camera.fov.fov, export_geojson)

        self.publish(fixed_camera.position)

    def _is_valid_estimate(self, fixed_camera: FixedCamera, r_guess_: np.ndarray) -> bool:
        """Returns True if the estimate is valid

        Compares computed estimate to guess based on set gimbal device attitude. This will reject estimates made when
        the gimbal was not stable (which is strictly not necessary), which is assumed to filter out more inaccurate
        estimates.
        """
        if r_guess_ is None:
            self.get_logger().warn('Reference gimbal attitude (r_guess) was not available, cannot do post-estimation '
                                   'validity check.')
            return False

        r_guess = Rotation.from_matrix(r_guess_)
        # Adjust for map rotation
        camera_yaw = fixed_camera.image_pair.ref.rotation
        camera_yaw = Rotation.from_euler('xyz', [0, 0, camera_yaw], degrees=False)
        r_guess *= camera_yaw

        r_estimate = Rotation.from_matrix(fixed_camera.pose.r)

        magnitude = Rotation.magnitude(r_estimate * r_guess.inv())

        threshold = self.get_parameter('attitude_deviation_threshold').get_parameter_value().integer_value
        threshold = np.radians(threshold)

        if magnitude > threshold:
            self.get_logger().warn(f'Estimated rotation difference to expected was too high (magnitude '
                                   f'{np.degrees(magnitude)}).')
            return False

        return True

    def _camera_roll_or_pitch_too_high(self, max_pitch: Union[int, float]) -> bool:
        """Returns True if (set) camera roll or pitch exceeds given limit OR camera pitch is unknown

        Used to determine whether camera roll or pitch is too high up from nadir to make matching against a map
        not worthwhile.

        :param max_pitch: The limit for the pitch in degrees from nadir over which it will be considered too high
        :return: True if pitch is too high
        """
        assert_type(max_pitch, get_args(Union[int, float]))
        pitch = None
        if self._gimbal_quaternion is not None:
            gimbal_attitude = Attitude(q=messaging.as_np_quaternion(self._gimbal_quaternion), extrinsic=True)
            # TODO: do not assume zero roll here - camera attitude handling needs refactoring
            # +90 degrees to re-center from FRD frame to nadir-facing camera as origin for max pitch comparison
            pitch = np.degrees(gimbal_attitude.pitch) + 90
        else:
            self.get_logger().warn('Gimbal attitude was not available, assuming camera pitch too high.')
            return True

        assert pitch is not None
        if pitch > max_pitch:
            self.get_logger().warn(f'Camera pitch {pitch} is above limit {max_pitch}.')
            return True

        return False

    def _export_position(self, position: GeoPt, fov: GeoTrapezoid, filename: str) -> None:
        """Exports the computed position and field of view (FOV) into a geojson file

        The GeoJSON file is not used by the node but can be accessed by GIS software to visualize the data it contains.

        :param position: Computed camera position or projected principal point for gimbal projection
        :param: fov: Field of view of camera
        :param filename: Name of file to write into
        :return:
        """
        assert_type(position, GeoPt)
        assert_type(fov, GeoTrapezoid)
        assert_type(filename, str)
        try:
            position._geoseries.append(fov._geoseries).to_file(filename)
        except Exception as e:
            self.get_logger().error(f'Could not write file {filename} because of exception:'
                                    f'\n{e}\n{traceback.print_exc()}')

    def publish(self, position: Position) -> None:
        """Publishes estimated position

        :param position: Estimated position
        """
        altitude_msg = Altitude(header=messaging.create_header('base_link'),
                                amsl=position.altitude.amsl,
                                local=position.altitude.home,
                                relative=position.altitude.home,
                                terrain=position.altitude.agl,
                                bottom_clearance=position.altitude.agl)
        self._altitude_pub.publish(altitude_msg)

        geopose_msg = GeoPoseStamped(
            header=messaging.create_header('base_link'),
            pose=GeoPose(
                position=GeoPoint(latitude=position.lat, longitude=position.lon, altitude=position.altitude.ellipsoid),
                orientation=messaging.as_ros_quaternion(position.attitude.q)
            )
        )
        self._geopose_pub.publish(geopose_msg)
