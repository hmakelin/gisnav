"""Module that contains the pose estimation node"""
import json
import math
from dataclasses import dataclass
from typing import Optional, Tuple, TypedDict, Union, get_args

import cv2
import numpy as np
import requests
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geographic_msgs.msg import GeoPoint, GeoPointStamped, GeoPose, GeoPoseStamped
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import Altitude
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image

from gisnav_msgs.msg import OrthoImage3D  # type: ignore

from ..assertions import ROS, assert_type, cache_if, enforce_types
from ..data import (
    Attitude,
    ContextualMapData,
    DataValueError,
    FixedCamera,
    ImageData,
    ImagePair,
    Img,
    Pose,
    Position,
)
from ..geo import GeoPt, GeoTrapezoid
from . import messaging


class PoseEstimationNode(Node):
    """Estimates and publishes pose between two images

    Compares images from :class:`sensor_msgs.msg.Image` message to maps from
    :class:`gisnav_msgs.msg.OrthoImage3D` message to estimate
    :class:`geographic_msgs.msg.GeoPoseStamped`.
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
    Max delay for messages with fast dynamics that go "stale" quickly, e.g. local
    position and attitude. The delay can be a bit higher than is intuitive because
    the vehicle EKF should be able to fuse things with fast dynamics with higher
    delay as long as the timestamps are accurate.
    """

    # _IMAGE_ENCODING = "bgr8"
    # """
    # Encoding of input video (input to CvBridge) e.g. gscam2 only supports bgr8
    # so this is used to override encoding in image header
    # """

    ROS_D_POSE_ESTIMATOR_ENDPOINT = "http://localhost:8090/predictions/loftr"
    """Default pose estimator endpoint URL"""

    ROS_D_MISC_MAX_PITCH = 30
    """Default maximum camera pitch from nadir in degrees for attempting to
    estimate pose against reference map

    .. seealso::
        :py:attr:`.ROS_D_MAP_UPDATE_MAX_PITCH`
        :py:attr:`.ROS_D_MAP_UPDATE_GIMBAL_PROJECTION`
    """

    ROS_D_MISC_MIN_MATCH_ALTITUDE = 80
    """Default minimum ground altitude in meters under which matches against
    map will not be attempted"""

    ROS_D_MISC_ATTITUDE_DEVIATION_THRESHOLD = 10
    """Magnitude of allowed attitude deviation of estimate from expectation in
    degrees"""

    @ROS.setup_node(
        [
            ("pose_estimator_endpoint", ROS_D_POSE_ESTIMATOR_ENDPOINT, True),
            ("max_pitch", ROS_D_MISC_MAX_PITCH, False),
            ("min_match_altitude", ROS_D_MISC_MIN_MATCH_ALTITUDE, False),
            (
                "attitude_deviation_threshold",
                ROS_D_MISC_ATTITUDE_DEVIATION_THRESHOLD,
                False,
            ),
        ]
    )
    def __init__(self, name: str) -> None:
        """Node initializer

        :param name: Node name
        """
        # super().__init__(name)  # Handled by @setup_node decorator
        self._package_share_dir = get_package_share_directory("gisnav")

        # Converts image_raw to cv2 compatible image
        self._cv_bridge = CvBridge()

        # Calling these decorated properties the first time will setup
        # subscriptions to the appropriate ROS topics
        self.orthoimage_3d
        self.terrain_altitude
        self.terrain_geopoint
        self.altitude
        self.gimbal_quaternion
        self.geopose
        self.home_geopoint

    @property
    @ROS.subscribe(messaging.ROS_TOPIC_ORTHOIMAGE, QoSPresetProfiles.SENSOR_DATA.value)
    def orthoimage_3d(self) -> Optional[OrthoImage3D]:
        """Input orthoimage and elevation raster pair for pose estimation"""

    @property
    @ROS.max_delay_ms(_DELAY_NORMAL_MS)
    @ROS.subscribe(
        messaging.ROS_TOPIC_TERRAIN_ALTITUDE, QoSPresetProfiles.SENSOR_DATA.value
    )
    def terrain_altitude(self) -> Optional[Altitude]:
        """Altitude of terrain directly under vehicle, or None if unknown or too old"""

    @property
    @ROS.max_delay_ms(_DELAY_NORMAL_MS)
    @ROS.subscribe(
        messaging.ROS_TOPIC_TERRAIN_GEOPOINT,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def terrain_geopoint(self) -> Optional[GeoPointStamped]:
        """
        Vehicle ground track as :class:`geographic_msgs.msg.GeoPointStamped`
        message, or None if not available

        Complementary to the terrain Altitude message, includes lat and lon in
        atomic message
        """

    @property
    @ROS.max_delay_ms(_DELAY_NORMAL_MS)
    @ROS.subscribe(
        messaging.ROS_TOPIC_VEHICLE_ALTITUDE, QoSPresetProfiles.SENSOR_DATA.value
    )
    def altitude(self) -> Optional[Altitude]:
        """Altitude of vehicle, or None if unknown or too old"""

    @property
    @ROS.max_delay_ms(_DELAY_NORMAL_MS)
    @ROS.subscribe(
        messaging.ROS_TOPIC_GIMBAL_QUATERNION, QoSPresetProfiles.SENSOR_DATA.value
    )
    def gimbal_quaternion(self) -> Optional[Quaternion]:
        """Gimbal orientation as :class:`geometry_msgs.msg.Quaternion` message
        or None if not available
        """

    @property
    @ROS.max_delay_ms(_DELAY_NORMAL_MS)
    @ROS.subscribe(
        messaging.ROS_TOPIC_VEHICLE_GEOPOSE, QoSPresetProfiles.SENSOR_DATA.value
    )
    def geopose(self) -> Optional[GeoPoseStamped]:
        """Vehicle GeoPoseStamped, or None if not available or too old"""

    @property
    @ROS.max_delay_ms(_DELAY_SLOW_MS)
    @ROS.subscribe(
        messaging.ROS_TOPIC_HOME_GEOPOINT, QoSPresetProfiles.SENSOR_DATA.value
    )
    def home_geopoint(self) -> Optional[GeoPointStamped]:
        """Home position GeoPointStamped, or None if not available or too old"""

    @property
    @ROS.max_delay_ms(_DELAY_SLOW_MS)
    @ROS.subscribe(messaging.ROS_TOPIC_CAMERA_INFO, QoSPresetProfiles.SENSOR_DATA.value)
    def camera_info(self) -> Optional[CameraInfo]:
        """Camera info for determining appropriate :attr:`.orthoimage_3d` resolution"""

    def _image_callback(self, msg: Image) -> None:
        """
        Callback for :class:`sensor_msgs.msg.Image` messages

        :param msg: The latest :class:`sensor_msgs.msg.Image` message
        """

        @enforce_types(self.get_logger().warn, "Cannot validate received image")
        def _image_callback(img: np.ndarray, camera_info: CameraInfo):
            img_shape = img.shape[0:2]
            declared_shape = (camera_info.height, camera_info.width)
            assert img_shape == declared_shape, (
                f"Converted image shape {img_shape} did not match declared image "
                f"shape ({declared_shape})."
            )

            self.geopose_stamped_estimate
            self.altitude_estimate

        img = self._cv_bridge.imgmsg_to_cv2(
            msg, desired_encoding="passthrough"
        )  # self._IMAGE_ENCODING)
        _image_callback(img, self.camera_info)

    @property
    @ROS.max_delay_ms(_DELAY_FAST_MS)
    @ROS.subscribe(
        messaging.ROS_TOPIC_IMAGE,
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=_image_callback,
    )
    def image(self) -> Optional[Image]:
        """Raw image data from vehicle camera for pose estimation"""

    def _should_estimate_geopose(self) -> bool:
        """Determines whether :attr:`.geopose_stamped_estimate` should be called

        Match should be attempted if (1) a reference map has been retrieved,
        (2) camera roll or pitch is not too high (e.g. facing horizon instead
        of nadir), and (3) drone is not flying too low.

        :return: True if pose estimation be attempted
        """

        @enforce_types(self.get_logger().warn, "Cannot estimate pose")
        def _should_estimate(orthoimage_3d: OrthoImage3D, altitude: Altitude):
            # Check condition (2) - whether ca_vehicle_altitudemera roll/pitch is too large
            max_pitch = (
                self.get_parameter("max_pitch").get_parameter_value().integer_value
            )
            if self._camera_roll_or_pitch_too_high(max_pitch):
                self.get_logger().warn(
                    f"Camera roll or pitch not available or above limit {max_pitch}. "
                    f"Skipping pose estimation."
                )
                return False

            # Check condition (3) - whether vehicle altitude is too low
            min_alt = (
                self.get_parameter("min_match_altitude")
                .get_parameter_value()
                .integer_value
            )
            assert min_alt > 0
            if altitude.terrain is np.nan:
                self.get_logger().warn(
                    "Cannot determine altitude AGL, skipping map update."
                )
                return False
            if altitude.terrain < min_alt:
                self.get_logger().warn(
                    f"Assumed altitude {altitude.terrain} was lower "
                    f"than minimum threshold for matching ({min_alt}) or could not "
                    f"be determined. Skipping pose estimation."
                )
                return False

            return True

        return bool(_should_estimate(self.orthoimage_3d, self.altitude))

    @property
    @ROS.publish(
        messaging.ROS_TOPIC_VEHICLE_GEOPOSE_ESTIMATE,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    @cache_if(_should_estimate_geopose)
    def geopose_stamped_estimate(self) -> Optional[GeoPoseStamped]:
        """
        Vehicle estimated pose as :class:`geographic_msgs.msg.GeoPoseStamped`
        message or None if not available
        """

        class _PoseEstimationInputs(TypedDict):
            """Input data for pose estimation service call"""

            query: np.ndarray
            reference: np.ndarray
            elevation: np.ndarray  # elevation reference
            k: np.ndarray  # camera intrinsics of shape (3, 3)

        @dataclass(frozen=True)
        class _PoseEstimationContext:
            """
            Required context for post-processing an estimated
            :class:`geometry_msgs.msg.Pose` into a :class:`geographic_msgs.msg.GeoPose`
            """

            gimbal_quaternion: Quaternion  # for camera yaw
            altitude: Altitude  # self._altitude_scaling? Scaling can only be computed after pose estimation? should not be required here as its estimated from the FOV?

        @enforce_types(self.get_logger().warn, "Cannot get pose estimation context")
        def _get_context(gimbal_quaternion: Quaternion, altitude: Altitude):
            return _PoseEstimationContext(
                gimbal_quaternion=gimbal_quaternion, altitude=altitude
            )

        @enforce_types(
            self.get_logger().warn,
            "Cannot get transformation matrix from pixel to WGS84 coordinates",
        )
        def _get_geotransformation_matrix(pixel_coords, geo_coords):
            # Convert lists to numpy arrays
            pixel_coords = np.float32(pixel_coords)
            geo_coords = np.float32(geo_coords)

            # Compute transformation matrix
            M = cv2.getPerspectiveTransform(pixel_coords, geo_coords)

            return M

        @enforce_types(
            self.get_logger().warn, "Cannot preprocess pose estimation inputs"
        )
        def _pre_process_inputs(
            image: Image, orthoimage: OrthoImage3D, camera_info: CameraInfo
        ) -> _PoseEstimationInputs:
            """Rotate and crop and orthoimage stack to align with query image"""

            def _get_rotation_matrix(image: np.ndarray, degrees: float) -> np.ndarray:
                height, width = image.shape[:2]
                cx, cy = width // 2, height // 2
                r = cv2.getRotationMatrix2D((cx, cy), degrees, 1.0)
                return r

            def _get_translation_matrix(dx, dy):
                t = np.float32([[1, 0, dx], [0, 1, dy]])
                return t

            def _get_affine_matrix(
                image: np.ndarray, degrees: float, crop_height: int, crop_width: int
            ):
                """
                Creates affine transformation that rotates around center and then
                center-crops an image
                """
                r = _get_rotation_matrix(image, degrees)
                dx = (image.shape[1] - crop_width) // 2
                dy = (image.shape[0] - crop_height) // 2
                t = _get_translation_matrix(dx, dy)

                # Combine the rotation and translation to get the final affine transformation matrix
                affine_matrix = np.dot(t, np.vstack([r, [0, 0, 1]]))
                return affine_matrix

            def _rotate_and_crop_image(
                image: np.ndarray, degrees: float, shape: Tuple[int, int]
            ) -> Tuple[np.ndarray, np.ndarray]:
                """
                Rotates around center and then center-crops image

                :return: Tuple of rotated and cropped image, and used affine
                    transformation matrix
                """
                # Image can have any number of channels
                affine = _get_affine_matrix(image, degrees, *crop_shape)
                return cv2.warpAffine(image, affine[:2, :], crop_shape), affine

            def _get_yaw_pitch_degrees_from_quaternion(
                quaternion,
            ) -> Tuple[float, float]:
                """
                To avoid gimbal lock when facing nadir (pitch -90 degrees in NED),
                assume roll is zero (stabilized gimbal)
                """
                # Unpack quaternion
                x = quaternion.x
                y = quaternion.y
                z = quaternion.z
                w = quaternion.w

                # Calculate yaw and pitch directly from the quaternion to
                # avoid gimbal lock. Assumption: roll is zero.
                yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
                pitch = np.arcsin(2.0 * (w * y - z * x))

                # Convert yaw and pitch from radians to degrees
                yaw_degrees = yaw * 180.0 / np.pi
                pitch_degrees = pitch * 180.0 / np.pi

                return yaw_degrees, pitch_degrees

            query_array = self._cv_bridge.imgmsg_to_cv2(
                image, desired_encoding="passthrough"
            )
            orthoimage = self._cv_bridge.imgmsg_to_cv2(
                orthoimage.img, desired_encoding="passthrough"
            )
            dem = self._cv_bridge.imgmsg_to_cv2(
                orthoimage.dem, desired_encoding="passthrough"
            )

            # Rotate and crop orthoimage stack
            yaw_degrees, _ = _get_yaw_pitch_degrees_from_quaternion(
                context.gimbal_quaternion
            )
            crop_shape = query_array.shape[0:2]
            orthoimage_stack = np.dstack((orthoimage, dem))
            orthoimage_stack, affine = _rotate_and_crop_image(
                orthoimage_stack, yaw_degrees, *crop_shape
            )

            reference_array = orthoimage_stack[:, :, 0:2]
            elevation_array = orthoimage_stack[:, :, 3]

            pre_processed_inputs: _PoseEstimationInputs = {
                "query": query_array,
                "reference": reference_array,
                "elevation": elevation_array,
                "k": camera_info.k.reshape((3, 3)),
            }
            return pre_processed_inputs

        context: _get_context(self.gimbal_quaternion, self.altitude)
        inputs = _pre_process_inputs(
            self.image,
            self.orthoimage_3d,
            self.camera_info,
        )

        # TODO: timeout, connection errors, exceptions etc.
        pose_estimator_endpoint = (
            self.get_parameter("pose_estimator_endpoint")
            .get_parameter_value()
            .string_value
        )
        r = requests.post(
            pose_estimator_endpoint,
            data=inputs,
        )

        if r.status_code == 200:
            # TODO: should return None if the length of these is 0?
            data = json.loads(r.text)
            if "r" in data and "t" in data:
                pose = np.asarray(data.get("r")), np.asarray(data.get("t"))
            else:
                self.get_logger().warn(
                    f"Could not estimate pose, returned text {r.text}"
                )
                return None
        else:
            self.get_logger().warn(
                f"Could not estimate pose, status code {r.status_code}"
            )
            return None
        # endregion pose estimation request

        @enforce_types(self.get_logger().warn, "Cannot post process output")
        def _post_process_outputs(pose: Pose) -> GeoPose:
            def translate_keypoints(keypoints, dx, dy):
                translated_keypoints = []
                for x, y in keypoints:
                    translated_keypoints.append((x + dx, y + dy))
                return translated_keypoints

            def rotate_keypoints(keypoints, degrees, cx, cy):
                degrees = -degrees  # Invert angle for reverse rotation
                radians = math.radians(degrees)
                cos = math.cos(radians)
                sin = math.sin(radians)
                rotated_keypoints = []
                for x, y in keypoints:
                    x_rot = (x - cx) * cos - (y - cy) * sin + cx
                    y_rot = (x - cx) * sin + (y - cy) * cos + cy
                    rotated_keypoints.append((x_rot, y_rot))
                return rotated_keypoints

            # Assume you have a list of keypoints in the rotated and cropped stack
            keypoints_rotated_cropped = [(50, 50), (25, 25)]

            # Translate keypoints back to the rotated (uncropped) image
            dy, dx = (height - crop_height) // 2, (width - crop_width) // 2
            keypoints_rotated = translate_keypoints(keypoints_rotated_cropped, dx, dy)

            # Rotate keypoints back to the original image
            cx, cy = width // 2, height // 2
            rotate_keypoints(keypoints_rotated, angle, cx, cy)

            # TODO: keypoints_original to geopose

        if pose is not None:
            pose_ = Pose(*pose)
            position = self._post_process_pose(pose_, image_pair)
        else:
            self.get_logger().warn("Could not estimate a pose, skipping this frame.")
            return None

        return GeoPoseStamped(
            header=messaging.create_header("base_link"),
            pose=GeoPose(
                position=GeoPoint(
                    latitude=position.lat,
                    longitude=position.lon,
                    altitude=position.altitude.ellipsoid,
                ),
                orientation=messaging.as_ros_quaternion(position.attitude.q),
            ),
        )

    @property
    @ROS.publish(
        messaging.ROS_TOPIC_VEHICLE_ALTITUDE_ESTIMATE,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def altitude_estimate(self) -> Optional[Altitude]:
        """Altitude estimate of vehicle, or None if unknown or too old"""
        return Altitude(
            header=messaging.create_header("base_link"),
            amsl=position.altitude.amsl,
            local=position.altitude.home,
            relative=position.altitude.home,
            terrain=position.altitude.agl,
            bottom_clearance=position.altitude.agl,
        )

    # @property
    # def _altitude_scaling(self) -> Optional[float]:
    #    """Returns camera focal length divided by camera altitude in meters"""
    #
    #    @enforce_types(self.get_logger().warn, "Cannot determine altitude scaling")
    #    def _altitude_scaling(camera_info: CameraInfo, altitude: Altitude):
    #        return camera_info.k[0] / altitude.terrain  # TODO: assumes fx == fy
    #
    #    return _altitude_scaling(self.camera_info, self.altitude)

    @property
    def _r_guess(self) -> Optional[np.ndarray]:
        """Gimbal rotation matrix guess"""

        @enforce_types(self.get_logger().warn, "Cannot determine rotation matrix guess")
        def _r_guess(self, gimbal_quaternion) -> np.ndarray:
            gimbal_attitude = Attitude(
                q=messaging.as_np_quaternion(self.gimbal_quaternion)
            )
            gimbal_attitude = (
                gimbal_attitude.to_esd()
            )  # Need coordinates in image frame, not NED
            return gimbal_attitude.r

        return _r_guess(self.gimbal_quaternion)

    @property
    def _contextual_map_data(self) -> Optional[ContextualMapData]:
        """Returns contextual (rotated) map data for pose estimation

        :return: Rotated map with associated metadata, or None if not available
        """
        # Get cropped and rotated map
        if self._gimbal_quaternion is not None:
            gimbal_attitude = Attitude(
                q=messaging.as_np_quaternion(self._gimbal_quaternion), extrinsic=True
            )
            roll = gimbal_attitude.roll
            camera_yaw = gimbal_attitude.yaw
            if abs(roll) > np.pi / 2:
                # Assume camera pitch (NED frame) is lower than -90 degrees
                # A naive interpretation of the Euler angles would assume the
                # camera (and the vehicle) is upside down cause the map being
                # rotated incorrectly by 180 degrees compared to what is needed
                # for pose estimation This might happen e.g. if gimbal has been
                # pointing straight down nadir and vehicle body has suddenly
                # gained additional negative pitch before gimbal has had time
                # to stabilize
                self.get_logger().debug(
                    "Gimbal absolute Euler roll angle over 90 degrees: assuming "
                    "gimbal is not upside down and that gimbal absolute pitch "
                    "is over 90 degrees instead."
                )
                camera_yaw = (camera_yaw + np.pi / 2) % (2 * np.pi)
            assert_type(camera_yaw, float)
            assert (
                -2 * np.pi <= camera_yaw <= 2 * np.pi
            ), f"Unexpected gimbal yaw value: {camera_yaw} ([-2*pi, 2*pi] expected)."
        else:
            self.get_logger().warn("Camera yaw unknown, cannot estimate pose.")
            return None

        return ContextualMapData(
            rotation=camera_yaw,
            map_data=self._map_data,
            crop=self.img_dim,
            altitude_scaling=self._altitude_scaling,
        )

    def _post_process_pose(self, pose: Pose, image_pair: ImagePair) -> Position:
        """Handles estimated pose

        :param pose: Pose result from pose estimation node worker
        :param image_pair: Image pair input from which pose was estimated
        """
        try:
            # Compute DEM value at estimated position
            # This is in camera intrinsic (pixel) units with origin at whatever
            # the DEM uses. For example, the USGS DEM uses NAVD 88.
            x, y = -pose.t.squeeze()[0:2]
            x, y = int(x), int(y)
            elevation = image_pair.ref.elevation.arr[y, x]
            pose = Pose(pose.r, pose.t - elevation)
        except DataValueError:
            self.get_logger().warn("Estimated pose was not valid, skipping this frame.")
            return None
        except IndexError:
            # TODO: might be able to handle this
            self.get_logger().warn("Estimated pose was not valid, skipping this frame.")
            return None

        try:
            assert self._terrain_geopoint is not None
            assert self._terrain_altitude is not None
            fixed_camera = FixedCamera(
                pose=pose,
                image_pair=image_pair,
                terrain_altitude_amsl=self._terrain_altitude.amsl,
                terrain_altitude_ellipsoid=self._terrain_geopoint.position.altitude,
                home_position=self._home_geopoint.position,
                timestamp=image_pair.qry.timestamp,
            )
        except DataValueError:
            self.get_logger().warn(
                "Could not estimate a valid camera position, skipping this frame."
            )
            return None

        if not self._is_valid_estimate(fixed_camera, self._r_guess):
            self.get_logger().warn(
                "Estimate did not pass post-estimation validity check, "
                "skipping this frame."
            )
            return None

        assert fixed_camera is not None
        # noinspection PyUnreachableCode
        if __debug__:
            # Visualize projected FOV estimate
            fov_pix = fixed_camera.fov.fov_pix
            ref_img = fixed_camera.image_pair.ref.image.arr
            map_with_fov = cv2.polylines(
                ref_img.copy(), [np.int32(fov_pix)], True, 255, 3, cv2.LINE_AA
            )

            img = np.vstack((map_with_fov, fixed_camera.image_pair.qry.image.arr))
            cv2.imshow("Projected FOV", img)
            cv2.waitKey(1)

        # self.publish(fixed_camera.position)
        return fixed_camera.position

    def _is_valid_estimate(
        self, fixed_camera: FixedCamera, r_guess_: np.ndarray
    ) -> bool:
        """Returns True if the estimate is valid

        Compares computed estimate to guess based on earlier gimbal attitude.
        This will reject estimates made when the gimbal was not stable (which
        is strictly not necessary) if gimbal attitude is based on set attitude
        and not actual attitude, which is assumed to filter out more inaccurate
        estimates.
        """
        if r_guess_ is None:
            self.get_logger().warn(
                "Reference gimbal attitude (r_guess) was not available, cannot "
                "do post-estimation validity check."
            )
            return False

        r_guess = Rotation.from_matrix(r_guess_)
        # Adjust for map rotation
        camera_yaw = fixed_camera.image_pair.ref.rotation
        camera_yaw = Rotation.from_euler("xyz", [0, 0, camera_yaw], degrees=False)
        r_guess *= camera_yaw

        r_estimate = Rotation.from_matrix(fixed_camera.pose.r)

        magnitude = Rotation.magnitude(r_estimate * r_guess.inv())

        threshold = (
            self.get_parameter("attitude_deviation_threshold")
            .get_parameter_value()
            .integer_value
        )
        threshold = np.radians(threshold)

        if magnitude > threshold:
            self.get_logger().warn(
                f"Estimated rotation difference to expected was too high (magnitude "
                f"{np.degrees(magnitude)})."
            )
            return False

        return True

    @staticmethod
    def off_nadir_angle(q):
        # Rotated vector
        rotated_x = 2.0 * (q.x * q.z - q.w * q.y)
        rotated_y = 2.0 * (q.y * q.z + q.w * q.x)
        rotated_z = q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z

        # Down direction
        down_x, down_y, down_z = 0.0, 0.0, -1.0

        # Dot product of rotated vector and down direction
        dot_product = rotated_x * down_x + rotated_y * down_y + rotated_z * down_z

        # Clamp dot_product to avoid floating-point precision issues
        dot_product = max(min(dot_product, 1.0), -1.0)

        # Compute the angle between the rotated vector and down direction
        angle_rad = math.acos(dot_product)

        # Convert the angle to degrees
        angle_deg = math.degrees(angle_rad)

        return angle_deg

    @staticmethod
    def _euler_from_quaternion(q):
        # Convert quaternion to euler angles
        t0 = 2.0 * (q.w * q.x + q.y * q.z)
        t1 = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(t0, t1)

        t2 = 2.0 * (q.w * q.y - q.z * q.x)
        t2 = 1.0 if t2 > 1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    def _camera_roll_or_pitch_too_high(self, max_pitch: Union[int, float]) -> bool:
        """Returns True if (set) camera roll or pitch exceeds given limit OR
        camera pitch is unknown

        Used to determine whether camera roll or pitch is too high up from
        nadir to make matching against a map worthwhile.

        :param max_pitch: The limit for the pitch in degrees from nadir over
            which it will be considered too high
        :return: True if pitch is too high
        """
        assert_type(max_pitch, get_args(Union[int, float]))
        pitch = None
        if self._gimbal_quaternion is not None:
            off_nadir_deg = self.off_nadir_angle(self._gimbal_quaternion) - 90
            self.get_logger().debug(
                f"Camera pitch is {off_nadir_deg} degrees off nadir"
            )
            roll, pitch, yaw = self._euler_from_quaternion(self._gimbal_quaternion)

            if off_nadir_deg > max_pitch:
                self.get_logger().warn(
                    f"Camera pitch {pitch} is above limit {max_pitch}."
                )
                return True
            else:
                return False
        else:
            self.get_logger().warn(
                "Gimbal attitude was not available, assuming camera pitch too high."
            )
            return True

    def publish(self, position: Position) -> None:
        """Publishes estimated position over ROS topic

        :param position: Estimated position
        """
        altitude_msg = Altitude(
            header=messaging.create_header("base_link"),
            amsl=position.altitude.amsl,
            local=position.altitude.home,
            relative=position.altitude.home,
            terrain=position.altitude.agl,
            bottom_clearance=position.altitude.agl,
        )
        self._altitude_pub.publish(altitude_msg)

        geopose_msg = GeoPoseStamped(
            header=messaging.create_header("base_link"),
            pose=GeoPose(
                position=GeoPoint(
                    latitude=position.lat,
                    longitude=position.lon,
                    altitude=position.altitude.ellipsoid,
                ),
                orientation=messaging.as_ros_quaternion(position.attitude.q),
            ),
        )
        self._geopose_pub.publish(geopose_msg)
