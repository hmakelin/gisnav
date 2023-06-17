"""Module that contains the pose estimation node"""
import json
import math
import pickle
from dataclasses import dataclass
from typing import Final, List, Optional, Tuple, TypedDict, Union, get_args

import cv2
import numpy as np
import requests
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geographic_msgs.msg import (
    BoundingBox,
    GeoPoint,
    GeoPointStamped,
    GeoPose,
    GeoPoseStamped,
)
from geometry_msgs.msg import Pose, Quaternion
from mavros_msgs.msg import Altitude
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import CameraInfo, Image

from gisnav_msgs.msg import OrthoImage3D  # type: ignore

from .. import messaging
from .._assertions import ROS, assert_type, cache_if, narrow_types
from .._data import Attitude, create_src_corners
from ..static_configuration import (
    GIS_NODE_NAME,
    ROS_NAMESPACE,
    ROS_TOPIC_RELATIVE_GROUND_TRACK_ELEVATION,
    ROS_TOPIC_RELATIVE_GROUND_TRACK_GEOPOSE,
    ROS_TOPIC_RELATIVE_ORTHOIMAGE,
    ROS_TOPIC_RELATIVE_VEHICLE_ALTITUDE,
    ROS_TOPIC_RELATIVE_VEHICLE_GEOPOSE,
)


class CVNode(Node):
    """Estimates and publishes pose between two images

    Compares images from :class:`sensor_msgs.msg.Image` message to maps from
    :class:`gisnav_msgs.msg.OrthoImage3D` message to estimate
    :class:`geographic_msgs.msg.GeoPoseStamped`.
    """

    class PoseEstimationInputs(TypedDict, total=True):
        """Input data for pose estimation service call"""

        query: np.ndarray
        reference: np.ndarray
        elevation: np.ndarray  # elevation reference
        k: np.ndarray  # camera intrinsics of shape (3, 3)

    @dataclass(frozen=True)
    class _PoseEstimationIntermediateOutputs:
        """Data generated by pre-processing that is useful for post-processing

        :ivar affine_transform: Transformation from orthoimage to rotated &
            cropped (aligned with video image) frame
        :ivar camera_yaw_degrees: Camera yaw in degrees in NED frame
        """

        affine_transform: np.ndarray
        camera_yaw_degrees: float

    @dataclass(frozen=True)
    class _PoseEstimationContext:
        """
        Required context for post-processing an estimated
        :class:`geometry_msgs.msg.Pose` into a :class:`geographic_msgs.msg.GeoPose`
        that should be frozen at same time as inputs.

        :ivar orthoimage: Orthoimage used for matching. In post-processing
            this is required to get a geotransformation matrix from the
            orthoimage pixel frame to WGS 84 coordinates. It is lazily
            evaluated in post-processing even though it could already be
            computed earlier - all required information is contained in the
            orthoimage itself.
        :ivar gimbal_quaternion: Qimbal quaternion in NED frame. The pose estimation
            is done against a rotated orthoimage and this is needed to get the
            pose in the original coordinate frame.
        :ivar terrain_altitude: Terrain or vehicle ground track Altitude.
            The pose estimation estimates above-ground (AGL) altitude, and
            context of terrain altitude is  required to generate other types
            of altitude such as ASML.
        :ivar terrain_geopoint: Terrain or vehicle ground track GeoPoint.
            Required to generate GeoPoint altitude (WGS 84 ellipsoid) since
            it is not included in the Altitude message.
        """

        orthoimage: OrthoImage3D
        gimbal_quaternion: Quaternion
        terrain_altitude: Altitude
        terrain_geopoint: GeoPointStamped

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

    _ROS_PARAM_DESCRIPTOR_READ_ONLY: Final = ParameterDescriptor(read_only=True)
    """A read only ROS parameter descriptor"""

    def __init__(self, *args, **kwargs) -> None:
        """Class initializer

        :param args: Positional arguments to parent :class:`.Node` constructor
        :param kwargs: Keyword arguments to parent :class:`.Node` constructor
        """
        super().__init__(*args, **kwargs)

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
        self.camera_info
        self.image

    @property
    @ROS.parameter(
        ROS_D_POSE_ESTIMATOR_ENDPOINT, descriptor=_ROS_PARAM_DESCRIPTOR_READ_ONLY
    )
    def pose_estimator_endpoint(self) -> Optional[str]:
        """Pose estimation service endpoint URL"""

    @property
    @ROS.parameter(ROS_D_MISC_MAX_PITCH)
    def max_pitch(self) -> Optional[int]:
        """Max :term:`camera` pitch in degrees from :term:`nadir` beyond which
        :term:`pose` estimation will not be attempted
        """

    @property
    @ROS.parameter(ROS_D_MISC_MIN_MATCH_ALTITUDE)
    def min_match_altitude(self) -> Optional[int]:
        """Minimum :term:`vehicle` :term:`altitude` in meters :term:`AGL` below which
        :term:`pose` estimation will not be attempted
        """

    @property
    @ROS.parameter(ROS_D_MISC_ATTITUDE_DEVIATION_THRESHOLD)
    def attitude_deviation_threshold(self) -> Optional[float]:
        """Maximum allowed attitude deviation in degrees of :term:`pose` estimate
        from expectation for the estimate to be considered valid
        """

    @property
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_ORTHOIMAGE.replace("~", GIS_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def orthoimage_3d(self) -> Optional[OrthoImage3D]:
        """Input orthoimage and elevation raster pair for pose estimation"""

    @property
    @ROS.max_delay_ms(_DELAY_NORMAL_MS)
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_GROUND_TRACK_ELEVATION.replace("~", GIS_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def terrain_altitude(self) -> Optional[Altitude]:
        """Altitude of terrain directly under vehicle, or None if unknown or too old"""

    @property
    @ROS.max_delay_ms(_DELAY_NORMAL_MS)
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_GROUND_TRACK_GEOPOSE.replace("~", GIS_NODE_NAME)}',
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
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_VEHICLE_ALTITUDE.replace("~", GIS_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def altitude(self) -> Optional[Altitude]:
        """Altitude of vehicle, or None if unknown or too old"""

    @property
    # @ROS.max_delay_ms(_DELAY_NORMAL_MS)
    @ROS.subscribe(
        messaging.ROS_TOPIC_GIMBAL_QUATERNION, QoSPresetProfiles.SENSOR_DATA.value
    )
    def gimbal_quaternion(self) -> Optional[Quaternion]:
        """Gimbal orientation as :class:`geometry_msgs.msg.Quaternion` message
        or None if not available
        """

    @property
    # @ROS.max_delay_ms(_DELAY_NORMAL_MS)  # TODO: re-enable
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_VEHICLE_GEOPOSE.replace("~", GIS_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def geopose(self) -> Optional[GeoPoseStamped]:
        """Vehicle GeoPoseStamped, or None if not available or too old"""

    @property
    # @ROS.max_delay_ms(_DELAY_SLOW_MS) - gst plugin config does not enable timestamp?
    @ROS.subscribe(messaging.ROS_TOPIC_CAMERA_INFO, QoSPresetProfiles.SENSOR_DATA.value)
    def camera_info(self) -> Optional[CameraInfo]:
        """Camera info for determining appropriate :attr:`.orthoimage_3d` resolution"""

    def _image_callback(self, msg: Image) -> None:
        """
        Callback for :class:`sensor_msgs.msg.Image` messages

        :param msg: The latest :class:`sensor_msgs.msg.Image` message
        """

        @narrow_types(self)
        def _image_callback(img: np.ndarray, camera_info: CameraInfo):
            img_shape = img.shape[0:2]
            declared_shape = (camera_info.height, camera_info.width)
            if not img_shape == declared_shape:
                self.get_logger().error(
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
    # @ROS.max_delay_ms(_DELAY_FAST_MS) - gst plugin config does not enable timestamp?
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

        @narrow_types(self)
        def _should_estimate(altitude: Altitude, max_pitch: int, min_alt: int):
            # Check condition (2) - whether camera roll/pitch is too large
            if self._camera_roll_or_pitch_too_high(max_pitch):
                self.get_logger().warn(
                    f"Camera roll or pitch not available or above limit {max_pitch}. "
                    f"Skipping pose estimation."
                )
                return False

            # Check condition (3) - whether vehicle altitude is too low
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

        return bool(
            _should_estimate(self.altitude, self.max_pitch, self.min_match_altitude)
        )

    @staticmethod
    def _get_yaw_pitch_degrees_from_quaternion(
        quaternion,
    ) -> Tuple[float, float]:
        """
        To avoid gimbal lock when facing nadir (pitch -90 degrees in NED),
        assumes roll is close to zero (i.e roll can be slightly non-zero).
        """
        # Unpack quaternion
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        # Calculate yaw and pitch directly from the quaternion to
        # avoid gimbal lock. Assumption/constraint: roll is close to zero.
        yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        pitch = np.arcsin(2.0 * (w * y - z * x))

        # Convert yaw and pitch from radians to degrees
        yaw_degrees = yaw * 180.0 / np.pi
        pitch_degrees = pitch * 180.0 / np.pi

        return yaw_degrees, pitch_degrees

    @narrow_types
    def _preprocess_geopose_inputs(
        self,
        image: Image,
        orthoimage: OrthoImage3D,
        camera_info: CameraInfo,
        context: _PoseEstimationContext,
    ) -> Optional[Tuple[PoseEstimationInputs, _PoseEstimationIntermediateOutputs]]:
        """Rotate and crop and orthoimage stack to align with query image"""

        query_array = self._cv_bridge.imgmsg_to_cv2(
            image, desired_encoding="passthrough"
        )
        orthophoto = self._cv_bridge.imgmsg_to_cv2(
            orthoimage.img, desired_encoding="passthrough"
        )
        dem = self._cv_bridge.imgmsg_to_cv2(
            orthoimage.dem, desired_encoding="passthrough"
        )

        # Rotate and crop orthoimage stack
        camera_yaw_degrees, _ = self._get_yaw_pitch_degrees_from_quaternion(
            context.gimbal_quaternion
        )
        crop_shape: Tuple[int, int] = query_array.shape[0:2]
        orthoimage_stack = np.dstack((orthophoto, dem))
        orthoimage_stack, affine = self._rotate_and_crop_image(
            orthoimage_stack, camera_yaw_degrees, crop_shape
        )
        reference_array = orthoimage_stack[:, :, 0:3]
        elevation_array = orthoimage_stack[:, :, 3]

        pre_processed_inputs: CVNode.PoseEstimationInputs = {
            "query": query_array,
            "reference": reference_array,
            "elevation": elevation_array,
            "k": camera_info.k.reshape((3, 3)),
        }
        intermediate_outputs = CVNode._PoseEstimationIntermediateOutputs(
            affine_transform=affine, camera_yaw_degrees=camera_yaw_degrees
        )
        return pre_processed_inputs, intermediate_outputs

    # TODO: fix and re-enable
    @narrow_types
    def _is_valid_pose_estimate(
        self,
        pose: Tuple[np.ndarray, np.ndarray],
        context: _PoseEstimationContext,
        intermediate_outputs: _PoseEstimationIntermediateOutputs,
        threshold: int,
    ):
        """Returns True if the estimate is valid

        Compares computed estimate to guess based on earlier gimbal attitude.
        This will reject estimates made when the gimbal was not stable (which
        is strictly not necessary) if gimbal attitude is based on set attitude
        and not actual attitude, which is assumed to filter out more inaccurate
        estimates.
        """
        yaw, pitch = self._get_yaw_pitch_degrees_from_quaternion(
            context.gimbal_quaternion
        )

        # yaw, pitch = self._get_yaw_pitch_degrees_from_quaternion(pose.orientation)
        # self.get_logger().error(f"raw pose yaw pitch {yaw} {pitch}")

        r_guess = Rotation.from_matrix(
            messaging.quaternion_to_rotation_matrix(context.gimbal_quaternion)
        )
        r, t = pose

        rot_90_Z = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])

        r_estimate = (
            self._seu_to_ned_matrix
            @ np.linalg.inv(intermediate_outputs.affine_transform)
            @ np.linalg.inv(rot_90_Z)
            @ r.T
            # @ np.linalg.inv(messaging.quaternion_to_rotation_matrix(
            # pose.orientation))
        )
        # r_estimate = Rotation.from_matrix(np.transpose(r, (1,0)))

        # self.get_logger().error(f"{r_estimate.shape} {np.matmul(-r.T, t)}")
        r_estimate = Rotation.from_matrix(r_estimate)
        yaw, pitch = self._get_yaw_pitch_degrees_from_quaternion(
            messaging.as_ros_quaternion(Rotation.as_quat(r_estimate))
        )

        magnitude = Rotation.magnitude(r_estimate * r_guess.inv())

        threshold = np.radians(threshold)

        if magnitude > threshold:
            self.get_logger().warn(
                f"Estimated rotation difference to expected was too high "
                f"(magnitude {np.degrees(magnitude)})."
            )
            return False

        return True

    @narrow_types
    def _post_process_pose(
        self,
        inputs,
        pose: Tuple[np.ndarray, np.ndarray],
        intermediate_outputs: _PoseEstimationIntermediateOutputs,
        context: _PoseEstimationContext,
    ) -> Optional[Tuple[GeoPoint, Altitude, Quaternion]]:
        """
        Post process estimated pose to vehicle GeoPoint, Altitude and gimbal
        Quaternion estimates

        Estimates camera GeoPoint (WGS84 coordinates + altitude in meters
        above mean sea level (AMSL) and ground level (AGL).
        """
        geotransform = self._get_geotransformation_matrix(context.orthoimage)
        r, t = pose
        t_world = np.matmul(r.T, -t)
        t_world_homogenous = np.vstack((t_world, [1]))
        try:
            t_unrotated_uncropped = (
                np.linalg.inv(intermediate_outputs.affine_transform)
                @ t_world_homogenous  # t
            )
        except np.linalg.LinAlgError as _:  # noqa: F841
            self.get_logger().warn(
                "Rotation and cropping was non-invertible, cannot compute "
                "GeoPoint and Altitude"
            )
            return None

        # ESD (cv2 x is width) to SEU (numpy array y is south) (x y might
        # be flipped because cv2)
        t_unrotated_uncropped = np.array(
            (
                t_unrotated_uncropped[1],
                t_unrotated_uncropped[0],
                -t_unrotated_uncropped[2],
                t_unrotated_uncropped[3],
            )
        )
        t_wgs84 = geotransform @ t_unrotated_uncropped
        lat, lon = t_wgs84.squeeze()[1::-1]
        alt = float(t_wgs84[2])

        altitude = Altitude(
            monotonic=0.0,  # TODO
            amsl=alt + context.terrain_altitude.amsl,
            local=0.0,  # TODO
            relative=0.0,  # TODO
            terrain=alt,
            bottom_clearance=alt,
        )
        geopoint = GeoPoint(
            altitude=alt + context.terrain_geopoint.position.altitude,
            latitude=lat,
            longitude=lon,
        )

        if geopoint is not None and altitude is not None:
            r, t = pose
            # r = messaging.quaternion_to_rotation_matrix(pose.orientation)

            # Rotation matrix is assumed to be in cv2.solvePnPRansac world
            # coordinate system (SEU axes), need to convert to NED axes after
            # reverting rotation and cropping
            try:
                r = (
                    self._seu_to_ned_matrix
                    @ np.linalg.inv(intermediate_outputs.affine_transform[:3, :3])
                    @ r.T  # @ -t
                )
            except np.linalg.LinAlgError as _:  # noqa: F841
                self.get_logger().warn(
                    "Cropping and rotation was non-invertible, canot estimate "
                    "GeoPoint and Altitude."
                )
                return None

            quaternion = messaging.rotation_matrix_to_quaternion(r)
            return geopoint, altitude, quaternion
        else:
            return None

    # @property
    # def _esu_to_ned_matrix(self):
    #    """Transforms from ESU to NED axes"""
    #    transformation_matrix = np.array(
    #        [[0, 1, 0], [1, 0, 0], [0, 0, -1]]  # E->N  # S->E  # U->D
    #    )
    #    return transformation_matrix

    @property
    def _seu_to_ned_matrix(self):
        """Transforms from ESU to NED axes"""
        transformation_matrix = np.array(
            [[-1, 0, 0], [0, 1, 0], [0, 0, -1]]  # S->N  # E->E  # U->D
        )
        return transformation_matrix

    @property
    @ROS.publish(
        messaging.ROS_TOPIC_VEHICLE_GEOPOSE_ESTIMATE,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def geopose_stamped_estimate(self) -> Optional[GeoPoseStamped]:
        estimate = self._geopose_stamped_estimate
        if estimate is not None:
            return estimate[0]
        else:
            return estimate

    @property
    @cache_if(_should_estimate_geopose)
    def _geopose_stamped_estimate(self) -> Optional[GeoPoseStamped]:
        """
        Vehicle estimated pose as :class:`geographic_msgs.msg.GeoPoseStamped`
        message or None if not available

        :return: GeoPoseStamped and Altitude tuple
        """

        context = self._pose_estimation_context
        results = self._preprocess_geopose_inputs(
            self.image, self.orthoimage_3d, self.camera_info, context
        )
        if not results:
            self.get_logger().warn(
                "Could not complete pre-processing for pose estimation"
            )
            return None

        inputs, intermediate_outputs = results
        pose = self._get_pose(inputs, self.pose_estimator_endpoint)

        # if not self._is_valid_pose_estimate(
        #        pose,
        #        context,
        #        intermediate_outputs,
        #        self.attitude_deviation_threshold
        # ):
        #    self.get_logger().warn(
        #        "Pose estimate did not pass post-processing validity check, "
        #        "skipping this frame."
        #    )
        #    return None

        post_processed_pose = self._post_process_pose(
            inputs, pose, intermediate_outputs, context
        )

        if post_processed_pose:
            (
                geopoint,
                altitude,
                gimbal_quaternion,
            ) = post_processed_pose  # TODO: uaternion might be in ESD and not NED

            return (
                GeoPoseStamped(
                    header=messaging.create_header("base_link"),
                    pose=GeoPose(
                        position=geopoint,
                        orientation=gimbal_quaternion,
                    ),
                ),
                altitude,
            )
        else:
            self.get_logger().warn(
                "Could not complete post-processing for pose estimation"
            )
            return None

    def _estimate_attitude(self, quaternion: Quaternion) -> Attitude:
        """Estimates gimbal (not vehicle) attitude in NED frame

        .. note::
            Stabilized gimbal *actual* (not set) attitude relative to vehicle
            body frame not always known so it is currently not computed.

        :return: Gimbal attitude in NED frame
        """
        # Convert estimated rotation to attitude quaternion for publishing
        # rT = r.T
        # assert not np.isnan(rT).any()
        gimbal_estimated_attitude = Rotation.from_quat(
            quaternion
        )  # rotated map pixel frame

        gimbal_estimated_attitude *= Rotation.from_rotvec(
            self.image_pair.ref.rotation * np.array([0, 0, 1])
        )  # unrotated map pixel frame

        # Re-arrange axes from unrotated (original) map pixel frame to NED frame
        rotvec = gimbal_estimated_attitude.as_rotvec()
        gimbal_estimated_attitude = Rotation.from_rotvec(
            [-rotvec[1], rotvec[0], rotvec[2]]
        )

        return Attitude(gimbal_estimated_attitude.as_quat())

    @property
    @ROS.publish(
        messaging.ROS_TOPIC_VEHICLE_ALTITUDE_ESTIMATE,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def altitude_estimate(self) -> Optional[Altitude]:
        """Altitude estimate of vehicle, or None if unknown or too old"""
        estimate = self._geopose_stamped_estimate
        if estimate is not None:
            return estimate[1]
        else:
            return estimate

    @property
    def _pose_estimation_context(self) -> Optional[_PoseEstimationContext]:
        @narrow_types(self)
        def _pose_estimation_context(
            orthoimage: OrthoImage3D,
            gimbal_quaternion: Quaternion,
            terrain_altitude: Altitude,
            terrain_geopoint: GeoPointStamped,
        ):
            return self._PoseEstimationContext(
                orthoimage=orthoimage,
                gimbal_quaternion=gimbal_quaternion,
                terrain_altitude=terrain_altitude,
                terrain_geopoint=terrain_geopoint,
            )

        return _pose_estimation_context(
            self.orthoimage_3d,
            self.gimbal_quaternion,
            self.terrain_altitude,
            self.terrain_geopoint,
        )

    @staticmethod
    # @lru_cache(1) - TODO cache using custom decorator or e.g. cachetools
    def _boundingbox_to_geo_coords(
        bounding_box: BoundingBox,
    ) -> List[Tuple[float, float]]:
        """
        Extracts the geo coordinates from a ROS
            geographic_msgs/BoundingBox and returns them as a list
            of tuples.

        Returns corners in order: top-left, bottom-left, bottom-right,
        top-right.

        Cached because it is assumed the same OrthoImage3D BoundingBox will
        be used for multiple matches.

        :param bbox: (geographic_msgs/BoundingBox): The bounding box.

        :return: The geo coordinates as a list of (longitude, latitude) tuples.
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

    @classmethod
    # @lru_cache(1)  # TODO: cache this use predicate decorator to update
    def _get_geotransformation_matrix(cls, orthoimage: OrthoImage3D):
        """
        Transform orthoimage frame pixel coordinates to WGS84 lon, lat coordinates
        """
        pixel_coords = create_src_corners(orthoimage.img.height, orthoimage.img.width)
        geo_coords = cls._boundingbox_to_geo_coords(orthoimage.bbox)

        pixel_coords = np.float32(pixel_coords).squeeze()
        geo_coords = np.float32(geo_coords).squeeze()

        M = cv2.getPerspectiveTransform(pixel_coords, geo_coords)

        # Insert z dimensions
        M = np.insert(M, 2, 0, axis=1)
        M = np.insert(M, 2, 0, axis=0)

        # Scaling of z-axis from orthoimage raster native units to meters
        bounding_box_perimeter_native = (
            2 * orthoimage.img.height + 2 * orthoimage.img.width
        )
        bounding_box_perimeter_meters = cls._bounding_box_perimeter_meters(
            orthoimage.bbox
        )
        M[2, 2] = bounding_box_perimeter_meters / bounding_box_perimeter_native

        return M

    @staticmethod
    def _get_rotation_matrix(image: np.ndarray, degrees: float) -> np.ndarray:
        height, width = image.shape[:2]
        cx, cy = height // 2, width // 2
        r = cv2.getRotationMatrix2D((cx, cy), degrees, 1.0)
        return r

    @staticmethod
    def _get_translation_matrix(dx, dy):
        t = np.float32([[1, 0, dx], [0, 1, dy]])
        return t

    @classmethod
    def _get_affine_matrix(
        cls, image: np.ndarray, degrees: float, crop_height: int, crop_width: int
    ) -> np.ndarray:
        """
        Creates affine transformation that rotates around center and then
        center-crops an image.

        .. note::
            Returns matrix in 3D since this matrix will not only be used for rotating
            and cropping the orthoimage rasters but also for converting 3D pose
            estimates in the rotated and cropped orthoimage frame back to the original
            unrotated and uncropped frame (from where it will then be converted to
            geocoordinates).

        Returns affine matrix padded to 3D (4x4 matrix) in the following format:
            [ R11  R12  0   Tx ]
            [ R21  R22  0   Ty ]
            [ 0    0    1   0  ]
            [ 0    0    0   1  ]
        where R11, R12, R21, R22 represents the rotation matrix, and Tx, Ty represent
        the translation along the x and y axis.

        :return: The affine transformation matrix in homogenous format as masked
            numpy array. Masking for use in 2D operations (e.g. cv2.warpAffine).
        """
        r = cls._get_rotation_matrix(image, degrees)
        assert r.shape == (2, 3)
        dx = (image.shape[0] - crop_height) // 2
        dy = (image.shape[1] - crop_width) // 2
        t = cls._get_translation_matrix(dx, dy)
        assert t.shape == (2, 3)

        # Combine rotation and translation to get the final affine transformation
        affine_2d = np.dot(t, np.vstack([r, [0, 0, 1]]))

        # Convert 2D affine matrix to 3D affine matrix
        # Create a 4x4 identity matrix
        affine_3d = np.eye(4)

        # Insert the 2D affine transformation into the 3D matrix
        affine_3d[:2, :2] = affine_2d[:, :2]
        affine_3d[:2, 3] = affine_2d[:, 2]

        assert affine_3d.shape == (4, 4)

        # Translation hack to make cv2.warpAffine warp the image into the top left
        # corner so that the output size argument of cv2.warpAffine acts as a
        # center-crop
        t[:2, 2] = -t[:2, 2][::-1]
        affine_hack = np.dot(t, np.vstack([r, [0, 0, 1]]))

        affine_3d[:2, :2] = affine_hack[:2, :2]
        affine_3d[:2, 3] = affine_hack[:2, 2]
        return affine_3d

    @classmethod
    def _rotate_and_crop_image(
        cls, image: np.ndarray, degrees: float, shape: Tuple[int, int]
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Rotates around center and then center-crops image

        Cached because the same rotated image is expected to be used for multiple
        matches.

        :return: Tuple of rotated and cropped image, and used affine
            transformation matrix
        """
        # Image can have any number of channels
        affine = cls._get_affine_matrix(image, degrees, *shape)
        affine_2d = np.delete(affine, 2, 1)
        affine_2d = affine_2d[:2, :]
        return cv2.warpAffine(image, affine_2d, shape[::-1]), affine

    @narrow_types
    def _get_pose(
        self, inputs: PoseEstimationInputs, pose_estimator_endpoint: str
    ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """
        Performs call to pose estimation service and returns pose as (r, t) tuple,
        or None if not available
        """

        def _matrices_to_pose(r, t):
            # Convert the rotation matrix to a quaternion
            rotation = Rotation.from_matrix(r)
            quaternion = rotation.as_quat()

            # Create a Pose object
            pose = Pose()

            # Set the position
            pose.position.x = t[0][0]
            pose.position.y = t[1][0]
            pose.position.z = t[2][0]

            # Set the orientation
            pose.orientation.x = quaternion[0]
            pose.orientation.y = quaternion[1]
            pose.orientation.z = quaternion[2]
            pose.orientation.w = quaternion[3]

            return pose

        response = requests.post(
            pose_estimator_endpoint,
            data={k: pickle.dumps(v) for k, v in inputs.items()},
        )

        # TODO: timeout, connection errors, exceptions etc.
        if response.status_code == 200:
            # TODO: should return None if the length of these is 0?
            data = json.loads(response.text)
            if "r" in data and "t" in data:
                r, t = np.asarray(data.get("r")), np.asarray(data.get("t"))

                if __debug__:
                    # Visualize projected FOV estimate
                    h = inputs.get("k") @ np.delete(np.hstack((r, t)), 2, 1)
                    src_pts = create_src_corners(
                        *inputs["query"].shape[0:2][::-1]
                    )  # cv2 flips axis order
                    try:
                        fov_pix = cv2.perspectiveTransform(src_pts, np.linalg.inv(h))
                        ref_img = inputs["reference"]
                        map_with_fov = cv2.polylines(
                            ref_img.copy(),
                            [np.int32(fov_pix)],
                            True,
                            255,
                            3,
                            cv2.LINE_AA,
                        )

                        img: np.ndarray = np.vstack((map_with_fov, inputs["query"]))
                        cv2.imshow("Projected FOV", img)
                        cv2.waitKey(1)
                    except np.linalg.LinAlgError as _:  # noqa: F841
                        self.get_logger().debug(
                            "H was non invertible, cannot visualize."
                        )

                # pose = _matrices_to_pose(r, t)

                # r_new = np.transpose(r, (1,0))
                # t_new = np.array((t[1], t[0], t[2])).reshape(t.shape)
                return r, t  # pose
            else:
                self.get_logger().warn(
                    f"Could not estimate pose, returned text {response.text}"
                )
                return None
        else:
            self.get_logger().warn(
                f"Could not estimate pose, status code {response.status_code}"
            )
            return None

    @classmethod
    # @lru_cache(1) TODO use own cache_if or cachetools from pypi
    def _bounding_box_perimeter_meters(cls, bounding_box: BoundingBox) -> float:
        """Returns the length of the bounding box perimeter in meters"""
        width_meters = cls.haversine_distance(
            bounding_box.min_pt.latitude,
            bounding_box.min_pt.longitude,
            bounding_box.min_pt.latitude,
            bounding_box.max_pt.longitude,
        )
        height_meters = cls.haversine_distance(
            bounding_box.min_pt.latitude,
            bounding_box.min_pt.longitude,
            bounding_box.max_pt.latitude,
            bounding_box.min_pt.longitude,
        )
        return 2 * width_meters + 2 * height_meters

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
        roll = math.arctan2(t0, t1)

        t2 = 2.0 * (q.w * q.y - q.z * q.x)
        t2 = 1.0 if t2 > 1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.arcsin(t2)

        t3 = 2.0 * (q.w * q.z + q.x * q.y)
        t4 = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.arctan2(t3, t4)

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
        if self.gimbal_quaternion is not None:
            off_nadir_deg = self.off_nadir_angle(self.gimbal_quaternion)

            if off_nadir_deg > max_pitch:
                self.get_logger().warn(
                    f"Camera is {off_nadir_deg} degrees off nadir and above "
                    f"limit {max_pitch}."
                )
                return True
            else:
                self.get_logger().debug(
                    f"Camera pitch is {off_nadir_deg} degrees off nadir"
                )
                return False
        else:
            self.get_logger().warn(
                "Gimbal attitude was not available, assuming camera pitch too high."
            )
            return True

    @staticmethod
    def haversine_distance(lat1, lon1, lat2, lon2) -> float:
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