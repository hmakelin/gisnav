"""Module that contains the pose estimation node"""
import json
import math
import pickle
from typing import Optional, Union, get_args

import cv2
import numpy as np
import requests
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from geographic_msgs.msg import GeoPoint, GeoPointStamped, GeoPose, GeoPoseStamped
from geometry_msgs.msg import Quaternion
from gisnav_msgs.msg import OrthoImage3D
from mavros_msgs.msg import Altitude
from rclpy.qos import QoSPresetProfiles
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Image

from ..assertions import assert_ndim, assert_shape, assert_type
from ..data import (
    Attitude,
    ContextualMapData,
    DataValueError,
    FixedCamera,
    ImageData,
    ImagePair,
    Img,
    MapData,
    Pose,
    Position,
)
from ..geo import GeoPt, GeoTrapezoid
from . import messaging
from .base.camera_subscriber_node import CameraSubscriberNode


class PoseEstimationNode(CameraSubscriberNode):
    """Estimates and publishes pose between two images

    Compares images from :class:`sensor_msgs.msg.Image` message to maps from
    :class:`gisnav_msgs.msg.OrthoImage3D` message to estimate
    :class:`geographic_msgs.msg.GeoPoseStamped`.
    """

    # Encoding of input video (input to CvBridge) e.g. gscam2 only supports
    # bgr8 so this is used to override encoding in image header
    _IMAGE_ENCODING = "bgr8"

    ROS_D_POSE_ESTIMATOR_ENDPOINT = "http://localhost:8090/predictions/loftr"
    """Default pose estimator endpoint URL"""

    ROS_D_DEBUG_EXPORT_POSITION = ""  # 'position.json'
    """Default filename for exporting GeoJSON containing estimated FOV and position

    Set to '' to disable
    """

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

    ROS_PARAM_DEFAULTS = [
        ("pose_estimator_endpoint", ROS_D_POSE_ESTIMATOR_ENDPOINT, True),
        ("max_pitch", ROS_D_MISC_MAX_PITCH, False),
        ("min_match_altitude", ROS_D_MISC_MIN_MATCH_ALTITUDE, False),
        (
            "attitude_deviation_threshold",
            ROS_D_MISC_ATTITUDE_DEVIATION_THRESHOLD,
            False,
        ),
        ("export_position", ROS_D_DEBUG_EXPORT_POSITION, False),
    ]
    """List containing ROS parameter name, default value and read_only flag tuples"""

    def __init__(self, name: str) -> None:
        """Node initializer

        :param name: Node name
        """
        super().__init__(name)
        self._package_share_dir = get_package_share_directory("gisnav")

        self._map_data = None

        # Converts image_raw to cv2 compatible image
        self._cv_bridge = CvBridge()

        # region subscribers
        self._orthoimage_3d = None
        self._orthoimage_3d_sub = self.create_subscription(
            OrthoImage3D,
            messaging.ROS_TOPIC_ORTHOIMAGE,
            self._orthoimage_3d_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self._terrain_altitude = None
        self._terrain_altitude_sub = self.create_subscription(
            Altitude,
            messaging.ROS_TOPIC_TERRAIN_ALTITUDE,
            self._terrain_altitude_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self._terrain_geopoint = None
        self._terrain_geopoint_sub = self.create_subscription(
            GeoPointStamped,
            messaging.ROS_TOPIC_TERRAIN_GEOPOINT,
            self._terrain_geopoint_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self._vehicle_altitude = None
        self._vehicle_altitude_sub = self.create_subscription(
            Altitude,
            messaging.ROS_TOPIC_VEHICLE_ALTITUDE,
            self._vehicle_altitude_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self._gimbal_quaternion = None
        self._gimbal_quaternion_sub = self.create_subscription(
            Quaternion,
            messaging.ROS_TOPIC_GIMBAL_QUATERNION,
            self._gimbal_quaternion_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self._vehicle_geopose = None
        self._vehicle_geopose_sub = self.create_subscription(
            GeoPoseStamped,
            messaging.ROS_TOPIC_VEHICLE_GEOPOSE,
            self._vehicle_geopose_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self._home_geopoint = None
        self._home_geopoint_sub = self.create_subscription(
            GeoPointStamped,
            messaging.ROS_TOPIC_HOME_GEOPOINT,
            self._home_geopoint_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        # endregion subscribers

        # region publishers
        self._geopose_pub = self.create_publisher(
            GeoPoseStamped,
            messaging.ROS_TOPIC_VEHICLE_GEOPOSE_ESTIMATE,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self._altitude_pub = self.create_publisher(
            Altitude,
            messaging.ROS_TOPIC_VEHICLE_ALTITUDE_ESTIMATE,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        # endregion publishers

    @property
    def _map_data(self) -> Optional[MapData]:
        """Stored map data"""
        return self.__map_data

    @_map_data.setter
    def _map_data(self, value: MapData) -> None:
        self.__map_data = value

    @property
    def _altitude_scaling(self) -> Optional[float]:
        """Returns camera focal length divided by camera altitude in meters"""
        if self.camera_data is not None and self._vehicle_altitude is not None:
            return (
                self.camera_data.fx / self._vehicle_altitude.terrain
            )  # TODO: assumes fx == fy
        else:
            self.get_logger().warn(
                "Could not estimate elevation scale because camera focal length "
                "and/or vehicle altitude is unknown."
            )
            return None

    @property
    def _r_guess(self) -> Optional[np.ndarray]:
        """Gimbal rotation matrix guess"""
        if self._gimbal_quaternion is None:
            self.get_logger().warn(
                "Gimbal set attitude not available, will not provide pose guess."
            )
            return None
        else:
            gimbal_attitude = Attitude(
                q=messaging.as_np_quaternion(self._gimbal_quaternion)
            )
            gimbal_attitude = (
                gimbal_attitude.to_esd()
            )  # Need coordinates in image frame, not NED
            return gimbal_attitude.r

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

    def _should_estimate(self) -> bool:
        """Determines whether :meth:`._estimate` should be called

        Match should be attempted if (1) a reference map has been retrieved,
        (2) camera roll or pitch is not too high (e.g. facing horizon instead
        of nadir), and (3) drone is not flying too low.

        :return: True if pose estimation be attempted
        """
        # Check condition (1) - that MapData exists
        if self._map_data is None:
            self.get_logger().warn(
                "No reference map available. Skipping pose estimation."
            )
            return False

        # Check condition (2) - whether camera roll/pitch is too large
        max_pitch = self.get_parameter("max_pitch").get_parameter_value().integer_value
        if self._camera_roll_or_pitch_too_high(max_pitch):
            self.get_logger().warn(
                f"Camera roll or pitch not available or above limit {max_pitch}. "
                f"Skipping pose estimation."
            )
            return False

        # Check condition (3) - whether vehicle altitude is too low
        min_alt = (
            self.get_parameter("min_match_altitude").get_parameter_value().integer_value
        )
        assert min_alt > 0
        if self._vehicle_altitude is None or self._vehicle_altitude.terrain is np.nan:
            self.get_logger().warn(
                "Cannot determine altitude AGL, skipping map update."
            )
            return False
        if self._vehicle_altitude.terrain < min_alt:
            self.get_logger().warn(
                f"Assumed altitude {self._vehicle_altitude.terrain} was lower "
                f"than minimum threshold for matching ({min_alt}) or could not "
                f"be determined. Skipping pose estimation."
            )
            return False

        return True

    def image_callback(self, msg: Image) -> None:
        """Handles latest :class:`sensor_msgs.msg.Image` message

        :param msg: The :class:`sensor_msgs.msg.Image` message
        """
        cv_image = self._cv_bridge.imgmsg_to_cv2(msg, self._IMAGE_ENCODING)

        # Check that image dimensions match declared dimensions
        if self.img_dim is not None:
            cv_img_shape = cv_image.shape[0:2]
            assert cv_img_shape == self.img_dim, (
                f"Converted cv_image shape {cv_img_shape} did not match "
                f"declared image shape {self.img_dim}."
            )

        if self.camera_data is None:
            self.get_logger().warn("Camera data not yet available, skipping frame.")
            return None

        image_data = ImageData(
            image=Img(cv_image),
            frame_id=msg.header.frame_id,
            timestamp=self.usec,
            camera_data=self.camera_data,
        )

        if self._should_estimate():
            assert self._map_data is not None
            assert self.camera_data is not None
            assert hasattr(
                self._map_data, "image"
            ), "Map data unexpectedly did not contain the image data."

            # TODO: check below assertion in _should_estimate?
            assert self._contextual_map_data is not None
            contextual_map_data: ContextualMapData = self._contextual_map_data
            image_pair = ImagePair(image_data, contextual_map_data)

            # region pose estimation request
            # TODO: timeout, connection errors, exceptions etc.
            pose_estimator_endpoint = (
                self.get_parameter("pose_estimator_endpoint")
                .get_parameter_value()
                .string_value
            )
            r = requests.post(
                pose_estimator_endpoint,
                data={
                    "query": pickle.dumps(image_pair.qry.image.arr),
                    "reference": pickle.dumps(image_pair.ref.image.arr),
                    "k": pickle.dumps(self.camera_data.k),
                    "elevation": pickle.dumps(image_pair.ref.map_data.elevation.arr),
                },
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

            if pose is not None:
                pose_ = Pose(*pose)
                self._post_process_pose(pose_, image_pair)
            else:
                self.get_logger().warn(
                    "Could not estimate a pose, skipping this frame."
                )
                return None

    def _orthoimage_3d_callback(self, msg: OrthoImage3D) -> None:
        """Handles latest :class:`gisnav_msgs.msg.OrthoImage3D` message

        :param msg: Latest :class:`gisnav_msgs.msg.OrthoImage3D` message
        """
        bbox = messaging.bounding_box_to_bbox(msg.bbox)
        img = self._cv_bridge.imgmsg_to_cv2(msg.img, desired_encoding="passthrough")
        dem = self._cv_bridge.imgmsg_to_cv2(msg.dem, desired_encoding="passthrough")
        assert_type(img, np.ndarray)
        assert_ndim(dem, 2)
        assert_shape(dem, img.shape[0:2])

        assert self.map_size_with_padding is not None

        # Should already have camera info so map_size_with_padding should not be None
        assert img.shape[0:2] == self.map_size_with_padding, (
            f"Decoded map {img.shape[0:2]} is not of specified "
            f"size {self.map_size_with_padding}."
        )

        elevation = Img(dem) if dem is not None else Img(np.zeros(img.shape[0:2]))
        self._map_data = MapData(bbox=bbox, image=Img(img), elevation=elevation)

    def _terrain_altitude_callback(self, msg: Altitude) -> None:
        """Handles latest terrain :class:`mavros_msgs.msg.Altitude` message

        :param msg: Latest terrain :class:`mavros_msgs.msg.Altitude` message
        """
        self._terrain_altitude = msg

    def _home_geopoint_callback(self, msg: GeoPointStamped) -> None:
        """Receives home :class:`geographic_msgs.msg.GeoPointStamped` message

        :param msg: Latest home :class:`geographic_msgs.msg.GeoPointStamped` message
        """
        self._home_geopoint = msg

    def _terrain_geopoint_callback(self, msg: GeoPointStamped) -> None:
        """Receives terrain :class:`geographic_msgs.msg.GeoPointStamped` message

        :param msg: Latest terrain :class:`geographic_msgs.msg.GeoPointStamped` message
        """
        self._terrain_geopoint = msg

    def _vehicle_altitude_callback(self, msg: Altitude) -> None:
        """Receives vehicle :class:`mavros_msgs.msg.Altitude` message

        :param msg: Latest vehicle :class:`mavros_msgs.msg.Altitude` message
        """
        self._vehicle_altitude = msg

    def _gimbal_quaternion_callback(self, msg: Quaternion) -> None:
        """Receives gimbal :class:`geometry_msgs.msg.Quaternion` message

        .. warning::
            This could be gimbal set attitude, not actual attitude

        :param msg: Latest gimbal :class:`geometry_msgs.msg.Quaternion` message
        """
        self._gimbal_quaternion = msg

    def _vehicle_geopose_callback(self, msg: GeoPoseStamped) -> None:
        """Receives vehicle :class:`geographic_msgs.msg.GeoPoseStamped` message

        :param msg: Latest vehicle :class:`geographic_msgs.msg.GeoPoseStamped` message
        """
        self._vehicle_geopose = msg

    def _post_process_pose(self, pose: Pose, image_pair: ImagePair) -> None:
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

            # Export GeoJSON
            export_geojson = (
                self.get_parameter("export_position").get_parameter_value().string_value
            )
            if export_geojson != "":
                self._export_position(
                    fixed_camera.position.xy, fixed_camera.fov.fov, export_geojson
                )

        self.publish(fixed_camera.position)

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

    def _export_position(
        self, position: GeoPt, fov: GeoTrapezoid, filename: str
    ) -> None:
        """Exports the computed position and field of view into a GeoJSON file

        .. note::
            The GeoJSON file is not used by the node but can be accessed by
            GIS software to visualize the data it contains

        :param position: Computed camera position or projected principal point
            for gimbal projection
        :param: fov: Field of view of camera projected to ground
        :param filename: Name of file to write into
        """
        assert_type(position, GeoPt)
        assert_type(fov, GeoTrapezoid)
        assert_type(filename, str)
        try:
            position._geoseries.append(fov._geoseries).to_file(filename)
        except Exception as e:
            self.get_logger().error(
                f"Could not write file {filename} because of exception: {e}"
            )

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
