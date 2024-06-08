"""This module contains :class:`.PoseNode`, a ROS node that estimates camera
pose in global (REP 105 ``earth``) and local (REP 103 ``camera_optical``) frames
of reference.

The reference image is either an orthoimage raster from the onboard GIS server, or a
previous image frame from the onboard camera. The former provides a global (absolute)
but noisy pose estimate that is drift-free, while the latter (visual odometry) provides
a smooth local (relative) pose estimate that drifts over the long-term.

A deep learning model is used for the global matching problem ("deep" matching), while
traditional computer vision is used for the local matching problem ("shallow")
matching.

The pose is estimated by finding matching keypoints between the query and
reference images and then solving the resulting PnP problem.
"""
import os
from typing import Final, Optional, Tuple, Union, cast

import cv2
import numpy as np
import rclpy
import tf2_ros
import tf_transformations
import torch
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from geometry_msgs.msg import (
    PoseWithCovariance,
    PoseWithCovarianceStamped,
    TwistWithCovarianceStamped,
)
from gisnav_msgs.msg import (  # type: ignore[attr-defined]
    MonocularStereoImage,
    OrthoStereoImage,
)
from kornia.feature import DISK, LightGlueMatcher, laf_from_center_scale_ori
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from robot_localization.srv import SetPose
from scipy.interpolate import interp1d
from sensor_msgs.msg import CameraInfo, Image, TimeReference

from .. import _transformations as tf_
from .._decorators import ROS, narrow_types
from ..constants import (
    MAVROS_TOPIC_TIME_REFERENCE,
    ROS_NAMESPACE,
    ROS_TOPIC_CAMERA_INFO,
    ROS_TOPIC_RELATIVE_POSE,
    ROS_TOPIC_RELATIVE_POSE_IMAGE,
    ROS_TOPIC_RELATIVE_QUERY_TWIST,
    ROS_TOPIC_RELATIVE_TWIST_IMAGE,
    STEREO_NODE_NAME,
    FrameID,
)

# TODO: make error model and generate covariance matrix dynamically
# Create dummy covariance matrix
_covariance_matrix = np.zeros((6, 6))
np.fill_diagonal(_covariance_matrix, 9)  # 3 meter SD = 9 variance
_covariance_matrix[3, 3] = np.radians(5**2)  # angle error should be set quite small
_covariance_matrix[4, 4] = _covariance_matrix[3, 3]
_covariance_matrix[5, 5] = _covariance_matrix[3, 3]
_COVARIANCE_LIST = _covariance_matrix.flatten().tolist()


class PoseNode(Node):
    """Estimates camera pose in global (REP 105 ``earth``) and local (REP 103
    ``camera_optical``) frames of reference by finding matching keypoints and
    solving the PnP problem.
    """

    CONFIDENCE_THRESHOLD_SHALLOW_MATCH = 0.9
    """Confidence threshold for filtering out bad keypoint matches for shallow matching

    > [!NOTE]
    > Stricter threshold for shallow matching because mistakes accumulate in VO
    """

    CONFIDENCE_THRESHOLD_DEEP_MATCH = 0.8
    """Confidence threshold for filtering out bad keypoint matches for deep matching"""

    MIN_MATCHES = 30
    """Minimum number of keypoint matches before attempting pose estimation"""

    class _ScalingBuffer:
        """Maintains timestamped query frame to world frame scaling in a sliding window
        buffer so that shallow matching (VO) pose can be scaled to meters using
        scaling information obtained from deep matching
        """

        _WINDOW_LENGTH: Final = 100

        def __init__(self):
            self._scaling_arr: np.ndarray = np.ndarray([])
            self._timestamp_arr: np.ndarray = np.ndarray([])

        def append(self, timestamp_usec: int, scaling: float) -> None:
            self._scaling_arr = np.append(self._scaling_arr, scaling)
            self._timestamp_arr = np.append(self._timestamp_arr, timestamp_usec)

            if self._scaling_arr.size > self._WINDOW_LENGTH:
                self._scaling_arr = self._scaling_arr[-self._WINDOW_LENGTH :]
            if self._timestamp_arr.size > self._WINDOW_LENGTH:
                self._timestamp_arr = self._timestamp_arr[-self._WINDOW_LENGTH :]

        def interpolate(self, timestamp_usec: int) -> Optional[float]:
            if self._scaling_arr.size < 2:
                return None

            interp_function = interp1d(
                self._timestamp_arr,
                self._scaling_arr,
                kind="linear",
                fill_value="extrapolate",
            )
            return interp_function(timestamp_usec)

    def __init__(self, *args, **kwargs):
        """Class initializer

        :param args: Positional arguments to parent :class:`.Node` constructor
        :param kwargs: Keyword arguments to parent :class:`.Node` constructor
        """
        super().__init__(*args, **kwargs)
        self._device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        if self._device == "cuda":
            original_load = torch.load

            def load_onto_gpu(*args, **kwargs):
                """Monkey-patch :func:`torch.load` to force loading directly
                onto GPU

                We load directly onto GPU to avoid memory issues on devices with only
                4GB of CPU memory (e.g. Jetson Nano 4GB model), whereby loading
                LightGlue would cause pt_main_thread to take up almost all
                available main memory and likely freeze the system.

                :func:`torch.hub.load_state_dict_from_url` is used by kornia when
                initializing :class:`LightGlueMatcher` (and possibly also DISK
                extractor). :func:`torch.hub.load_state_dict_from_url` again
                calls :func:`torch.load`, which is the one we will patch just
                in case we skip calling :func:`torch.hub.load_state_dict_from_url`.
                """
                if kwargs.get("map_location", None) != self._device:
                    assert self._device == "cuda"
                    kwargs["map_location"] = self._device
                if not kwargs.get("mmap", False):
                    kwargs["mmap"] = True
                return original_load(*args, **kwargs)

            torch.load = load_onto_gpu

        try:
            # Initialize DL model for map matching (noisy global position, no drift)
            self._matcher = (
                LightGlueMatcher(
                    "disk",
                    params={
                        "filter_threshold": self.CONFIDENCE_THRESHOLD_DEEP_MATCH,
                        "depth_confidence": -1,
                        "width_confidence": -1,
                    },
                )
                .to(self._device)
                .half()
                .eval()
            )
            self._extractor = DISK.from_pretrained("depth").to(self._device).half()
        finally:
            if self._device == "cuda":
                # Restore the original function to avoid side effects
                torch.load = original_load

        # Initialize ORB detector and brute force matcher for VO
        # (smooth relative position with drift)
        self._orb = cv2.ORB_create()
        self._bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

        self._cv_bridge = CvBridge()

        # initialize subscriptions
        self.camera_info
        self.pose_image
        self.twist_image
        self.time_reference

        # initialize publishers (for launch tests)
        self.pose
        self.camera_optical_twist_in_camera_optical_frame(None)

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self._set_pose_client = self.create_client(
            SetPose, "/robot_localization/set_pose"
        )
        while not self._set_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for EKF node set_pose service...")
        self._set_pose_request = SetPose.Request()
        self._pose_sent = False

        self._scaling_buffer = self._ScalingBuffer()

    def _set_initial_pose(self, pose):
        if not self._pose_sent:
            self._set_pose_request.pose = pose
            self._future = self._set_pose_client.call_async(self._set_pose_request)
            self._pose_sent = True

    @property
    @ROS.subscribe(
        MAVROS_TOPIC_TIME_REFERENCE,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def time_reference(self) -> Optional[TimeReference]:
        """Time reference from FCU, or None if unknown"""

    @property
    @ROS.subscribe(
        ROS_TOPIC_CAMERA_INFO,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def camera_info(self) -> Optional[CameraInfo]:
        """Camera info including the intrinsics matrix, or None if unknown"""

    def _pose_image_cb(self, msg: Image) -> None:
        """Callback for :attr:`.pose_image` message"""
        pose = self.pose
        if pose is not None:
            # TODO: need to set via FCU EKF since VO might already be publishing to
            #  EKF node
            self._set_initial_pose(pose)

    def _twist_image_cb(self, msg: Image) -> None:
        """Callback for :attr:`.twist_image` message"""
        self.camera_optical_twist_in_camera_optical_frame(self.twist_image)

    @property
    @ROS.publish(
        ROS_TOPIC_RELATIVE_POSE,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    @ROS.transform(child_frame_id="camera_optical")
    def pose(self) -> Optional[PoseWithCovarianceStamped]:
        """Camera pose in :term:`REP 105` ``earth`` frame

        This represents the global 3D position and orientation of the ``camera_optical``
        frame in the REP 105 ``earth`` (ECEF) frame. This is obtained via deep
        matching and is a discontinuous estimate of pose. It is intended to be fused
        with and complement the continuous or smooth twist estimate obtained via
        shallow matching or visual odometry (VO).
        """
        return self._get_pose(self.pose_image)

    @narrow_types
    def _get_pose(
        self, msg: Union[OrthoStereoImage, MonocularStereoImage]
    ) -> Optional[PoseWithCovarianceStamped]:
        qry, ref, dem = self._preprocess(msg)
        mkp_qry, mkp_ref = self._process(qry, ref)
        shallow_inference = isinstance(msg, MonocularStereoImage)
        pose = self._postprocess(
            mkp_qry,
            mkp_ref,
            dem,
            qry,
            ref,
            "Shallow match / relative position (VO)"
            if shallow_inference
            else "Deep match / absolute global position (GIS)",
        )
        if pose is None:
            return None
        r, t = pose

        r_inv = r.T

        camera_optical_position_in_world = -r_inv @ t

        tf_.visualize_camera_position(
            ref.copy(),
            camera_optical_position_in_world,
            f"Camera {'principal point' if shallow_inference else 'position'} "
            f"in {'previous' if shallow_inference else 'world'} frame, "
            f"{'shallow' if shallow_inference else 'deep'} inference",
        )

        if isinstance(msg, MonocularStereoImage):
            scaling = self._scaling_buffer.interpolate(
                tf_.usec_from_header(msg.query.header)
            )
            if scaling is not None:
                camera_optical_position_in_world = (
                    camera_optical_position_in_world * scaling
                )
            else:
                self.get_logger().debug(
                    "No scaling available for VO - will not return VO pose"
                )
                return None

        pose = tf_.create_pose_msg(
            msg.query.header.stamp,
            cast(FrameID, "earth")
            if isinstance(msg, OrthoStereoImage)
            else cast(FrameID, "camera_optical"),
            r_inv,
            camera_optical_position_in_world,
        )

        if isinstance(msg, OrthoStereoImage):
            affine = tf_.proj_to_affine(msg.crs.data)

            # use z scaling value
            usec = tf_.usec_from_header(msg.query.header)
            scaling = np.abs(affine[2, 2])
            if usec is not None and scaling is not None:
                self._scaling_buffer.append(usec, scaling)

            t_wgs84 = affine @ np.append(camera_optical_position_in_world, 1)
            x, y, z = tf_.wgs84_to_ecef(*t_wgs84.tolist())
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = z

            # Get rotation matrix from world frame to ENU map_gisnav frame - this
            # should only be a rotation around the yaw (and a flip of z axis and sign
            # of rotation, since map_gisnav is ENU while world frame is right-down-
            # forward which projected to ground means ESD)
            R = affine[:3, :3]
            R = R / np.linalg.norm(R, axis=0)

            camera_optical_rotation_in_enu = R @ r_inv

            r_ecef = np.eye(4)
            r_ecef[:3, :3] = tf_.enu_to_ecef_matrix(t_wgs84[0], t_wgs84[1])
            r_ecef[:3, :3] = r_ecef[:3, :3] @ camera_optical_rotation_in_enu

            q = tf_transformations.quaternion_from_matrix(r_ecef)
            pose.pose.orientation = tf_.as_ros_quaternion(np.array(q))

        pose_with_covariance = PoseWithCovariance(
            pose=pose.pose, covariance=_COVARIANCE_LIST
        )

        pose_with_covariance = PoseWithCovarianceStamped(
            header=pose.header, pose=pose_with_covariance
        )

        return pose_with_covariance

    @ROS.publish(
        ROS_TOPIC_RELATIVE_QUERY_TWIST,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    @narrow_types
    def camera_optical_twist_in_camera_optical_frame(
        self, msg: MonocularStereoImage
    ) -> Optional[TwistWithCovarianceStamped]:
        """REP 105 ``camera_optical`` frame twist in its intrinsic frame"""
        scaling = self._scaling_buffer.interpolate(
            tf_.usec_from_header(msg.query.header)
        )
        if scaling is None:
            return None

        x, y, z = (
            scaling * 320.0,
            scaling * 180.0,
            scaling * -205.0,
        )  # todo do not hard code
        previous_pose = tf_.create_identity_pose_stamped(x, y, z)
        previous_pose.header = msg.reference.header
        current_pose = self._get_pose(msg)
        if current_pose is not None:
            return tf_.poses_to_twist(current_pose, previous_pose)
        else:
            return None

    @property
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_POSE_IMAGE.replace("~", STEREO_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=_pose_image_cb,
    )
    def pose_image(self) -> Optional[OrthoStereoImage]:
        """Aligned and cropped query, reference, DEM rasters from
        :class:`.StereoNode`

        This image couple is used for "deep" matching.
        """

    @property
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_TWIST_IMAGE.replace("~", STEREO_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=_twist_image_cb,
    )
    def twist_image(self) -> Optional[MonocularStereoImage]:
        """Aligned and cropped query and reference images from :class:`.StereoNode`

        This image couple is used for visual odometry or "shallow" matching.
        """

    @narrow_types
    def _preprocess(
        self,
        stereo_image: Union[OrthoStereoImage, MonocularStereoImage],
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Converts :class:`.Image` message to numpy arrays

        :param stereo_image: A GISNav format stereo image message
        :return: A 3-tuple/triplet of query image, reference image, and DEM rasters
        """
        # Convert the ROS Image message to an OpenCV image
        if isinstance(stereo_image, OrthoStereoImage):
            # Extract individual channels
            query_img = self._cv_bridge.imgmsg_to_cv2(
                stereo_image.query, desired_encoding="passthrough"
            )
            query_img = cv2.cvtColor(query_img, cv2.COLOR_BGR2GRAY)
            reference_img = self._cv_bridge.imgmsg_to_cv2(
                stereo_image.reference, desired_encoding="mono8"
            )
            assert reference_img.ndim == 2 or reference_img.shape[2] == 1
            reference_elevation = self._cv_bridge.imgmsg_to_cv2(
                stereo_image.dem, desired_encoding="mono8"
            )

            return (
                query_img,
                reference_img,
                reference_elevation,
            )
        else:
            assert isinstance(stereo_image, MonocularStereoImage)
            query_img = self._cv_bridge.imgmsg_to_cv2(
                stereo_image.query, desired_encoding="mono8"
            )
            reference_img = self._cv_bridge.imgmsg_to_cv2(
                stereo_image.reference, desired_encoding="mono8"
            )

            return query_img, reference_img, np.zeros_like(reference_img)

    def _process(
        self, qry: np.ndarray, ref: np.ndarray, shallow_inference: bool = False
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Returns keypoint matches for input image pair

        :return: Tuple of matched query image keypoints, and matched reference image
            keypoints
        """
        if not shallow_inference:
            qry_tensor = torch.Tensor(qry[None, None]).to(self._device).half() / 255.0
            ref_tensor = torch.Tensor(ref[None, None]).to(self._device).half() / 255.0
            qry_tensor = qry_tensor.expand(-1, 3, -1, -1)
            ref_tensor = ref_tensor.expand(-1, 3, -1, -1)

            with torch.inference_mode():  # , torch.autocast(self._device):
                input = torch.cat([qry_tensor, ref_tensor], dim=0)
                # limit number of features to run faster, None means no limit i.e.
                # slow but accurate
                max_keypoints = 256  # 4096  # None
                feat_qry, feat_ref = self._extractor(
                    input, max_keypoints, pad_if_not_divisible=True
                )
                kp_qry, desc_qry = feat_qry.keypoints, feat_qry.descriptors
                kp_ref, desc_ref = feat_ref.keypoints, feat_ref.descriptors
                lafs_qry = laf_from_center_scale_ori(
                    kp_qry[None], torch.ones(1, len(kp_qry), 1, 1, device=self._device)
                )
                lafs_ref = laf_from_center_scale_ori(
                    kp_ref[None], torch.ones(1, len(kp_ref), 1, 1, device=self._device)
                )
                dists, match_indices = self._matcher(
                    desc_qry.half(), desc_ref.half(), lafs_qry.half(), lafs_ref.half()
                )

            mkp_qry = kp_qry[match_indices[:, 0]].cpu().numpy()
            mkp_ref = kp_ref[match_indices[:, 1]].cpu().numpy()

            # Free memory
            torch.cuda.empty_cache()

            return mkp_qry, mkp_ref

        else:
            # find the keypoints and descriptors with ORB
            kp_qry, desc_qry = self._orb.detectAndCompute(qry, None)
            kp_ref, desc_ref = self._orb.detectAndCompute(ref, None)

            matches = self._bf.knnMatch(desc_qry, desc_ref, k=2)

            # Apply ratio test
            good = []
            for m, n in matches:
                if m.distance < self.CONFIDENCE_THRESHOLD_SHALLOW_MATCH * n.distance:
                    good.append(m)

            mkps = list(
                map(
                    lambda dmatch: (
                        tuple(kp_qry[dmatch.queryIdx].pt),
                        tuple(kp_ref[dmatch.trainIdx].pt),
                    ),
                    good,
                )
            )

            mkp_qry, mkp_ref = zip(*mkps)
            mkp_qry = np.array(mkp_qry)
            mkp_ref = np.array(mkp_ref)

            # Free memory by deleting intermediate variables
            # del kp_qry, desc_qry, kp_ref, desc_ref, matches, good, mkps
            torch.cuda.empty_cache()

            return mkp_qry, mkp_ref

    def _postprocess(
        self,
        mkp_qry: np.ndarray,
        mkp_ref: np.ndarray,
        elevation: np.ndarray,
        qry_img: np.ndarray,
        ref_img: np.ndarray,
        label: str,
    ) -> Optional[PoseWithCovarianceStamped]:
        """Computes camera pose from keypoint matches"""

        @narrow_types(self)
        def _visualize_matches_and_pose(
            camera_info: CameraInfo,
            qry: np.ndarray,
            ref: np.ndarray,
            mkp_qry: np.ndarray,
            mkp_ref: np.ndarray,
            r: np.ndarray,
            t: np.ndarray,
            label: str,
        ) -> None:
            """Visualizes matches and projected FOV"""

            def _project_fov(img, h_matrix):
                """Projects FOV on reference image"""
                height, width = img.shape[0:2]
                src_pts = np.float32(
                    [[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]]
                ).reshape(-1, 1, 2)
                try:
                    return cv2.perspectiveTransform(src_pts, np.linalg.inv(h_matrix))
                except np.linalg.LinAlgError:
                    return src_pts

            k = camera_info.k.reshape((3, 3))
            h_matrix = k @ np.delete(np.hstack((r, t)), 2, 1)
            projected_fov = _project_fov(qry, h_matrix)

            img_with_fov = cv2.polylines(
                ref, [np.int32(projected_fov)], True, 255, 3, cv2.LINE_AA
            )

            mkp_qry = [cv2.KeyPoint(x[0], x[1], 1) for x in mkp_qry]
            mkp_ref = [cv2.KeyPoint(x[0], x[1], 1) for x in mkp_ref]

            matches = [cv2.DMatch(i, i, 0) for i in range(len(mkp_qry))]

            match_img = cv2.drawMatches(
                img_with_fov,
                mkp_ref,
                qry,
                mkp_qry,
                matches,
                None,
                matchColor=(0, 255, 0),
                flags=2,
            )
            # Require HEADLESS explicitly set to 0 before we call highgui
            if int(os.getenv("HEADLESS", 1)) == 0:
                cv2.imshow(label, match_img)
                cv2.waitKey(1)

            # Free memory by deleting intermediate variables
            torch.cuda.empty_cache()

        @narrow_types(self)
        def _compute_pose(
            camera_info: CameraInfo,
            mkp_qry: np.ndarray,
            mkp_ref: np.ndarray,
            elevation: np.ndarray,
            qry_img: np.ndarray,
            ref_img: np.ndarray,
            label: str,
        ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
            if len(mkp_qry) < self.MIN_MATCHES:
                self.get_logger().debug("Not enough matches - returning None")
                return None

            def _compute_3d_points(
                mkp_ref: np.ndarray, elevation: np.ndarray
            ) -> np.ndarray:
                """Computes 3D points from matches"""
                if elevation is None:
                    return np.hstack((mkp_ref, np.zeros((len(mkp_ref), 1))))

                x, y = np.transpose(np.floor(mkp_ref).astype(int))
                z_values = elevation[y, x].reshape(-1, 1)
                return np.hstack((mkp_ref, z_values))

            def _compute_pose(
                mkp2_3d: np.ndarray, mkp_qry: np.ndarray, k_matrix: np.ndarray
            ) -> Tuple[np.ndarray, np.ndarray]:
                """Computes :term:`pose` using :func:`cv2.solvePnPRansac`"""
                dist_coeffs = np.zeros((4, 1))
                # self.get_logger().info(f"{mkp2_3d} {mkp_qry}")
                _, r, t, _ = cv2.solvePnPRansac(
                    mkp2_3d.astype("float32"),
                    mkp_qry.astype("float32"),
                    k_matrix,
                    dist_coeffs,
                    useExtrinsicGuess=False,
                    iterationsCount=10,
                )
                r_matrix, _ = cv2.Rodrigues(r)

                return r_matrix, t

            k_matrix = camera_info.k.reshape((3, 3))

            mkp2_3d = _compute_3d_points(mkp_ref, elevation)

            r, t = _compute_pose(mkp2_3d, mkp_qry, k_matrix)

            _visualize_matches_and_pose(
                camera_info,
                qry_img.copy(),
                ref_img.copy(),
                mkp_qry,
                mkp_ref,
                r,
                t,
                label,
            )

            return r, t

        return _compute_pose(
            self.camera_info, mkp_qry, mkp_ref, elevation, qry_img, ref_img, label
        )

    def _get_stamp(self, msg) -> Time:
        if self.time_reference is None:
            stamp = msg.header.stamp
        else:
            stamp = (
                rclpy.time.Time.from_msg(msg.header.stamp)
                - (
                    rclpy.time.Time.from_msg(self.time_reference.header.stamp)
                    - rclpy.time.Time.from_msg(self.time_reference.time_ref)
                )
            ).to_msg()
        return stamp
