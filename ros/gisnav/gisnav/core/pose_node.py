"""This module contains :class:`.PoseNode`, a ROS node that estimates camera
pose in the global (REP 105 ``earth``) frame of reference.

The reference image is an orthoimage raster from the onboard GIS server. Deep learning
based keypoint matching provides a global (absolute) but noisy pose estimate that is
drift-free.

The pose is estimated by finding matching keypoints between the query and
reference images and then solving the resulting PnP problem.
"""
from typing import Optional, cast

import cv2
import numpy as np
import rclpy
import tf2_ros
import tf_transformations
import torch
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped
from gisnav_msgs.msg import OrthoStereoImage  # type: ignore[attr-defined]
from kornia.feature import LightGlueMatcher, get_laf_center, laf_from_center_scale_ori
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from robot_localization.srv import SetPose
from sensor_msgs.msg import CameraInfo, Image

from .. import _transformations as tf_
from .._decorators import ROS, narrow_types
from ..constants import (
    ROS_NAMESPACE,
    ROS_TOPIC_CAMERA_INFO,
    ROS_TOPIC_RELATIVE_MATCHES_IMAGE,
    ROS_TOPIC_RELATIVE_POSE,
    ROS_TOPIC_RELATIVE_POSE_IMAGE,
    ROS_TOPIC_RELATIVE_POSITION_IMAGE,
    STEREO_NODE_NAME,
    FrameID,
)
from ._shared import (
    COVARIANCE_LIST_GLOBAL,
    KEYPOINT_DTYPE,
    compute_pose,
    visualize_matches_and_pose,
)

# Fix "UserWarning: Plan failed with a cudnnException:
# CUDNN_BACKEND_EXECUTION_PLAN_DESCRIPTOR: cudnnFinalize Descriptor Failed
# cudnn_status: CUDNN_STATUS_NOT_SUPPORTED (Triggered internally at
# ../aten/src/ATen/native/cudnn/Conv_v8.cpp:919"
# torch.backends.cudnn.benchmark = True
# torch.backends.cudnn.deterministic = True


class PoseNode(Node):
    """Estimates camera pose in global (REP 105 ``earth``) frame of reference by
    finding matching keypoints and solving the PnP problem.
    """

    CONFIDENCE_THRESHOLD = 0.8
    """Confidence threshold for filtering out bad keypoint matches"""

    MIN_MATCHES = 30
    """Minimum number of keypoint matches before attempting pose estimation"""

    def __init__(self, *args, **kwargs):
        """Class initializer

        :param args: Positional arguments to parent :class:`.Node` constructor
        :param kwargs: Keyword arguments to parent :class:`.Node` constructor
        """
        super().__init__(*args, **kwargs)
        self._device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self._cv_bridge = CvBridge()

        # Initialize DL model for map matching
        self._matcher = (
            LightGlueMatcher(
                "sift",
                params={
                    "filter_threshold": self.CONFIDENCE_THRESHOLD,
                    "depth_confidence": -1,
                    "width_confidence": -1,
                },
            )
            .to(self._device)
            .eval()
        )
        self._extractor = cv2.SIFT_create()

        # initialize subscriptions
        self.camera_info
        self.pose_image

        # initialize publisher (for launch tests)
        self.pose

        # Client for setting initial pose to EKF
        # TODO: this only sets pose, not velocity/twist
        self._set_pose_client = self.create_client(
            SetPose, "/robot_localization/set_pose"
        )
        while not self._set_pose_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Waiting for EKF node set_pose service...")
        self._set_pose_request = SetPose.Request()
        self._pose_sent = False

        # Publishers for dev image
        self._matches_publisher = self.create_publisher(
            Image, ROS_TOPIC_RELATIVE_MATCHES_IMAGE, 10
        )
        self._position_publisher = self.create_publisher(
            Image, ROS_TOPIC_RELATIVE_POSITION_IMAGE, 10
        )

        # Deep matching can be very slow when running on CPU so we need to keep
        # transformations in a buffer for a long time
        self._tf_buffer = tf2_ros.Buffer(rclpy.duration.Duration(seconds=30))
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer, self, spin_thread=True
        )
        self._tf_broadcaster = tf2_ros.transform_broadcaster.TransformBroadcaster(self)
        self._tf_static_broadcaster = (
            tf2_ros.static_transform_broadcaster.StaticTransformBroadcaster(self)
        )

    def _set_initial_pose(self, pose):
        if not self._pose_sent:
            self._set_pose_request.pose = pose
            self._future = self._set_pose_client.call_async(self._set_pose_request)
            self._pose_sent = True

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
            #  EKF node?
            self._set_initial_pose(pose)

    @property
    @ROS.publish(
        ROS_TOPIC_RELATIVE_POSE,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def pose(self) -> Optional[PoseWithCovarianceStamped]:
        """Camera pose in :term:`REP 105` ``earth`` frame

        This represents the global 3D position and orientation of the ``camera_optical``
        frame in the REP 105 ``earth`` (ECEF) frame. This is obtained via deep
        matching and is a discontinuous estimate of pose. It is intended to be fused
        with and complement the continous or smooth twist estimate obtained via
        shallow matching or visual odometry (VO).
        """

        @narrow_types(self)
        def _pose(
            camera_info: CameraInfo,
            msg: OrthoStereoImage,
        ) -> Optional[PoseWithCovarianceStamped]:
            # Get the point cloud data as a numpy array
            data = np.frombuffer(msg.query_sift.data, dtype=KEYPOINT_DTYPE)

            # TODO: insert z/depth coordinates from elsewhere?
            kp_qry_cv2 = np.column_stack((data["x"], data["y"]))
            descs_qry_cv2 = data["descriptor"]
            kp_qry_cv2_size = data["size"]
            kp_qry_cv2_angle = data["angle"]

            # Convert the ROS Image message to an OpenCV image
            ref = self._cv_bridge.imgmsg_to_cv2(msg.reference, desired_encoding="mono8")
            assert ref.ndim == 2 or ref.shape[2] == 1

            # reference_img = cv2.cvtColor(reference_img, cv2.COLOR_BGR2GRAY)
            # TODO: Support 16-bit elevation
            reference_elevation = self._cv_bridge.imgmsg_to_cv2(
                msg.dem, desired_encoding="mono8"
            )

            with torch.inference_mode():
                kp_ref_cv2_orig, descs_ref_cv2 = self._extractor.detectAndCompute(
                    ref, None
                )
                kp_ref_cv2 = cv2.KeyPoint_convert(kp_ref_cv2_orig)
                kp_ref_cv2_size = np.array(
                    tuple(map(lambda kp: kp.size, kp_ref_cv2_orig)), dtype=np.float32
                )
                kp_ref_cv2_angle = np.array(
                    tuple(map(lambda kp: kp.angle, kp_ref_cv2_orig)), dtype=np.float32
                )

                kp_qry_cv2, descs_qry_cv2, kp_qry_cv2_size, kp_qry_cv2_angle = tuple(
                    map(
                        lambda arr: torch.from_numpy(arr).to(self._device),
                        (kp_qry_cv2, descs_qry_cv2, kp_qry_cv2_size, kp_qry_cv2_angle),
                    )
                )
                kp_ref_cv2, descs_ref_cv2, kp_ref_cv2_size, kp_ref_cv2_angle = tuple(
                    map(
                        lambda arr: torch.from_numpy(arr).to(self._device),
                        (kp_ref_cv2, descs_ref_cv2, kp_ref_cv2_size, kp_ref_cv2_angle),
                    )
                )

                lafs_qry_cv2 = laf_from_center_scale_ori(
                    kp_qry_cv2.unsqueeze(0),
                    kp_qry_cv2_size[None, :, None, None],
                    kp_qry_cv2_angle[None, :, None],
                )
                lafs_ref_cv2 = laf_from_center_scale_ori(
                    kp_ref_cv2.unsqueeze(0),
                    kp_ref_cv2_size[None, :, None, None],
                    kp_ref_cv2_angle[None, :, None],
                )

                # Convert to RootSIFT (required by kornia LightGlueMatcher)
                descs_qry_cv2 = torch.nn.functional.normalize(
                    descs_qry_cv2, dim=-1, p=1
                ).sqrt()
                descs_ref_cv2 = torch.nn.functional.normalize(
                    descs_ref_cv2, dim=-1, p=1
                ).sqrt()
                dists, match_indices = self._matcher(
                    descs_qry_cv2, descs_ref_cv2, lafs_qry_cv2, lafs_ref_cv2
                )

                kp_qry = get_laf_center(lafs_qry_cv2).squeeze()
                kp_ref = get_laf_center(lafs_ref_cv2).squeeze()

                # Artificially increase matching time (simulate CPU or resource
                # constrained device)
                # time.sleep(5)

                mkp_qry = kp_qry[match_indices[:, 0]].cpu().numpy()
                mkp_ref = kp_ref[match_indices[:, 1]].cpu().numpy()

            if len(mkp_qry) < self.MIN_MATCHES:
                self.get_logger().warning(
                    f"Not enough matches ({len(mkp_qry)})- returning None"
                )
                return None

            pose = compute_pose(camera_info, mkp_qry, mkp_ref, reference_elevation)
            if pose is None:
                return None
            r, t = pose

            # VISUALIZE
            # TODO: include query image in OrthoStereoImage message to enable
            #  this visualization, now we only have SIFT features
            match_img = visualize_matches_and_pose(
                camera_info,
                np.zeros_like(ref),  # todo query image here
                ref.copy(),
                mkp_qry,
                mkp_ref,
                r,
                t,
            )
            ros_match_image = self._cv_bridge.cv2_to_imgmsg(match_img)
            self._matches_publisher.publish(ros_match_image)
            # END VISUALIZE

            r_inv = r.T
            camera_optical_position_in_world = -r_inv @ t

            # Publish camera position in world frame to ROS for debugging
            x, y = camera_optical_position_in_world[0:2].squeeze().tolist()
            x, y = int(x), int(y)

            if not (0 <= x <= ref.shape[0] and 0 <= y <= ref.shape[1]):
                self.get_logger().warning(f"center {(x, y)} was not in expected range")
                return None

            image = cv2.circle(np.array(ref.copy()), (x, y), 5, (0, 255, 0), -1)
            ros_image = self._cv_bridge.cv2_to_imgmsg(image)
            self._position_publisher.publish(ros_image)

            pose = tf_.create_pose_msg(
                msg.query.header.stamp,
                cast(FrameID, "earth"),
                r_inv,
                camera_optical_position_in_world,
            )
            if pose is None:
                self.get_logger().warning("Could not create pose msg - returning None")
                # TODO: handle better
                return None

            affine = tf_.proj_to_affine(msg.crs.data)

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

            # todo add functions for transforms arithmetic and clean up this whole
            #  section
            earth_to_gisnav_camera_optical = tf_.pose_to_transform(
                pose, "gisnav_camera_link_optical"
            )

            if self._tf_buffer.can_transform(
                "gisnav_camera_link_optical", "gisnav_odom", rclpy.time.Time()
            ):
                query_time = rclpy.time.Time(
                    seconds=msg.query.header.stamp.sec,
                    nanoseconds=msg.query.header.stamp.nanosec,
                )

                if not self._tf_buffer.can_transform("earth", "gisnav_map", query_time):
                    try:
                        camera_optical_to_map = self._tf_buffer.lookup_transform(
                            "camera_optical",
                            "map",
                            query_time,
                            rclpy.duration.Duration(seconds=0.1),
                        )
                    except (
                        tf2_ros.LookupException,
                        tf2_ros.ConnectivityException,
                        tf2_ros.ExtrapolationException,
                    ) as e:
                        self.get_logger().warning(
                            f"Could not transform from camera_optical to "
                            f"map. Skipping publishing pose. {e}"
                        )
                        return None

                    # Put gisnav_map roughly where (mavros_)map is, this should make it
                    # ENU and thereby comply with REP 105. Assumes current
                    # camera_optical to map transform from FCU via MAVROS is
                    # sufficiently correct
                    # TODO: implement without assumption FCU EKF has correct state
                    #  estimate?
                    earth_to_gisnav_map = tf_.add_transform_stamped(
                        earth_to_gisnav_camera_optical, camera_optical_to_map
                    )
                    earth_to_gisnav_map.header.frame_id = "earth"
                    earth_to_gisnav_map.child_frame_id = "gisnav_map"
                    self._tf_static_broadcaster.sendTransform([earth_to_gisnav_map])

                    # TODO implement better, no need to return None here, we can publish
                    return None

                # TODO: this is earth to map
                gisnav_map_to_earth = tf_.lookup_transform(
                    self._tf_buffer,
                    "gisnav_map",
                    "earth",
                    (msg.query.header.stamp, rclpy.duration.Duration(seconds=0.2)),
                    self.get_logger(),
                )
                # TODO: this is base_link to camera_link_optical
                gisnav_camera_optical_to_base_link = tf_.lookup_transform(
                    self._tf_buffer,
                    "gisnav_camera_link_optical",
                    "gisnav_base_link",
                    (msg.query.header.stamp, rclpy.duration.Duration(seconds=0.2)),
                    self.get_logger(),
                )
                if (
                    gisnav_map_to_earth is None
                    or gisnav_camera_optical_to_base_link is None
                ):
                    self.get_logger().warning(
                        "Could not transform from gisnav_camera_link_optical to "
                        "gisnav_base_link. Skipping publishing pose."
                    )
                    return None

                gisnav_map_to_camera_link_optical = tf_.add_transform_stamped(
                    gisnav_map_to_earth, earth_to_gisnav_camera_optical
                )
                gisnav_map_to_base_link = tf_.add_transform_stamped(
                    gisnav_map_to_camera_link_optical,
                    gisnav_camera_optical_to_base_link,
                )

                pose_msg = tf_.transform_to_pose(gisnav_map_to_base_link)
                pose_msg.header.frame_id = "gisnav_map"
            else:
                self.get_logger().warning(
                    "Odom frame likely not yet initialized, skpping publishing global "
                    "pose"
                )
                return None

            assert pose_msg is not None

            # TODO: re-enable covariance/implement error model
            pose_with_covariance = PoseWithCovariance(
                pose=pose_msg.pose  # , covariance=COVARIANCE_LIST_GLOBAL
            )

            pose_with_covariance = PoseWithCovarianceStamped(
                header=pose_msg.header, pose=pose_with_covariance
            )

            # Pose should have the query image timestamp
            pose_with_covariance.header.stamp = msg.query.header.stamp

            return pose_with_covariance

        return _pose(self.camera_info, self.pose_image)

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
