"""This module contains :class:`.TwistNode`, a ROS node that estimates camera
twist i.e. velocity in the camera's intrinsic (REP 103 ``camera_optical``) frame
of reference.

The reference image is a previous image frame from the onboard camera. Visual odometry
(VO) provides a smooth local (relative) pose or twist estimate that drifts over
the long-term.

The pose is estimated by finding matching keypoints between the query and
reference images and then solving the resulting PnP problem.

Does not publish a Twist message, instead publishes Pose which should then be
fused differentially by the EKF.
"""
from typing import Optional, cast

import cv2
import numpy as np
import rclpy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import CameraInfo, Image

from .. import _transformations as tf_
from .._decorators import ROS, narrow_types
from ..constants import (
    ROS_TOPIC_CAMERA_INFO,
    ROS_TOPIC_IMAGE,
    ROS_TOPIC_RELATIVE_MATCHES_IMAGE,
    ROS_TOPIC_RELATIVE_POSE,
    ROS_TOPIC_RELATIVE_POSITION_IMAGE,
    FrameID,
)
from ._shared import COVARIANCE_LIST, compute_pose, visualize_matches_and_pose


class TwistNode(Node):
    """Estimates camera pose in intrinsic (REP 103 ``camera_optical``) frame of
    reference by finding matching keypoints and solving the PnP problem.
    """

    CONFIDENCE_THRESHOLD = 0.9
    """Confidence threshold for filtering out bad keypoint matches"""

    MIN_MATCHES = 30
    """Minimum number of keypoint matches before attempting pose estimation"""

    def __init__(self, *args, **kwargs):
        """Class initializer

        :param args: Positional arguments to parent :class:`.Node` constructor
        :param kwargs: Keyword arguments to parent :class:`.Node` constructor
        """
        super().__init__(*args, **kwargs)

        self._cv_bridge = CvBridge()

        # Initialize ORB detector and brute force matcher for VO
        # (smooth relative position with drift)
        self._orb = cv2.ORB_create()
        self._bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

        # Publishers for dev image
        self._matches_publisher = self.create_publisher(
            Image, ROS_TOPIC_RELATIVE_MATCHES_IMAGE, 10
        )
        self._position_publisher = self.create_publisher(
            Image, ROS_TOPIC_RELATIVE_POSITION_IMAGE, 10
        )

        self._tf_buffer = tf2_ros.Buffer(rclpy.duration.Duration(seconds=30))
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer, self, spin_thread=True
        )
        self._tf_broadcaster = tf2_ros.transform_broadcaster.TransformBroadcaster(self)

        # Cached reference image for visual odometry
        self._cached_reference: Optional[Image] = None
        self._previous_image: Optional[Image] = None  # backup for cached reference

        # initialize subscriptions
        self.camera_info
        # initialize publisher (for launch tests)
        self.pose

    @property
    @ROS.subscribe(
        ROS_TOPIC_CAMERA_INFO,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def camera_info(self) -> Optional[CameraInfo]:
        """Camera info including the intrinsics matrix, or None if unknown"""

    def _image_cb(self, msg: Image) -> None:
        """Callback for :attr:`.image` message"""
        if self._cached_reference is not None:
            self.pose
        else:
            self._cached_reference = msg

        self._previous_image = msg

    @property
    @ROS.subscribe(
        ROS_TOPIC_IMAGE,
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=_image_cb,
    )
    def image(self) -> Optional[Image]:
        """Subscribed raw image from vehicle camera, or None if unknown"""

    @property
    @ROS.publish(
        ROS_TOPIC_RELATIVE_POSE,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def pose(self) -> Optional[PoseWithCovarianceStamped]:
        """Camera pose in previous :term:`REP 105` ``gisnav_odom`` frame

        This is computed as the the relative 3D position and orientation of the
        REP 105 ``camera_optical`` frame in its intrinsic frame from successive
        camera images
        """

        @narrow_types(self)
        def _pose(
            camera_info: CameraInfo, query: Image, reference: Image
        ) -> Optional[PoseWithCovarianceStamped]:
            qry = self._cv_bridge.imgmsg_to_cv2(query, desired_encoding="mono8")
            ref = self._cv_bridge.imgmsg_to_cv2(reference, desired_encoding="mono8")

            # find the keypoints and descriptors with ORB
            kp_qry, desc_qry = self._orb.detectAndCompute(qry, None)
            kp_ref, desc_ref = self._orb.detectAndCompute(ref, None)

            try:
                matches = self._bf.knnMatch(desc_qry, desc_ref, k=2)
            except cv2.error as e:
                self.get_logger().debug(
                    f"Could not match - resetting reference frame: {e}"
                )
                self._cached_reference = self._previous_image
                return None

            if len(matches) < self.MIN_MATCHES:
                self.get_logger().debug(
                    "Not enough matches - resetting reference frame"
                )
                self._cached_reference = self._previous_image
                return None

            # Apply ratio test
            good = []
            for m, n in matches:
                if m.distance < self.CONFIDENCE_THRESHOLD * n.distance:
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
            if mkps is None or len(mkps) < self.MIN_MATCHES:
                self.get_logger().debug(
                    "Not enough matches - resetting reference frame"
                )
                self._cached_reference = self._previous_image
                return None

            mkp_qry, mkp_ref = zip(*mkps)
            mkp_qry = np.array(mkp_qry)
            mkp_ref = np.array(mkp_ref)

            pose = compute_pose(camera_info, mkp_qry, mkp_ref, np.zeros_like(qry))
            if pose is None:
                return None
            r, t = pose

            # VISUALIZE
            match_img = visualize_matches_and_pose(
                camera_info,
                qry.copy(),
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
            try:
                x, y = camera_optical_position_in_world[0:2].squeeze().tolist()
                x, y = int(x), int(y)
                image = cv2.circle(np.array(ref.copy()), (x, y), 5, (0, 255, 0), -1)
                ros_image = self._cv_bridge.cv2_to_imgmsg(image)
                self._position_publisher.publish(ros_image)
            except (cv2.error, ValueError) as e:
                self.get_logger().info(f"Could not draw camera position: {e}")
                return None

            assert self._hfov is not None  # we have camera info
            maximum_pitch_before_horizon_visible = (np.pi / 2) - (self._hfov / 2)
            angle_off_nadir: Optional[float] = None
            query_time = rclpy.time.Time(
                seconds=query.header.stamp.sec,
                nanoseconds=query.header.stamp.nanosec,
            )
            try:
                transform = self._tf_buffer.lookup_transform(
                    "base_link_stabilized",
                    "camera_frd",
                    query_time,
                    rclpy.duration.Duration(seconds=0.2),
                )
                rotation = transform.transform.rotation
                quaternion = (rotation.x, rotation.y, rotation.z, rotation.w)
                angle_off_nadir = tf_.angle_off_nadir(quaternion)
            except Exception as e:
                self.get_logger().warn(f"Could not get transform: {e}")
                return None

            if angle_off_nadir > maximum_pitch_before_horizon_visible:
                self.get_logger().warning(
                    f"Angle off nadir: {np.degrees(angle_off_nadir)} degrees, "
                    f"max angle {np.degrees(maximum_pitch_before_horizon_visible)}. "
                    f" - skipping matching."
                )
                return None

            assert angle_off_nadir is not None
            # TODO: get a better estimate of distance to ground
            try:
                distance_to_ground_transform = self._tf_buffer.lookup_transform(
                    "map", "base_link", query_time, rclpy.duration.Duration(seconds=0.2)
                )
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as e:
                self.get_logger().warning(f"Cannot estimate scale for VO: {e}")
                return None

            distance_to_ground = distance_to_ground_transform.transform.translation.z
            # TODO: handle infinity here
            distance_to_ground_along_optical_axis = distance_to_ground / np.cos(
                angle_off_nadir
            )
            fx = camera_info.k[0]
            scaling = np.abs(distance_to_ground_along_optical_axis / fx)
            camera_optical_position_in_world = (
                camera_optical_position_in_world * scaling
            )

            pose_msg = tf_.create_pose_msg(
                query.header.stamp,
                cast(FrameID, "gisnav_camera_link_optical"),
                r_inv,
                camera_optical_position_in_world,
            )
            if pose_msg is None:
                # TODO: handle better
                return None

            fx = camera_info.k[0]
            pose_msg.pose.position.x -= scaling * camera_info.width / 2
            pose_msg.pose.position.y -= scaling * camera_info.height / 2
            pose_msg.pose.position.z += scaling * fx

            reftime = rclpy.time.Time(
                seconds=reference.header.stamp.sec,
                nanoseconds=reference.header.stamp.nanosec,
            )
            if self._tf_buffer.can_transform(
                "gisnav_odom",
                "gisnav_camera_link_optical",
                reftime,
                rclpy.duration.Duration(seconds=0.2),  # rclpy.time.Time(),
            ):
                pose_msg.header.stamp = reference.header.stamp

                try:
                    pose_msg = self._tf_buffer.transform(pose_msg, "gisnav_odom")
                except tf2_ros.ExtrapolationException as e:
                    self.get_logger().warning(
                        f"Could not extrapolate odom frame - skipping publishing this "
                        f"one: {e}"
                    )
                    return None

                pose_msg.header.stamp = query.header.stamp
            else:
                self.get_logger().warning(
                    "Could not find transform from gisnav_camera_link_optical to "
                    "gisnav_odom - resetting gisnav_odom frame."
                )
                pose_msg.header.stamp = query.header.stamp
                pose_msg.header.frame_id = "gisnav_odom"

            # report base_link pose instead of camera frame pose
            # (fix difference in orientation)
            try:
                transform = self._tf_buffer.lookup_transform(
                    "gisnav_camera_link_optical",
                    "gisnav_base_link",
                    query_time,
                    rclpy.duration.Duration(seconds=0.2),
                )
            except (
                tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException,
            ) as e:
                self.get_logger().warning(
                    f"Could not transform from gisnav_camera_link_optical to "
                    f"gisnav_base_link. Skipping publishing pose. {e}"
                )
                return None
            transform = tf_.add_transform_stamped(pose_msg, transform)
            pose_msg.pose.orientation = transform.transform.rotation
            assert pose_msg.header.frame_id == "gisnav_odom"
            assert pose_msg.header.stamp == query.header.stamp

            # TODO: use custom error model for VO
            pose_with_covariance = PoseWithCovariance(
                pose=pose_msg.pose, covariance=COVARIANCE_LIST
            )
            pose_with_covariance = PoseWithCovarianceStamped(
                header=pose_msg.header, pose=pose_with_covariance
            )
            self._cached_reference = query

            return pose_with_covariance

        return _pose(self.camera_info, self.image, self._cached_reference)

    @property
    def _hfov(self) -> Optional[float]:
        """Horizontal field of view in radians"""

        @narrow_types(self)
        def _hfov(camera_info: CameraInfo) -> Optional[float]:
            # Extract the focal length in the x direction from the intrinsics matrix
            fx = self.camera_info.k[0]

            # Calculate the horizontal field of view (HFOV)
            hfov = 2 * np.arctan(self.camera_info.width / (2 * fx))

            return hfov

        return _hfov(self.camera_info)
