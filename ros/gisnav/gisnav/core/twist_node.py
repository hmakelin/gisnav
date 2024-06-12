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
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped
from gisnav_msgs.msg import MonocularStereoImage  # type: ignore[attr-defined]
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import CameraInfo, Image, TimeReference

from .. import _transformations as tf_
from .._decorators import ROS, narrow_types
from ..constants import (
    MAVROS_TOPIC_TIME_REFERENCE,
    ROS_NAMESPACE,
    ROS_TOPIC_CAMERA_INFO,
    ROS_TOPIC_RELATIVE_MATCHES_IMAGE,
    ROS_TOPIC_RELATIVE_POSE,
    ROS_TOPIC_RELATIVE_POSITION_IMAGE,
    ROS_TOPIC_RELATIVE_TWIST_IMAGE,
    STEREO_NODE_NAME,
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

        # initialize subscriptions
        self.camera_info
        self.time_reference
        self.twist_image
        # initialize publisher (for launch tests)
        self.pose

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

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

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

    def _twist_image_cb(self, msg: Image) -> None:
        """Callback for :attr:`.twist_image` message"""
        self.pose

    @property
    @ROS.publish(
        ROS_TOPIC_RELATIVE_POSE,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    @ROS.transform(child_frame_id="camera_optical")
    def pose(self) -> Optional[PoseWithCovarianceStamped]:
        """Camera pose in previous :term:`REP 105` ``camera_optical`` frame

        This represents the relative 3D position and orientation of the  REP 105
        ``camera_optical`` frame in its intrinsic frame in the REP 105 ``earth``
        (ECEF) frame.
        """

        @narrow_types(self)
        def _pose(
            camera_info: CameraInfo, msg: MonocularStereoImage
        ) -> Optional[PoseWithCovarianceStamped]:
            qry = self._cv_bridge.imgmsg_to_cv2(msg.query, desired_encoding="mono8")
            ref = self._cv_bridge.imgmsg_to_cv2(msg.reference, desired_encoding="mono8")

            # find the keypoints and descriptors with ORB
            kp_qry, desc_qry = self._orb.detectAndCompute(qry, None)
            kp_ref, desc_ref = self._orb.detectAndCompute(ref, None)

            matches = self._bf.knnMatch(desc_qry, desc_ref, k=2)

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
            if mkps is None:
                # TODO: log here
                return None

            mkp_qry, mkp_ref = zip(*mkps)
            mkp_qry = np.array(mkp_qry)
            mkp_ref = np.array(mkp_ref)

            if len(mkp_qry) < self.MIN_MATCHES:
                self.get_logger().debug("Not enough matches - returning None")
                return None

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
            x, y = camera_optical_position_in_world[0:2].squeeze().tolist()
            x, y = int(x), int(y)
            image = cv2.circle(np.array(ref.copy()), (x, y), 5, (0, 255, 0), -1)
            ros_image = self._cv_bridge.cv2_to_imgmsg(image)
            self._position_publisher.publish(ros_image)

            # TODO: scale position based on altitude here
            #  Altitude can come from FCU EKF - no need to have tight coupling with
            #  PoseNode deep matching.
            # if isinstance(msg, MonocularStereoImage):
            #    scaling = self._scaling_buffer.interpolate(
            #        tf_.usec_from_header(msg.query.header)
            #    )
            #    if scaling is not None:
            #        camera_optical_position_in_world = (
            #            camera_optical_position_in_world * scaling
            #        )
            #    else:
            #        self.get_logger().debug(
            #            "No scaling availble for VO - will not return VO pose"
            #        )
            #        return None

            pose = tf_.create_pose_msg(
                msg.query.header.stamp,
                cast(FrameID, "camera_optical"),
                r_inv,
                camera_optical_position_in_world,
            )

            if pose is not None:
                pose_with_covariance = PoseWithCovariance(
                    pose=pose.pose, covariance=COVARIANCE_LIST
                )
                pose_with_covariance = PoseWithCovarianceStamped(
                    header=pose.header, pose=pose_with_covariance
                )
                return pose_with_covariance
            else:
                return None

        return _pose(self.camera_info, self.twist_image)

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
