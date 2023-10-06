"""This module contains :class:`.TorchNode`, a :term:`ROS` node for estimating
:term:`camera` relative pose between a :term:`query` and :term:`reference` image
"""
from typing import Optional

import numpy as np
import torch
import cv2
import tf_transformations

from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from kornia.feature import LoFTR
from geometry_msgs.msg import Pose, PoseStamped, Point, Quaternion
from sensor_msgs.msg import CameraInfo, Image
from cv_bridge import CvBridge

import ..messaging
from ..decorators import ROS, narrow_types
from ..static_configuration import ROS_NAMESPACE, CV_NODE_NAME, ROS_TOPIC_RELATIVE_CAMERA_ESTIMATED_POSE, ROS_TOPIC_RELATIVE_IMAGE_PAIR


class TorchNode(Node):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._model = LoFTR(pretrained="outdoor")
        self._cv_bridge = CvBridge()

        # initialize subscription
        self.camera_info
        self.image_triplet

    @property
    # @ROS.max_delay_ms(messaging.DELAY_SLOW_MS) - gst plugin does not enable timestamp?
    @ROS.subscribe(messaging.ROS_TOPIC_CAMERA_INFO, QoSPresetProfiles.SENSOR_DATA.value)
    def camera_info(self) -> Optional[CameraInfo]:
        """Camera info for determining appropriate :attr:`.orthoimage` resolution"""

    @property
    @ROS.max_delay_ms(messaging.DELAY_DEFAULT_MS)
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_IMAGE_TRIPLET.replace("~", CV_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=_image_triplet_cb,
    )
    def image_triplet(self) -> Optional[Image]:
        """term:`Query`, :term:`reference`, and :term:`elevation` image triplet,
        represented as a single image. The query image is on the left, the
        reference image is in the middle, and the elevation image is on the right.

        The header frame_id is a PROJ string that contains the information to
        project the relative pose into a global pose.

        .. note::
            The existing :class:`sensor_msgs.msg.Image` message is repurposed
            to represent a stereo couple with depth information (technically a
            triplet) to avoid having to introduce custom messages that would
            have to be distributed in a separate package. It will be easier to
            package this later as a rosdebian if everything is already in the
            rosdep index.
        """

    @property
    @ROS.publish(
        ROS_TOPIC_RELATIVE_CAMERA_ESTIMATED_POSE,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def camera_estimated_pose(self) -> Optional[PoseStamped]:
        """Published :term:`camera` relative :term:`pose` estimate, or None if
        not available or too old

        The header frame_id is a PROJ string that contains the information to
        project the relative pose into a global pose.
        """
        @narrow_types(self)
        def _camera_estimated_pose(image_triplet: Image) -> Optional[PoseStamped]:
            preprocessed = self.preprocess(image_triplet)
            inferred = self.inference(preprocessed)
            pose = self.postprocess(inferred)
            return PoseStamped(
                header=self.image_triplet.header,
                pose=pose
            )

        return _camera_estimated_pose(self.image_triplet)

    def _image_triplet_cb(self, msg: Image) -> None:
        """Callback for :attr:`.image_triplet` message"""
        self.camera_estimated_pose

    @narrow_types
    def _preprocess(self, image_triplet: Image) -> dict:
        """Converts incoming images to torch tensors

        :param image_triplet: :term:`Query`, :term:`reference`, and :term:`elevation`
            image triplet, represented as a single image. The query image is
            on the left, the reference image is in the middle, and the
            elevation image is on the right.
        """
        # Convert the ROS Image message to an OpenCV image
        full_image_cv = self.bridge.imgmsg_to_cv2(image_triplet, desired_encoding="passthrough")

        # Check that the width is divisible by 3
        width = full_image_cv.shape[1]
        assert width % 3 == 0, "The width of the image_triplet must be divisible by 3"

        # Split the full image into query, reference, and elevation images
        third_width = width // 3
        query_img = full_image_cv[:, :third_width]
        reference_img = full_image_cv[:, third_width:2 * third_width]
        reference_elevation = full_image_cv[:, 2 * third_width:]

        # self._display_images("Query", query_img, "Reference", reference_img)

        qry_tensor, ref_tensor = self._convert_images_to_tensors(
            query_img, reference_img
        )

        return (
            {"image0": qry_tensor, "image1": ref_tensor},
            query_img,
            reference_img,
            reference_elevation,
        )

    # @staticmethod
    # def _display_images(*args):
    #    """Displays images using OpenCV"""
    #    for i in range(0, len(args), 2):
    #        cv2.imshow(args[i], args[i + 1])
    #    cv2.waitKey(1)

    @staticmethod
    def _convert_images_to_tensors(qry, ref):
        """Converts grayscale images to torch tensors"""
        qry_tensor = torch.Tensor(cv2.cvtColor(qry, cv2.COLOR_BGR2GRAY)[None, None]).cuda() / 255.0
        ref_tensor = torch.Tensor(cv2.cvtColor(ref, cv2.COLOR_BGR2GRAY)[None, None]).cuda() / 255.0
        return qry_tensor, ref_tensor

    def inference(self, preprocessed_data):
        """Do keypoint matching."""
        with torch.no_grad():
            results = self._model(preprocessed_data[0])
        return results, *preprocessed_data[1:]

    def postprocess(self, inferred_data) -> Optional[Pose]:
        """Filters matches based on confidence threshold and calculates :term:`pose`"""

        def _postprocess(camera_info: CameraInfo) -> Optional[Pose]:
            results, query_img, reference_img, elevation = inferred_data
            mkp_qry, mkp_ref = self._filter_matches_based_on_confidence(results)

            if mkp_qry is None or len(mkp_qry) < self.MIN_MATCHES:
                return None

            k_matrix = camera_info.k.reshape((3, 3))

            mkp2_3d = self._compute_3d_points(mkp_ref, elevation)
            pose = self._compute_pose(mkp2_3d, mkp_qry, k_matrix)
            self._visualize_matches_and_pose(
                query_img, reference_img, mkp_qry, mkp_ref, k_matrix, *pose
            )

            pose_msg = Pose()

            # Convert the rotation matrix to a quaternion
            quaternion = tf_transformations.quaternion_from_matrix(pose[0])
            pose_msg.orientation = Quaternion(*quaternion)

            # Populate the translation (position) fields
            pose_msg.position = Point(*pose[1].flatten())

        return _postprocess(self.camera_info)

    def _filter_matches_based_on_confidence(self, results):
        """Filters matches based on confidence threshold"""
        conf = results["confidence"].cpu().numpy()
        valid = conf > self.CONFIDENCE_THRESHOLD
        return (
            results["keypoints0"].cpu().numpy()[valid, :],
            results["keypoints1"].cpu().numpy()[valid, :],
        )

    @staticmethod
    def _compute_3d_points(mkp_ref, elevation):
        """Computes 3D points from matches"""
        if elevation is None:
            return np.hstack((mkp_ref, np.zeros((len(mkp_ref), 1))))

        x, y = np.transpose(np.floor(mkp_ref).astype(int))
        z_values = elevation[y, x].reshape(-1, 1)
        return np.hstack((mkp_ref, z_values))

    @staticmethod
    def _compute_pose(mkp2_3d, mkp_qry, k_matrix):
        """Computes :term:`pose` using :func:`cv2.solvePnPRansac`"""
        dist_coeffs = np.zeros((4, 1))
        _, r, t, _ = cv2.solvePnPRansac(
            mkp2_3d,
            mkp_qry,
            k_matrix,
            dist_coeffs,
            useExtrinsicGuess=False,
            iterationsCount=10,
        )
        r_matrix, _ = cv2.Rodrigues(r)
        return r_matrix, t

    def _visualize_matches_and_pose(self, qry, ref, mkp_qry, mkp_ref, k, r, t):
        """Visualizes matches and projected :term:`FOV`"""
        h_matrix = k @ np.delete(np.hstack((r, t)), 2, 1)
        projected_fov = self._project_fov(qry, h_matrix)
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

        cv2.imshow("Matches and field of view", match_img)
        cv2.waitKey(1)

    @staticmethod
    def _project_fov(img, h_matrix):
        """Projects :term:`FOV` on :term:`reference` image"""
        height, width = img.shape[0:2]
        src_pts = np.float32(
            [[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]]
        ).reshape(-1, 1, 2)
        try:
            return cv2.perspectiveTransform(src_pts, np.linalg.inv(h_matrix))
        except np.linalg.LinAlgError:
            return src_pts
