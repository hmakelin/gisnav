"""This module contains :class:`.PoseNode`, a :term:`ROS` node for estimating
:term:`camera` relative pose between a :term:`query` and :term:`reference` image

The pose is estimated by finding matching keypoints between the query and reference
images and then solving the resulting :term:`PnP` problem.

.. mermaid::
    :caption: :class:`.PoseNode` computational graph

    graph LR
        subgraph PnPNode
            pose[gisnav/pose_node/pose]
        end

        subgraph TransformNode
            image[gisnav/transform_node/image]
        end

        subgraph gscam
            camera_info[camera/camera_info]
        end

        camera_info -->|sensor_msgs/CameraInfo| PoseNode
        image -->|sensor_msgs/Image| PoseNode
        pose -->|geometry_msgs/PoseStamped| MockGPSNode:::hidden
"""
from typing import Optional, Tuple
from copy import deepcopy

import rclpy
import cv2
import numpy as np
import torch
import tf2_ros
import tf_transformations

from cv_bridge import CvBridge
from kornia.feature import LoFTR
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros.transform_broadcaster import TransformBroadcaster


from .. import messaging
from ..constants import (
   ROS_NAMESPACE,
   ROS_TOPIC_RELATIVE_PNP_IMAGE,
   TRANSFORM_NODE_NAME,
)
from ..decorators import ROS, narrow_types


class PoseNode(Node):
    """Solves the keypoint matching and :term:`PnP` problems and publishes the
    solution via ROS transformations library
    """

    CONFIDENCE_THRESHOLD = 0.7
    """Confidence threshold for filtering out bad keypoint matches"""

    MIN_MATCHES = 7
    """Minimum number of keypoint matches before attempting pose estimation"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        self._model = LoFTR(pretrained="outdoor")
        self._model.to(self._device)

        self._cv_bridge = CvBridge()

        # initialize subscription
        self.camera_info
        self.image

        # Initialize the transform broadcaster
        self.broadcaster = TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    @property
    # @ROS.max_delay_ms(messaging.DELAY_SLOW_MS) - gst plugin does not enable timestamp?
    @ROS.subscribe(messaging.ROS_TOPIC_CAMERA_INFO, QoSPresetProfiles.SENSOR_DATA.value)
    def camera_info(self) -> Optional[CameraInfo]:
        """Camera info for determining appropriate :attr:`.orthoimage` resolution"""

    def _image_cb(self, msg: Image) -> None:
        """Callback for :attr:`.image` message"""
        preprocessed = self.preprocess(msg)
        inferred = self.inference(preprocessed)
        pose_stamped = self.postprocess(inferred)

        if pose_stamped is None:
            return None

        r, t = pose_stamped

        t[2] = -t[2]

        transform_camera = messaging.create_transform_msg(
            msg.header.stamp, "world", "camera", -r, (-r @ t).squeeze()
        )
        self.broadcaster.sendTransform([transform_camera])

        debug_msg = messaging.get_transform(self, "world", "camera",
                                           rclpy.time.Time())

        # The child frame is the 'camera' frame of the PnP problem as
        # defined here: https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html
        if debug_msg is not None:
            debug_ref_image = self._cv_bridge.imgmsg_to_cv2(
                deepcopy(msg), desired_encoding="passthrough"
            )

            t = np.array((debug_msg.transform.translation.x, debug_msg.transform.translation.y,
                          debug_msg.transform.translation.z))

            debug_ref_image = debug_ref_image[:, :, 1]  # seocnd channel is ref (world) image
            # current image timestamp does not yet have the transform but this should get the previous one
            x, y = int(t[0]), int(self.camera_info.height - t[1])  # move height origin from bottom to top left for cv2
            debug_ref_image = cv2.circle(np.array(debug_ref_image), (x, y), 5, (0, 255, 0), -1)
            cv2.imshow("Camera position in world frame", debug_ref_image)
            cv2.waitKey(1)

    @property
    @ROS.max_delay_ms(messaging.DELAY_DEFAULT_MS)
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_PNP_IMAGE.replace("~", TRANSFORM_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=_image_cb,
    )
    def image(self) -> Optional[Image]:
        """term:`Query`, :term:`reference`, and :term:`elevation` image
        in a single 4-channel :term:`stack`. The query image is in the first
        channel, the reference image is in the second, and the elevation reference
        is in the last two (sum them together to get a 16-bit elevation reference).

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

    @narrow_types
    def preprocess(
        self, image_quad: Image
    ) -> Tuple[dict, np.ndarray, np.ndarray, np.ndarray]:
        """Converts incoming 4-channel image to torch tensors

        :param image_quad: A 4-channel image where the first channel is the
            :term:`Query`, the second channel is the 8-bit
            :term:`elevation reference`, and the last two channels combined
            represent the 16-bit :term:`elevation reference`.
        """
        # Convert the ROS Image message to an OpenCV image
        full_image_cv = self._cv_bridge.imgmsg_to_cv2(
            image_quad, desired_encoding="passthrough"
        )

        # Check that the image has 4 channels
        channels = full_image_cv.shape[2]
        assert channels == 4, "The image must have 4 channels"

        # Extract individual channels
        query_img = full_image_cv[:, :, 0]
        reference_img = full_image_cv[:, :, 1]
        elevation_16bit_high = full_image_cv[:, :, 2]
        elevation_16bit_low = full_image_cv[:, :, 3]

        # Reconstruct 16-bit elevation from the last two channels
        reference_elevation = (
            elevation_16bit_high.astype(np.uint16) << 8
        ) | elevation_16bit_low.astype(np.uint16)

        # Optionally display images
        #self._display_images("Query", query_img, "Reference", reference_img)

        if torch.cuda.is_available():
            qry_tensor = torch.Tensor(query_img[None, None]).cuda() / 255.0
            ref_tensor = torch.Tensor(reference_img[None, None]).cuda() / 255.0
        else:
            qry_tensor = torch.Tensor(query_img[None, None]) / 255.0
            ref_tensor = torch.Tensor(reference_img[None, None]) / 255.0

        return (
            {"image0": qry_tensor, "image1": ref_tensor},
            query_img,
            reference_img,
            reference_elevation,
        )

    @staticmethod
    def _display_images(*args):
        """Displays images using OpenCV"""
        for i in range(0, len(args), 2):
            cv2.imshow(args[i], args[i + 1])
        cv2.waitKey(1)

    def inference(self, preprocessed_data):
        """Do keypoint matching."""
        with torch.no_grad():
            results = self._model(preprocessed_data[0])
        return results, *preprocessed_data[1:]

    def postprocess(self, inferred_data) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """Filters matches based on confidence threshold and calculates :term:`pose`"""

        @narrow_types(self)
        def _postprocess(
            camera_info: CameraInfo, inferred_data
        ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
            results, query_img, reference_img, elevation = inferred_data

            conf = results["confidence"].cpu().numpy()
            valid = conf > self.CONFIDENCE_THRESHOLD
            mkp_qry = results["keypoints0"].cpu().numpy()[valid, :]
            mkp_ref = results["keypoints1"].cpu().numpy()[valid, :]

            if mkp_qry is None or len(mkp_qry) < self.MIN_MATCHES:
                return None

            k_matrix = camera_info.k.reshape((3, 3))

            mkp2_3d = self._compute_3d_points(mkp_ref, elevation)
            # Adjust y-axis for ROS convention (origin is bottom left, not top left),
            # elevation (z) coordinate remains unchanged
            mkp2_3d[:, 1] = camera_info.height - mkp2_3d[:, 1]
            mkp_qry[:, 1] = camera_info.height - mkp_qry[:, 1]
            r, t = self._compute_pose(mkp2_3d, mkp_qry, k_matrix)

            self._visualize_matches_and_pose(
                query_img.copy(), reference_img.copy(), mkp_qry, mkp_ref, k_matrix, r, t
            )

            return r, t

        return _postprocess(self.camera_info, inferred_data)

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

        # We modify these from ROS to cv2 axes convention so we create copies
        mkp_qry = mkp_qry.copy()
        mkp_ref = mkp_ref.copy()

        h_matrix = k @ np.delete(np.hstack((r, t)), 2, 1)
        projected_fov = self._project_fov(qry, h_matrix)

        # Invert the y-coordinate, considering the image height (input r and t
        # are in ROS convention where origin is at bottom left of image, we
        # want origin to be at top left for cv2
        h = self.camera_info.height
        mkp_ref[:, 1] = mkp_ref[:, 1]
        mkp_qry[:, 1] = h - mkp_qry[:, 1]

        projected_fov[:, :, 1] = h - projected_fov[:, :, 1]
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
