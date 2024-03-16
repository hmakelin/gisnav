"""This module contains :class:`.PoseNode`, a :term:`ROS` node for estimating
:term:`camera` relative pose between a :term:`query` and :term:`reference` image.

The reference image can be either a orthoimagery :term:`raster` from the onboard GIS
server (deep map matching for noisy global position), or a previous image frame form
the camera (visual odometry for smooth relative position).

The pose is estimated by finding matching keypoints between the query and
reference images and then solving the resulting :term:`PnP` problem.

``image`` frame is the image plane as defined in the OpenCV `PnP problem
<https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html>`_. ``reference`` frame here
refers exclusively to the orthoimage reference frame, not the visual odometry reference
frame.

We cache the ``image`` to reference frame transformation so that we can connect the
poses from the visual odometry transformation chain to reference frame at every query
frame. We also cache query frame timestamp (included in message Header) so that we can
connect it to the correct reference frame.

.. todo::
    Come up with a distinct names for orthoimagery and visual odometry matching
    reference frames (e.g. query and reference, and query and previous?)
"""
from copy import deepcopy
from typing import Optional, Tuple, Any

import cv2
import numpy as np
import rclpy
import tf2_ros
import torch
import tf_transformations

from cv_bridge import CvBridge
from kornia.feature import LoFTR
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import CameraInfo, Image, TimeReference
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf2_ros.transform_broadcaster import TransformBroadcaster
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time

from .. import _transformations as tf_
from .._decorators import ROS, narrow_types
from ..constants import (
    DELAY_DEFAULT_MS,
    MAVROS_TOPIC_TIME_REFERENCE,
    ROS_NAMESPACE,
    ROS_TOPIC_CAMERA_INFO,
    ROS_TOPIC_RELATIVE_PNP_IMAGE,
    ROS_TOPIC_RELATIVE_STEREO_IMAGE,
    STEREO_NODE_NAME,
    FrameID,
)


class PoseNode(Node):
    """Solves the keypoint matching and :term:`PnP` problems and publishes the
    solution to ROS
    """

    CONFIDENCE_THRESHOLD = 0.7
    """Confidence threshold for filtering out bad keypoint matches"""

    MIN_MATCHES = 20
    """Minimum number of keypoint matches before attempting pose estimation"""

    def __init__(self, *args, **kwargs):
        """Class initializer

        :param args: Positional arguments to parent :class:`.Node` constructor
        :param kwargs: Keyword arguments to parent :class:`.Node` constructor
        """
        super().__init__(*args, **kwargs)
        self._device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

        # Initialize DL model for map matching (noisy global position, no drift)
        self._model = LoFTR(pretrained="outdoor")
        self._model.to(self._device)

        # Initialize ORB detector and brute force matcher for VO
        # (smooth relative position with drift)
        self._orb = cv2.ORB_create()
        self._bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=False)

        self._cv_bridge = CvBridge()

        # initialize subscriptions
        self.camera_info
        self.image_ortho
        self.image_vo
        self.time_reference

        self._initialize_tf()
        self._publish_camera_to_camera_optical()
        self._init_vo_cache()

    def _initialize_vo_cache(self):
        """Initialize cached attributes needed to make visual odometry work"""
        # Cached camera_optical frame to world frame transformation (i.e. projection
        # matrix consisting of inversion of both pose and camera intrinsics) as defined
        # in cv2.solvePnP assuming the REP 103 "camera_optical" frame corresponds to the
        # solvePnP "camera" frame.
        self.pose_camera_optical_in_previous_vo_world: Optional[PoseStamped] = None

        # Cached camera pose in gisnav_world frame coordinates. "pnp_world_vo" is a
        # translated image plane for the previous query image, using the cv2.solvePnP
        # terminology.
        self._camera_optical_pose_gisnav_world: Optional[PoseStamped] = None

    def _initialize_tf(self):
        """Initializes the ``tf`` and ``tf_static`` topics and the listening buffer"""
        self.broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def _publish_camera_optical_to_camera(self):
        """Camera_optical frame to camera frame transformation
        as defined in `REP 103 <https://www.ros.org/reps/rep-0103.html#axis-orientation>`_

        The camera_optical as defined by REP 103 is also the same one used by cv2
        for the camera frame in the `PnP problem
        <https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html>`_.
        """
        assert self.static_broadcaster is not None, \
            "Please initialize the static transform broadcaster first"

        q = (0.5, 0.5, -0.5, -0.5)
        header = tf_.create_header(
            self, "", self.time_reference
        )  # time reference is not important here
        transform_camera = tf_.create_transform_msg(
            header.stamp, "camera_optical", "camera", q, np.zeros(3)
        )
        self.static_broadcaster.sendTransform([transform_camera])

    @property
    @ROS.subscribe(
        MAVROS_TOPIC_TIME_REFERENCE,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def time_reference(self) -> Optional[TimeReference]:
        """:term:`FCU` time reference via :term:`MAVROS`"""

    @property
    # @ROS.max_delay_ms(messaging.DELAY_SLOW_MS) - gst plugin does not enable timestamp?
    @ROS.subscribe(
        ROS_TOPIC_CAMERA_INFO,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def camera_info(self) -> Optional[CameraInfo]:
        """Camera info message including the camera intrinsics matrix"""

    def _image_cb(self, msg: Image) -> None:
        """Callback for :attr:`.image` message"""
        qry, ref, dem = self._preprocess(msg)
        mkp_qry, mkp_ref = self.inference(qry, ref, dem)
        r, t = self._postprocess(qry, ref, dem, mkp_qry, mkp_ref, "Deep match / global position (GIS)")

        pose_camera_optical_in_world = tf_.create_pose_msg(msg.header.stamp, "world", r, t)

        if pose_camera_optical_in_world is None:
            return None
        else:
            # Cache latest camera_optical frame pose in PnP world coordinates for
            #  linking up VO with world coordinates in the other callback. Matching by
            #  exact timestamp is important as the reference and therefore also world
            #  frame are discontinous relative to the VO image frames.
            self._camera_optical_pose_gisnav_world = pose_camera_optical_in_world

        transform_world_to_camera_optical = tf_.pose_to_transform(
            pose_camera_optical_in_world,"world", "camera_optical"
        )

        # TODO: implement this as a computed property, not a method
        #self.pose(pose_stamped)

        self.broadcaster.sendTransform([transform_world_to_camera_optical])

        # TODO: migrate debug visualizations to rviz (easier to do entire 3D pose
        #  instead of 2D position only)
        tf_.visualize_camera_position(
            ref.copy(),
            t,
            "Camera position in world frame",
        )

    @ROS.publish(
        "~/camera/pose_stamped",
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def pose(self, msg: PoseStamped) -> Optional[PoseStamped]:
        """Camera pose in world frame"""
        # TODO fix this implementation - make derived/computed property not method
        return msg

    @ROS.publish(
        "~/camera/vo/odometry",
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def odometry(self, msg: Odometry) -> Optional[Odometry]:
        """Odometry in odom frame"""
        # TODO convert odometry to a timestamped reference frame and fix scaling (meters) for odom frame
        # TODO fix this implementation - make derived/computed property not method
        return msg

    @ROS.publish(
        "~/camera/vo/pose_stamped",
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def pose_previous_query(self, msg: PoseStamped) -> Optional[PoseStamped]:
        """Camera pose in previous_query frame"""
        # TODO fix this implementation - make derived/computed property not method
        return msg

    @property
    @ROS.max_delay_ms(DELAY_DEFAULT_MS)
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_PNP_IMAGE.replace("~", STEREO_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=_image_cb,
    )
    def image(self) -> Optional[Image]:
        """:term:`Query <query>`, :term:`reference`, and :term:`elevation` image
        in a single 4-channel :term:`stack`. The query image is in the first
        channel, the reference image is in the second, and the elevation reference
        is in the last two (sum them together to get a 16-bit elevation reference).


        .. note::
            The existing :class:`sensor_msgs.msg.Image` message is repurposed
            to represent a stereo couple with depth information (technically a
            triplet) to avoid having to introduce custom messages that would
            have to be distributed in a separate package. It will be easier to
            package this later as a rosdebian if everything is already in the
            rosdep index.
        """

    def _image_vo_cb(self, msg: Image) -> None:
        """Callback for :attr:`.image` message"""
        qry, ref, dem = self._preprocess(msg, shallow_inference=True)
        mkp_qry, mkp_ref = self.inference(qry, ref, shallow_inference=True)
        r, t = self._postprocess(qry, ref, dem, mkp_qry, mkp_ref, "Shallow match / relative position (monocular VO)")

        stamp = self._get_stamp(msg)  # TODO: should use previous query frame timestamp?
        pnp_world_frame_id: FrameID = "pnp_world_%i_%i"
        pnp_world_frame_id_timestamped = pnp_world_frame_id % (stamp.sec, stamp.nanosec)
        pose_camera_optical_in_current_vo_world = (
            tf_.create_pose_msg(msg.header.stamp, pnp_world_frame_id_timestamped, r, t)
        )
        if pose_camera_optical_in_current_vo_world is None:
            # We could not obtain a good VO match
            return None

        #self.pose_previous_query(camera_pose)

        if self._camera_optical_pose_in_previous_vo_world is not None:

            # Publish VO frames relative transform to tf2
            pose_diff = self._pose_diff(self.camera_info, pose_camera_optical_in_current_vo_world, self._camera_optical_pose_in_previous_vo_world)
            previous_stamp = self._get_stamp(self._camera_optical_pose_in_previous_vo_world)
            transform = tf_.pose_to_transform(
                pose_diff,
                pose_diff.header.frame_id, # refactor out frame_id input argument
                "pnp_world_%i_%i" % (previous_stamp.sec, previous_stamp.nanosec)
            )
            self.broadcaster.sendTransform([transform])  # query to camera_rfu

            # TODO: get cached camera pose in
            past_camera_pose_in_gisnav_world = self._camera_optical_pose_gisnav_world
            linking_stamp = self._get_stamp(past_camera_pose_in_gisnav_world)
            target_frame_id = "pnp_world_%i_%i" % (linking_stamp.sec, linking_stamp.nanosec)
            transform_to_linking_query_frame = tf_.get_transform(
                self, target_frame_id, pose_diff.header.frame_id, rclpy.time.Time()
            )
            H, r, t = tf_.pose_stamped_to_matrices(
                tf_.transform_to_pose(transform_to_linking_query_frame)
            )
            H_, r_, t_ = tf_.pose_stamped_to_matrices(self._camera_optical_pose_gisnav_world)
            current_camera_pose_in_gisnav_world = H @ H_

            current_camera_pose_in_gisnav_world = tf_.create_pose_msg(pose_diff.header.stamp)

            # TODO scale values by M[2,2] - this is currently in reference frame units
            #self.odometry(odometry_msg)

        self._camera_optical_pose_in_previous_vo_world = pose_camera_optical_in_current_vo_world

    def _pose_diff(self, current_pose: PoseStamped, previous_pose: PoseStamped) -> PoseStamped:
        """Returns pose difference between two Poses assuming the current pose
        frame_id has the same timestamp as the previous pose (assuming poses are
        successive poses from VO

        The visual odometry transformation chain goes like this:

        .. mermaid::
            graph TB
                previous_camera_pose_in_previous_world_xy -->|"inv(previous_projection_matrix)"| previous_camera_pose_in_previous_plane_uv
                previous_camera_pose_in_previous_plane_uv -->|"translation(center to top-left)"| previous_camera_pose_in_current_world_xy

        .. note::
            We do not use ``tf2`` here for transformations because these transforms
            involve projection matrices and therefore cannot be assumed to be rigid
            transformations.
        """

        @narrow_types(self)
        def _pose_diff(camera_info: CameraInfo, current_pose: PoseStamped, previous_pose: PoseStamped) -> Optional[PoseStamped]:
            # Transform from previous image plane to current world frame (translation
            # of origin from center to top-left corner)
            t_previous_plane_uv_to_current_world_xy = np.ndarray([-camera_info.width / 2, -camera_info.height / 2, 0])

            k = self.camera_info.k.reshape((3, 3))
            # H_current, _, _ = tf_.pose_stamped_to_matrices(current_pose)
            H_previous, _, _ = tf_.pose_stamped_to_matrices(previous_pose)
            # current_projection_matrix = k @ H_previous
            previous_projection_matrix = k @ H_previous

            try:
                inv_previous_projection_matrix = np.linalg.inv(previous_projection_matrix)
            except np.linalg.LinAlgError:
                return None

            previous_pose_in_previous_plane_uv = inv_previous_projection_matrix @ H_previous
            previous_pose_in_current_world_xy = previous_pose_in_previous_plane_uv + t_previous_plane_uv_to_current_world_xy

            pose_difference = current_pose
            pose_difference.pose.position.x = current_pose.pose.position.x - previous_pose.pose.position.x
            pose_difference.pose.position.y = current_pose.pose.position.y - previous_pose.pose.position.y
            pose_difference.pose.position.z = current_pose.pose.position.z - previous_pose.pose.position.z

            # Calculate orientation difference
            current_quaternion = [
                current_pose.pose.orientation.x,
                current_pose.pose.orientation.y,
                current_pose.pose.orientation.z,
                current_pose.pose.orientation.w
            ]

            previous_quaternion = [
                previous_pose.pose.orientation.x,
                previous_pose.pose.orientation.y,
                previous_pose.pose.orientation.z,
                previous_pose.pose.orientation.w
            ]

            # Quaternion conjugate and multiplication to find relative rotation
            quaternion_diff = tf_transformations.quaternion_conjugate(previous_quaternion)
            orientation_difference = tf_transformations.quaternion_multiply(
                quaternion_diff, current_quaternion)

            current_pose.pose.orientation = orientation_difference

            return pose_difference

        return _pose_diff(self.camera_info, current_pose, previous_pose)

    @property
    @ROS.max_delay_ms(DELAY_DEFAULT_MS)
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_STEREO_IMAGE.replace("~", STEREO_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=_image_vo_cb,
    )
    def image_vo(self) -> Optional[Image]:
        """Image pair consisting of query image and reference image for :term:`VO` use.

        .. note::
            Images are set side by side - the first or left image is the current (query)
            image, while the second or right image is the previous (reference) image.
        """

    @property
    @ROS.max_delay_ms(DELAY_DEFAULT_MS)
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_PNP_IMAGE.replace("~", STEREO_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=_image_cb,
    )
    def image(self) -> Optional[Image]:
        """:term:`Query <query>`, :term:`reference`, and :term:`elevation` image
        in a grayscale image. The query image is the first (left) image, and the reference
        image is the second (right) image.

        .. note::
            The existing :class:`sensor_msgs.msg.Image` message is repurposed
            to represent a stereo couple to avoid having to introduce custom messages
            that would have to be distributed in a separate package. It will be easier to
            package this later as a rosdebian if everything is already in the
            rosdep index.
        """

    @narrow_types
    def _preprocess(
        self, stereo_image: Image, shallow_inference: bool = False,
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Converts sensor_msgs/Image message to numpy arrays

        :param stereo_image: A 4-channel image where the first channel is the
            :term:`query`, the second channel is the 8-bit
            :term:`elevation reference`, and the last two channels combined
            represent the 16-bit :term:`elevation reference` (for deep matching/absolute
            global position), or a 1-channel image where the left side is the query
            image, and the right side is the reference image (for VO/shallow matching/
            relative position)
        :param shallow_inference: If set to True, prepare for faster matching method
            suitable for visual odometry (VO) instead of for deep learning model

        :return: A 3-tuple/triplet of query image, reference image, and DEM rasters
        """
        # Convert the ROS Image message to an OpenCV image
        full_image_cv = self._cv_bridge.imgmsg_to_cv2(
            stereo_image, desired_encoding="passthrough"
        )

        if not shallow_inference:
            # Check that the image has 4 channels
            channels = full_image_cv.shape[2]
            assert channels == 4, \
                (f"The input image for deep matching against orthoimage is expected to "
                 f"have 4 channels - {channels} received instead.")

            # Extract individual channels
            query_img = full_image_cv[:, :, 0]
            reference_img = full_image_cv[:, :, 1]
            elevation_16bit_high = full_image_cv[:, :, 2]
            elevation_16bit_low = full_image_cv[:, :, 3]

            # Reconstruct 16-bit elevation from the last two channels
            reference_elevation = (
                elevation_16bit_high.astype(np.uint16) << 8
            ) | elevation_16bit_low.astype(np.uint16)

            return (
                query_img,
                reference_img,
                reference_elevation,
            )
        else:
            ndim = full_image_cv.ndim
            assert ndim == 2, \
                (f"The input image for shallow matching against previous image is "
                 f"expected to have 2 dimensions - {ndim} received instead.")

            h, w = full_image_cv.shape

            # this image should consist of two images placed side-by-side
            assert w % 2 == 0, \
                (f"The input image for shallow matching against previous image should "
                f"consist of two images side by side making width divisible by two.")

            half_w = int(w/2)
            query_img = full_image_cv[:, :half_w]
            reference_img = full_image_cv[:, half_w:]
            return query_img, reference_img, np.zeros_like(reference_img)

    def _process(self, qry: np.ndarray, ref: np.ndarray,
                 shallow_inference: bool = False) -> Tuple[np.ndarray, np.ndarray]:
        """Returns keypoint matches for input image pair

        :return: Tuple of matched query image keypoints, and matched reference image
            keypoints
        """
        if not shallow_inference:#

            if torch.cuda.is_available():
                qry_tensor = torch.Tensor(qry[None, None]).cuda() / 255.0
                ref_tensor = torch.Tensor(ref[None, None]).cuda() / 255.0
            else:
                self.get_logger().warning("CUDA not available - using CPU.")
                qry_tensor = torch.Tensor(qry[None, None]) / 255.0
                ref_tensor = torch.Tensor(ref[None, None]) / 255.0

            with torch.no_grad():
                results = self._model({"image0": qry_tensor, "image1": ref_tensor})

            conf = results["confidence"].cpu().numpy()
            good = conf > self.CONFIDENCE_THRESHOLD
            mkp_qry = results["keypoints0"].cpu().numpy()[good, :]
            mkp_ref = results["keypoints1"].cpu().numpy()[good, :]

            return mkp_qry, mkp_ref
        else:
            # find the keypoints and descriptors with ORB
            kp_qry, desc_qry = self._orb.detectAndCompute(qry, None)
            kp_ref, desc_ref = self._orb.detectAndCompute(ref, None)

            matches = self._bf.knnMatch(desc_qry, desc_ref, k=2)

            # Apply ratio test
            good = []
            for m, n in matches:
                # TODO: have a separate confidence threshold for shallow and deep
                #  keypoint matching?
                if m.distance < self.CONFIDENCE_THRESHOLD * n.distance:
                    good.append(m)

            mkps = list(
                map(
                    lambda dmatch: (
                        tuple(kp_qry[dmatch.queryIdx].pt),
                        tuple(kp_ref[dmatch.trainIdx].pt)
                    ),
                    good
                )
            )

            if len(mkps) > 0:
                mkp_qry, mkp_ref = zip(*mkps)
            else:
                return None
            mkp_qry = np.array(mkp_qry)
            mkp_ref = np.array(mkp_ref)

            return mkp_qry, mkp_ref

    def _postprocess(self, mkp_qry: np.ndarray, mkp_ref: np.ndarray, elevation: np.ndarray, qry_img: np.ndarray, ref_img: np.ndarray, label: str, stamp: Time, frame_id: str) -> Optional[PoseStamped]:
        """Computes camera pose from keypoint matches"""

        @narrow_types(self)
        def _visualize_matches_and_pose(camera_info, qry, ref, mkp_qry, mkp_ref,
                                        k, r, t, label) -> None:
            """Visualizes matches and projected :term:`FOV`"""

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

            # We modify these from ROS to cv2 axes convention so we create copies
            # mkp_qry = mkp_qry.copy()
            # mkp_ref = mkp_ref.copy()

            h_matrix = k @ np.delete(np.hstack((r, t)), 2, 1)
            projected_fov = _project_fov(qry, h_matrix)

            # Invert the y-coordinate, considering the image height (input r and t
            # are in ROS convention where origin is at bottom left of image, we
            # want origin to be at top left for cv2
            h = camera_info.height
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

            cv2.imshow(label, match_img)
            cv2.waitKey(1)

        @narrow_types(self)
        def _compute_pose(
            camera_info: CameraInfo, mkp_qry: np.ndarray, mkp_ref: np.ndarray, elevation: np.ndarray, qry_img: np.ndarray, ref_img: np.ndarray, label: str
        ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
            if len(mkp_qry) < self.MIN_MATCHES:
                self.get_logger().debug("Not enough matches - returning None")
                return None

            def _compute_3d_points(mkp_ref: np.ndarray, elevation: np.ndarray) -> np.ndarray:
                """Computes 3D points from matches"""
                if elevation is None:
                    return np.hstack((mkp_ref, np.zeros((len(mkp_ref), 1))))

                x, y = np.transpose(np.floor(mkp_ref).astype(int))
                z_values = elevation[y, x].reshape(-1, 1)
                return np.hstack((mkp_ref, z_values))

            def _compute_pose(mkp2_3d: np.ndarray, mkp_qry: np.ndarray, k_matrix: np.ndarray) -> Tuple[
                np.ndarray, np.ndarray]:
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
                t = np.ndarray(t)

                return r_matrix, t

            k_matrix = camera_info.k.reshape((3, 3))

            mkp2_3d = _compute_3d_points(mkp_ref, elevation)

            # Adjust y-axis for ROS convention (origin is bottom left, not top left),
            # elevation (z) coordinate remains unchanged
            #mkp2_3d[:, 1] = camera_info.height - mkp2_3d[:, 1]
            #mkp_qry[:, 1] = camera_info.height - mkp_qry[:, 1]

            r, t = _compute_pose(mkp2_3d, mkp_qry, k_matrix)

            _visualize_matches_and_pose(
                qry_img.copy(), ref_img.copy(), mkp_qry, mkp_ref, k_matrix, r, t, label
            )

            return r, t

        return _compute_pose(self.camera_info, mkp_qry, mkp_ref, elevation, qry_img, ref_img, label)

    def _get_stamp(self, msg) -> Time:
        if self.time_reference is None:
            self.get_logger().warning(
                "Publishing world to camera_pinhole transformation without time "
                "reference."
            )
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
