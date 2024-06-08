"""This module contains :class:`.StereoNode`, a ROS node that generates and publishes a
synthetic query and reference stereo image couple.

Synthetic refers to the fact that no stereo camera is actually assumed or required.
The reference can be an older image from the same monocular camera, or alternatively an
aligned orthoimage and DEM raster from the GIS server.

Alignment and cropping to the same dimension as the query image is done to the
reference orthoimage by using information of the onboard camera resolution and the
vehicle's heading. Alignment  is required since the deep learning network that is used
for matching keypoints is not assumed to be rotation agnostic.
"""
from typing import Final, Optional, Tuple

import cv2
import numpy as np
import rclpy
import tf2_ros
import tf_transformations
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from gisnav_msgs.msg import (  # type: ignore[attr-defined]
    MonocularStereoImage,
    OrthoImage,
    OrthoStereoImage,
)
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from .. import _transformations as tf_
from .._decorators import ROS, narrow_types
from ..constants import (
    GIS_NODE_NAME,
    ROS_NAMESPACE,
    ROS_TOPIC_CAMERA_INFO,
    ROS_TOPIC_IMAGE,
    ROS_TOPIC_RELATIVE_ORTHOIMAGE,
    ROS_TOPIC_RELATIVE_POSE_IMAGE,
    ROS_TOPIC_RELATIVE_TWIST_IMAGE,
)


class StereoNode(Node):
    """Generates and publishes a synthetic query and reference stereo image couple."""

    _ROS_PARAM_DESCRIPTOR_READ_ONLY: Final = ParameterDescriptor(read_only=True)
    """A read only ROS parameter descriptor"""

    def __init__(self, *args, **kwargs) -> None:
        """Class initializer

        :param args: Positional arguments to parent :class:`.Node` constructor
        :param kwargs: Keyword arguments to parent :class:`.Node` constructor
        """
        super().__init__(*args, **kwargs)

        # Converts image_raw to cv2 compatible image
        self._cv_bridge = CvBridge()

        # Calling these decorated properties the first time will setup
        # subscriptions to the appropriate ROS topics
        self.orthoimage
        self.camera_info
        self.image

        # TODO Declare as property?
        self.previous_image: Optional[Image] = None

        # setup publisher to pass launch test without image callback being
        # triggered
        self.pose_image
        # self.twist_image

        # Initialize the transform broadcaster and listener
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

    @property
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_ORTHOIMAGE.replace("~", GIS_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def orthoimage(self) -> Optional[OrthoImage]:
        """Subscribed orthoimage, or None if unknown"""

    @property
    # @ROS.max_delay_ms(messaging.DELAY_SLOW_MS) - gst plugin does not enable timestamp?
    @ROS.subscribe(
        ROS_TOPIC_CAMERA_INFO,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def camera_info(self) -> Optional[CameraInfo]:
        """Subscribed camera info for determining appropriate :attr:`.orthoimage` crop
        resolution, or None if unknown"""

    def _image_cb(self, msg: Image) -> None:
        """Callback for :attr:`.image` message"""
        self.pose_image  # publish rotated and cropped orthoimage stack

        # self.twist_image  # publish two subsequent images for VO

        # TODO this is brittle - nothing is enforcing that this is assigned after
        #  publishing stereo_image
        self.previous_image = (
            msg  # needed for VO - leave this for last in this callback
        )

    @property
    # @ROS.max_delay_ms(messaging.DELAY_FAST_MS) - gst plugin does not enable timestamp?
    @ROS.subscribe(
        ROS_TOPIC_IMAGE,
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=_image_cb,
    )
    def image(self) -> Optional[Image]:
        """Subscribed raw image from vehicle camera, or None if unknown"""

    def _world_to_reference_proj_str(
        self,
        M: np.ndarray,
        crs: str,
    ) -> Optional[str]:
        @narrow_types(self)
        def _transform(
            M: np.ndarray,
            crs: str,
        ) -> Optional[TransformStamped]:
            # 3D version of the inverse rotation and cropping transform
            M_3d = np.eye(4)
            M_3d[:2, :2] = M[:2, :2]
            M_3d[:2, 3] = M[:2, 2]

            try:
                tf_transformations.quaternion_from_matrix(M_3d)
            except np.linalg.LinAlgError:
                self.get_logger().warning(
                    "_pnp_image: Could not compute quaternion from estimated rotation. "
                    "Returning None."
                )
                return None

            # TODO clean this up
            M = tf_.proj_to_affine(crs)
            # Flip x and y in between to make this transformation chain work
            T = np.array([[0, 1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            compound_transform = M @ T @ np.linalg.inv(M_3d)
            proj_str = tf_.affine_to_proj(compound_transform)

            return proj_str

        return _transform(M, crs)

    @property
    @ROS.publish(
        ROS_TOPIC_RELATIVE_POSE_IMAGE,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def pose_image(self) -> Optional[OrthoStereoImage]:
        """Published aligned and cropped orthoimage consisting of query image,
        reference image, and optional reference elevation raster (DEM).
        """

        @narrow_types(self)
        def _pnp_image(
            image: Image,
            orthoimage: OrthoImage,
            transform: TransformStamped,
        ) -> Optional[OrthoStereoImage]:
            """Rotate and crop and orthoimage stack to align with query image"""
            transform = transform.transform

            orthoimage_arr = self._cv_bridge.imgmsg_to_cv2(
                orthoimage.image, desired_encoding="passthrough"
            )
            dem_arr = self._cv_bridge.imgmsg_to_cv2(
                orthoimage.dem, desired_encoding="mono8"
            )
            orthoimage_arr = cv2.cvtColor(orthoimage_arr, cv2.COLOR_BGR2GRAY)
            orthoimage_stack = np.dstack((orthoimage_arr, dem_arr))

            # TODO: make dem 16 bit
            assert orthoimage_stack.shape[2] == 2, (
                f"Orthoimage stack channel count was {orthoimage_stack.shape[2]} "
                f"when 2 was expected (one channel for 8-bit grayscale reference "
                f"image and one 8-bit channel for 8-bit elevation reference)"
            )

            # Rotate and crop orthoimage stack
            # TODO: implement this part better e.g. use
            #  tf_transformations.euler_from_quaternion
            camera_yaw_degrees = tf_.extract_yaw(transform.rotation)
            camera_roll_degrees = tf_.extract_roll(transform.rotation)
            # This is assumed to be positive clockwise when looking down nadir
            # (z axis up in an ENU frame), z is aligned with zenith so in that sense
            # this is positive in the counter-clockwise direction. E.g. east aligned
            # rotation is positive 90 degrees.
            rotation = (camera_yaw_degrees + camera_roll_degrees) % 360

            crop_shape: Tuple[int, int] = image.height, image.width

            # here positive rotation is counter-clockwise, so we invert
            orthoimage_rotated_stack, M = self._rotate_and_crop_center(
                orthoimage_stack, rotation, crop_shape
            )

            reference_image_msg = self._cv_bridge.cv2_to_imgmsg(
                orthoimage_rotated_stack[:, :, 0], encoding="mono8"
            )

            # TODO: 16 bit DEM
            dem_msg = self._cv_bridge.cv2_to_imgmsg(
                orthoimage_rotated_stack[:, :, 1], encoding="mono8"
            )
            reference_image_msg.header.stamp = image.header.stamp
            dem_msg.header.stamp = image.header.stamp

            ortho_stereo_image_msg = OrthoStereoImage(
                query=image, reference=reference_image_msg, dem=dem_msg
            )

            # Publish transformation
            proj_str = self._world_to_reference_proj_str(
                np.linalg.inv(M),  # TODO: try-except
                orthoimage.crs.data,
            )

            ortho_stereo_image_msg.crs = String(data=proj_str)

            return ortho_stereo_image_msg

        query_image, orthoimage = self.image, self.orthoimage

        # Need camera orientation in an ENU frame ("map") to rotate
        # the orthoimage stack
        transform = (
            tf_.get_transform(self, "map", "camera", rclpy.time.Time())
            if hasattr(self, "_tf_buffer")
            else None
        )

        return _pnp_image(
            query_image,
            orthoimage,
            transform,
        )

    @property
    @ROS.publish(
        ROS_TOPIC_RELATIVE_TWIST_IMAGE,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def twist_image(self) -> Optional[MonocularStereoImage]:
        """Published stereo couple image consisting of query image and reference image

        Used for visual odometry and specifically velocity estimation
        """

        @narrow_types(self)
        def _stereo_image(qry: Image, ref: Image) -> Optional[MonocularStereoImage]:
            return MonocularStereoImage(query=qry, reference=ref)

        return _stereo_image(qry=self.image, ref=self.previous_image)

    # @staticmethod
    def _rotate_and_crop_center(
        self, image: np.ndarray, angle_degrees: float, shape: Tuple[int, int]
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Rotates an image around its center axis and then crops it to the
        specified shape.

        :param image: Numpy array representing the image.
        :param angle: Rotation angle in degrees.
        :param shape: Tuple (height, width) representing the desired shape
            after cropping.
        :return: Tuple of 1. Cropped and rotated image, and 2. matrix that can be
            used to convert points in rotated and cropped frame back into original
            frame
        """
        # Image dimensions
        h, w = image.shape[:2]

        # Center of rotation
        center = (w // 2, h // 2)

        # Calculate the rotation matrix
        rotation_matrix = cv2.getRotationMatrix2D(center, angle_degrees, 1.0)

        # Perform the rotation
        rotated_image = cv2.warpAffine(image, rotation_matrix, (w, h))

        # Calculate the cropping coordinates
        dx = center[0] - shape[1] // 2
        dy = center[1] - shape[0] // 2

        # Perform the cropping
        cropped_image = rotated_image[dy : dy + shape[0], dx : dx + shape[1]]

        # Invert the matrix
        extended_matrix = np.vstack([rotation_matrix, [0, 0, 1]])
        inverse_matrix = np.linalg.inv(extended_matrix)

        # Center-crop inverse translation
        T = np.array([[1, 0, dx], [0, 1, dy], [0, 0, 1]])

        inverse_matrix = inverse_matrix @ T

        # refimg = self._cv_bridge.imgmsg_to_cv2(
        #    self.orthoimage, desired_encoding="passthrough"
        # ).copy()
        # br = inverse_matrix @ np.array([640, 360, 1])
        # tf_.visualize_camera_corners(
        #    refimg,
        #    [inverse_matrix @ np.array([0, 0, 1]), br],
        #    "World frame origin/top-left position in reference frame",
        # )

        return cropped_image, inverse_matrix
