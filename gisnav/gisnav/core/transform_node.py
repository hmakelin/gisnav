"""This module contains :class:`.TransformNode`, a :term:`ROS` node generating the
:term:`query` and :term:`reference` image pair by rotating the reference
image based on :term:`vehicle` heading, and then cropping it based on the
:term:`camera` information.

.. mermaid::
    :caption: :class:`.PnPNode` computational graph

    graph LR
        subgraph TransformNode
            pnp_image[gisnav/transform_node/image]
        end

        subgraph gscam
            camera_info[camera/camera_info]
            image[camera/image_raw]
        end

        subgraph BBoxNode
            camera_pose[gisnav/bbox_node/camera/geopose]
        end

        subgraph GISNode
            orthoimage[gisnav/gis_node/image]
        end

        image -->|sensor_msgs/Image| TransformNode
        camera_info -->|sensor_msgs/CameraInfo| TransformNode
        orthoimage -->|sensor_msgs/Image| TransformNode
        camera_pose -->|geometry_msgs/PoseStamped| TransformNode
        pnp_image -->|sensor_msgs/Image| PnPNode:::hidden
"""
from copy import deepcopy
from typing import Final, Optional, Tuple

import cv2
import numpy as np
import rclpy
import tf2_ros
import tf_transformations
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros.transform_broadcaster import TransformBroadcaster

from .. import _messaging as messaging
from .._decorators import ROS, narrow_types
from ..constants import (
    GIS_NODE_NAME,
    ROS_NAMESPACE,
    ROS_TOPIC_CAMERA_INFO,
    ROS_TOPIC_IMAGE,
    ROS_TOPIC_RELATIVE_ORTHOIMAGE,
    ROS_TOPIC_RELATIVE_PNP_IMAGE,
    FrameID,
)


class TransformNode(Node):
    """Publishes :term:`query` and :term:`reference` image pair

    Rotates the reference image based on :term:`vehicle` heading, and then
    crops it based on :term:`camera` image resolution.
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

        # Converts image_raw to cv2 compatible image
        self._cv_bridge = CvBridge()

        # Calling these decorated properties the first time will setup
        # subscriptions to the appropriate ROS topics
        self.orthoimage
        self.camera_info
        self.image

        # Initialize the transform broadcaster and listener
        self.broadcaster = TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    @property
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_ORTHOIMAGE.replace("~", GIS_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def orthoimage(self) -> Optional[Image]:
        """Subscribed :term:`orthoimage` for :term:`pose` estimation"""

    @property
    # @ROS.max_delay_ms(messaging.DELAY_SLOW_MS) - gst plugin does not enable timestamp?
    @ROS.subscribe(
        ROS_TOPIC_CAMERA_INFO,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def camera_info(self) -> Optional[CameraInfo]:
        """Camera info for determining appropriate :attr:`.orthoimage` resolution"""

    def _image_cb(self, msg: Image) -> None:
        """Callback for :attr:`.image` message"""
        self.pnp_image

    @property
    # @ROS.max_delay_ms(messaging.DELAY_FAST_MS) - gst plugin does not enable timestamp?
    @ROS.subscribe(
        ROS_TOPIC_IMAGE,
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=_image_cb,
    )
    def image(self) -> Optional[Image]:
        """Raw image data from vehicle camera for pose estimation"""

    @staticmethod
    def _determine_utm_zone(longitude):
        """Determine the UTM zone for a given longitude."""
        return int((longitude + 180) / 6) + 1

    @property
    @ROS.publish(
        ROS_TOPIC_RELATIVE_PNP_IMAGE,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def pnp_image(self) -> Optional[Image]:
        """Published :term:`stacked <stack>` image consisting of query image,
        reference image, and reference elevation raster (:term:`DEM`).

        .. note::
            Semantically not a single image, but a stack of two 8-bit grayscale
            images and one 16-bit "image-like" elevation reference, stored in a
            compact way in an existing message type so to avoid having to also
            publish custom :term:`ROS` message definitions.
        """

        @narrow_types(self)
        def _pnp_image(
            image: Image,
            orthoimage: Image,
            transform: TransformStamped,
        ) -> Optional[Image]:
            """Rotate and crop and orthoimage stack to align with query image"""
            transform = transform.transform

            parent_frame_id: FrameID = orthoimage.header.frame_id
            assert parent_frame_id == "reference"

            query_img = self._cv_bridge.imgmsg_to_cv2(image, desired_encoding="mono8")

            orthoimage_stack = self._cv_bridge.imgmsg_to_cv2(
                orthoimage, desired_encoding="passthrough"
            )

            assert orthoimage_stack.shape[2] == 3, (
                f"Orthoimage stack channel count was {orthoimage_stack.shape[2]} "
                f"when 3 was expected (one channel for 8-bit grayscale reference "
                f"image and two 8-bit channels for 16-bit elevation reference)"
            )

            # Rotate and crop orthoimage stack
            # TODO: implement this part better
            camera_yaw_degrees = messaging.extract_yaw(transform.rotation)
            camera_roll_degrees = messaging.extract_roll(transform.rotation)
            rotation = camera_yaw_degrees + camera_roll_degrees
            crop_shape: Tuple[int, int] = query_img.shape[0:2]
            orthoimage_rotated_stack = self._rotate_and_crop_center(
                orthoimage_stack, rotation, crop_shape
            )

            # Add query image on top to complete full image stack
            pnp_image_stack = np.dstack((query_img, orthoimage_rotated_stack))

            pnp_image_msg = self._cv_bridge.cv2_to_imgmsg(
                pnp_image_stack, encoding="passthrough"
            )

            # The child frame is the 'world' frame of the PnP problem as
            # defined here: https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html
            child_frame_id: FrameID = "world"
            pnp_image_msg.header.stamp = image.header.stamp
            pnp_image_msg.header.frame_id = child_frame_id

            center = (orthoimage_stack.shape[0] // 2, orthoimage_stack.shape[1] // 2)

            cx, cy = center[0], center[1]
            dx = cx - crop_shape[1] / 2
            dy = cy - crop_shape[0] / 2

            # Compute transformation (rotation around center + crop)
            theta = np.radians(rotation)

            # Translation to origin
            T1 = np.array([[1, 0, -cx], [0, 1, -cy], [0, 0, 1]])

            # Rotation
            R = np.array(
                [
                    [np.cos(theta), -np.sin(theta), 0],
                    [np.sin(theta), np.cos(theta), 0],
                    [0, 0, 1],
                ]
            )

            # Translation back from origin
            T2 = np.array([[1, 0, cx], [0, 1, cy], [0, 0, 1]])

            # Center-crop translation
            T3 = np.array([[1, 0, -dx], [0, 1, -dy], [0, 0, 1]])

            # Combined affine matrix: reference coordinate to world coordinate
            affine_2d = T3 @ T2 @ R @ T1

            # Convert to 4x4 matrix
            affine_3d = np.eye(4)
            affine_3d[0:2, 0:2] = affine_2d[0:2, 0:2]  # Copy rotation
            affine_3d[0:2, 3] = affine_2d[0:2, 2]  # Copy translation

            translation = affine_3d[:3, 3]

            try:
                q = tf_transformations.quaternion_from_matrix(affine_3d)
            except np.linalg.LinAlgError:
                self.get_logger().warning(
                    "_pnp_image: Could not compute quaternion from estimated rotation. "
                    "Returning None."
                )
                return None

            transform_camera = messaging.create_transform_msg(
                pnp_image_msg.header.stamp,
                child_frame_id,
                parent_frame_id,
                q,
                translation,
            )
            # We also publish a reference_{timestamp} frame so that we will be able
            # to trace the transform chain back to the exact timestamp (to match
            # with the geotransform message). This is needed because interpolation will
            # often produce very inaccurate results with the discontinous reference
            # frame. The orthoimage timestamp which we use here is used as a proxy
            # for the geotransform timestamp (i.e. they must be published with
            # the same timestamps).
            # TODO: remove this assumption or make the design less brittle in some
            #  other way
            transform_camera_stamped = deepcopy(transform_camera)
            transform_camera_stamped.child_frame_id = (
                f"{transform_camera.child_frame_id}"
                f"_{orthoimage.header.stamp.sec}"
                f"_{orthoimage.header.stamp.nanosec}"
            )
            self.broadcaster.sendTransform([transform_camera, transform_camera_stamped])

            # if orthoimage is not None:
            #    ref = deepcopy(orthoimage_stack[:, :, 0])
            #    camera_pose_transform = messaging.get_transform(
            #        self,
            #        "reference",
            #        "camera_pinhole",
            #        image.header.stamp
            #    )
            #    if camera_pose_transform is not None:
            #        h = orthoimage_stack.shape[0]
            #        messaging.visualize_transform(
            #            camera_pose_transform,
            #            ref,
            #            h,
            #            "Camera position in ref frame"
            #        )

            return pnp_image_msg

        query_image, orthoimage = self.image, self.orthoimage

        transform = (
            messaging.get_transform(
                self, "map", "gimbal", rclpy.time.Time()
            )  # query_image.header.stamp)
            if self.image is not None
            else None
        )

        return _pnp_image(
            query_image,
            orthoimage,
            transform,
        )

    @staticmethod
    def _rotate_and_crop_center(
        image: np.ndarray, angle_degrees: float, shape: Tuple[int, int]
    ):
        """Rotates an image around its center axis and then crops it to the
        specified shape.

        :param image: Numpy array representing the image.
        :param angle: Rotation angle in degrees.
        :param shape: Tuple (height, width) representing the desired shape
            after cropping.
        :return: Cropped and rotated image.
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
        x = center[0] - shape[1] // 2
        y = center[1] - shape[0] // 2

        # Perform the cropping
        cropped_image = rotated_image[y : y + shape[0], x : x + shape[1]]

        return cropped_image
