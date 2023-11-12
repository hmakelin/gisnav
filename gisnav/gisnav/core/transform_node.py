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
from typing import Final, Optional, Tuple
from copy import deepcopy

import cv2
import numpy as np
import tf2_ros
import rclpy
import tf_transformations

from cv_bridge import CvBridge
from geometry_msgs.msg import Quaternion, TransformStamped
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros.transform_broadcaster import TransformBroadcaster


from .. import messaging
from ..constants import (
    GIS_NODE_NAME,
    ROS_NAMESPACE,
    ROS_TOPIC_RELATIVE_ORTHOIMAGE,
    ROS_TOPIC_RELATIVE_PNP_IMAGE,
)
from ..decorators import ROS, narrow_types


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
    @ROS.subscribe(messaging.ROS_TOPIC_CAMERA_INFO, QoSPresetProfiles.SENSOR_DATA.value)
    def camera_info(self) -> Optional[CameraInfo]:
        """Camera info for determining appropriate :attr:`.orthoimage` resolution"""

    def _image_cb(self, msg: Image) -> None:
        """Callback for :attr:`.image` message"""
        self.pnp_image

    @property
    # @ROS.max_delay_ms(messaging.DELAY_FAST_MS) - gst plugin does not enable timestamp?
    @ROS.subscribe(
        messaging.ROS_TOPIC_IMAGE,
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=_image_cb,
    )
    def image(self) -> Optional[Image]:
        """Raw image data from vehicle camera for pose estimation"""

    @staticmethod
    def _extract_yaw(q: Quaternion) -> float:
        """Calculate the yaw angle from a quaternion in the ENU frame.

        Returns yaw with origin centered at North (i.e. applies a 90 degree adjustment).

        :param q: A list containing the quaternion [qx, qy, qz, qw].
        :return: The yaw angle in degrees.
        """
        enu_yaw = np.arctan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y**2 + q.z**2))
        enu_yaw_deg = np.degrees(enu_yaw)

        # Convert ENU yaw to heading with North as origin
        heading = 90.0 - enu_yaw_deg

        # Normalize to [0, 360) range
        heading = (heading + 360) % 360

        return heading

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

            parent_frame_id: messaging.FrameID = orthoimage.header.frame_id
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
            camera_yaw_degrees = self._extract_yaw(transform.rotation)
            self.get_logger().error(f"camera yaw {camera_yaw_degrees}")
            crop_shape: Tuple[int, int] = query_img.shape[0:2]
            orthoimage_rotated_stack = self._rotate_and_crop_center(
                orthoimage_stack, camera_yaw_degrees, crop_shape
            )

            # Add query image on top to complete full image stack
            pnp_image_stack = np.dstack((query_img, orthoimage_rotated_stack))

            pnp_image_msg = self._cv_bridge.cv2_to_imgmsg(
                pnp_image_stack, encoding="passthrough"
            )

            # The child frame is the 'world' frame of the PnP problem as
            # defined here: https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html
            child_frame_id: messaging.FrameID = "world"
            pnp_image_msg.header.stamp = image.header.stamp
            pnp_image_msg.header.frame_id = child_frame_id

            center = (orthoimage_stack.shape[0] // 2, orthoimage_stack.shape[1] // 2)
            stamp = rclpy.time.Time()
            self._publish_transform(center, camera_yaw_degrees, crop_shape, parent_frame_id, child_frame_id, pnp_image_msg.header.stamp)

            return pnp_image_msg

        query_image, orthoimage = self.image, self.orthoimage

        transform = (
            messaging.get_transform(self, "map", "gimbal", rclpy.time.Time())  #query_image.header.stamp)
            if self.image is not None
            else None
        )

        # TODO: publish camera positio overlaid on orthoimage (reference frame)
        #  here - move this code block to a more appropriate place in the future
        if orthoimage is not None:
            debug_ref_image = self._cv_bridge.imgmsg_to_cv2(
                deepcopy(orthoimage), desired_encoding="passthrough"
            )
            debug_ref_image = debug_ref_image[:, :, 0]  # first channel is grayscale image
            # current image timestamp does not yet have the transform but this should get the previous one
            camera_pose_transform = messaging.get_transform(self, "camera", "reference", rclpy.time.Time()) # query_image.header.stamp)
            if camera_pose_transform is not None:
                x, y = int(camera_pose_transform.transform.translation.x), int(camera_pose_transform.transform.translation.y)
                self.get_logger().error(f"translation in reference {camera_pose_transform.transform.translation}")
                debug_ref_image = cv2.circle(np.array(debug_ref_image), (-x, -y), 5, (0,255,0), -1)
                cv2.imshow("Camera position in reference frame", debug_ref_image)
                cv2.waitKey(1)

        return _pnp_image(
            query_image,
            orthoimage,
            transform,
        )

    @staticmethod
    def _rotate_and_crop_center(image: np.ndarray, angle_degrees: float, shape: Tuple[int, int]):
        """
        Rotates an image around its center axis and then crops it to the specified shape.

        :param image: Numpy array representing the image.
        :param angle: Rotation angle in degrees.
        :param shape: Tuple (height, width) representing the desired shape after cropping.
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
        cropped_image = rotated_image[y:y+shape[0], x:x+shape[1]]

        return cropped_image

    @staticmethod
    def get_reverse_transformation(center, angle_degrees, original_shape, crop_shape):
        """
        Computes the transformation matrix to reverse a rotation and cropping operation.

        :param center: Tuple (x, y) representing the center of rotation.
        :param angle_degrees: Rotation angle in degrees (same as used in the forward transformation).
        :param original_shape: Tuple (height, width) of the original image.
        :param crop_shape: Tuple (height, width) of the cropped image.
        :return: The transformation matrix to reverse the rotation and cropping.
        """
        # Calculate the inverse rotation matrix
        inverse_rotation_matrix = cv2.getRotationMatrix2D(center, -angle_degrees, 1.0)

        # Calculate the translation to adjust for the cropping
        crop_x = center[0] - crop_shape[1] // 2
        crop_y = center[1] - crop_shape[0] // 2
        translation_matrix = np.float32([[1, 0, crop_x], [0, 1, crop_y]])

        # Combine the inverse rotation and translation
        reverse_transformation_matrix = np.dot(translation_matrix, inverse_rotation_matrix)

        return reverse_transformation_matrix

    def _publish_transform(self, center: Tuple[int, int], angle_degrees: float, crop_shape: Tuple[int, int], frame_id: str, child_frame_id: str, stamp):
        """
        Publishes a transform that represents the rotation and cropping operation.

        :param broadcaster: tf2_ros TransformBroadcaster object.
        :param center: Tuple (x, y) representing the center of rotation.
        :param angle: Rotation angle in degrees.
        :param crop_shape: Tuple (height, width) representing the cropping dimensions.
        :param frame_id: The frame ID to which this transform is related.
        :param child_frame_id: The child frame ID for this transform.
        """
        angle_rad = np.deg2rad(angle_degrees)
        quaternion = tf_transformations.quaternion_from_euler(0, 0, angle_rad)

        # Create a TransformStamped message
        t = TransformStamped()

        # Fill the message
        t.header.stamp = stamp
        t.header.frame_id = frame_id
        t.child_frame_id = child_frame_id
        t.transform.translation.x = center[0] - crop_shape[1] / 2
        t.transform.translation.y = center[1] - crop_shape[0] / 2
        t.transform.translation.z = 0.
        t.transform.rotation.x = quaternion[0]
        t.transform.rotation.y = quaternion[1]
        t.transform.rotation.z = quaternion[2]
        t.transform.rotation.w = quaternion[3]

        # Broadcast the transform
        self.broadcaster.sendTransform(t)
