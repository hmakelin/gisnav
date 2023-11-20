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

            cx, cy = center[0], center[1]
            dx = cx - crop_shape[1] / 2
            dy = cy - crop_shape[0] / 2

            # Compute transformation (rotation around center + crop)
            theta = np.radians(camera_yaw_degrees)

            # Translation to origin
            T1 = np.array([[1, 0, -cx],
                           [0, 1, -cy],
                           [0, 0, 1]])

            # Rotation
            R = np.array([[np.cos(theta), -np.sin(theta), 0],
                          [np.sin(theta), np.cos(theta), 0],
                          [0, 0, 1]])

            # Translation back from origin
            T2 = np.array([[1, 0, cx],
                           [0, 1, cy],
                           [0, 0, 1]])

            # Center-crop translation
            T3 = np.array([[1, 0, -dx],
                           [0, 1, -dy],
                           [0, 0, 1]])

            # Combined affine matrix: reference coordinate to world coordinate
            affine_matrix = T3 @ T2 @ R @ T1
            r = np.eye(3)
            r[:2, :2] = affine_matrix[:2, :2]
            t = affine_matrix[:3, 2]
            t[2] = 0.

            # TODO: clean this up - possibly invert sign of camera yaw above
            transform_msg = messaging.create_transform_msg(
                pnp_image_msg.header.stamp, parent_frame_id, child_frame_id, r.T, (-r.T @ t).squeeze()
            )
            self.broadcaster.sendTransform([transform_msg])


            # TODO: publish camera positio overlaid on orthoimage (reference frame)
            #  here - move this code block to a more appropriate place in the future
            if orthoimage is not None:

                # TODO: fix get_transform - currently returns the inverse (i.e. frame_ids in wrong order?)
                # TODO: use exact timestamp, reference frame is not continuous and cannot be interpolated
                camera_pose_transform = messaging.get_transform(self, "world", "reference",
                                                                rclpy.time.Time())  # query_image.header.stamp)
                if camera_pose_transform is not None:
                    # TODO parse r and t from the camera_pose_transform message
                    #  to ensure it is correct, do not use them directly here
                    position_in_world_frame = r[:2, :2] @ np.array(center) + t[:2]
                    world = deepcopy(orthoimage_rotated_stack[:, :, 0])
                    ref = deepcopy(orthoimage_stack[:, :, 0])
                    self.get_logger().error(f"Ref center position in world frame {position_in_world_frame}")
                    ref_center_position_in_world_frame = cv2.circle(world, tuple(map(int, position_in_world_frame)), 5, (0, 255, 0), -1)
                    cv2.imshow("Ref center position in world frame", ref_center_position_in_world_frame)

                    ref_center_position_in_ref_frame = cv2.circle(ref, tuple(map(int, center)), 5, (0, 255, 0), -1)
                    self.get_logger().error(f"Ref center position in ref frame {center}")
                    cv2.imshow("Ref center position in ref frame", ref_center_position_in_ref_frame)
                    cv2.waitKey(1)

                # TODO: fix get_transform - currently returns the inverse (i.e. frame_ids in wrong order?)
                camera_pose_transform = messaging.get_transform(self, "reference", "camera",
                                                                rclpy.time.Time())  #pnp_image_msg.header.stamp)  # query_image.header.stamp)
                if camera_pose_transform is not None:
                    q = camera_pose_transform.transform.rotation
                    q = [q.x, q.y, q.z, q.w]
                    r = tf_transformations.quaternion_matrix(q)[:3, :3]
                    t = np.array((camera_pose_transform.transform.translation.x, camera_pose_transform.transform.translation.y, camera_pose_transform.transform.translation.z))
                    affine = np.eye(4)
                    affine[:3, :3] = r
                    affine[:3, 3] = t
                    #pos = -affine.T @ np.array((0, 0, 0, 1))
                    ref = deepcopy(orthoimage_stack[:, :, 0])
                    pos = -r.T @ t
                    camera_position_in_ref_frame = cv2.circle(ref, tuple(map(int, pos[:2])), 5, (0, 255, 0), -1)
                    self.get_logger().error(f"Camera position in ref frame {pos}")
                    cv2.imshow("Camera position in ref frame", camera_position_in_ref_frame)
                    cv2.waitKey(1)


            return pnp_image_msg

        query_image, orthoimage = self.image, self.orthoimage

        transform = (
            messaging.get_transform(self, "map", "gimbal", rclpy.time.Time())  #query_image.header.stamp)
            if self.image is not None
            else None
        )

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
