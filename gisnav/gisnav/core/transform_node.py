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

import cv2
import numpy as np
from cv_bridge import CvBridge
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import Quaternion
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from .. import messaging
from .._decorators import ROS, narrow_types
from ..static_configuration import (
    BBOX_NODE_NAME,
    GIS_NODE_NAME,
    ROS_NAMESPACE,
    ROS_TOPIC_RELATIVE_CAMERA_GEOPOSE,
    ROS_TOPIC_RELATIVE_ORTHOIMAGE,
    ROS_TOPIC_RELATIVE_PNP_IMAGE,
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
        self.camera_geopose

        # Initialize the static transform broadcaster
        self.broadcaster = StaticTransformBroadcaster(self)

    @property
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_ORTHOIMAGE.replace("~", GIS_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def orthoimage(self) -> Optional[Image]:
        """Subscribed :term:`orthoimage` for :term:`pose` estimation"""

    # TODO need some way of not sending stuff to pnp if looks like camera is
    #  looking in the wrong direction
    @property
    # @ROS.max_delay_ms(messaging.DELAY_DEFAULT_MS)
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_CAMERA_GEOPOSE.replace("~", BBOX_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def camera_geopose(self) -> Optional[GeoPoseStamped]:
        """:term:`Camera` :term:`geopose`, or None if not available"""

    @property
    # @ROS.max_delay_ms(messaging.DELAY_SLOW_MS) - gst plugin does not enable timestamp?
    @ROS.subscribe(messaging.ROS_TOPIC_CAMERA_INFO, QoSPresetProfiles.SENSOR_DATA.value)
    def camera_info(self) -> Optional[CameraInfo]:
        """Camera info for determining appropriate :attr:`.orthoimage` resolution"""

    @property
    # @ROS.max_delay_ms(messaging.DELAY_FAST_MS) - gst plugin does not enable timestamp?
    @ROS.subscribe(
        messaging.ROS_TOPIC_IMAGE,
        QoSPresetProfiles.SENSOR_DATA.value,
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
            camera_geopose: GeoPoseStamped,
        ) -> Optional[Image]:
            """Rotate and crop and orthoimage stack to align with query image"""

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
            camera_yaw_degrees = self._extract_yaw(camera_geopose.pose.orientation)
            crop_shape: Tuple[int, int] = query_img.shape[0:2]
            (
                orthoimage_rotated_stack,
                r_rotated,
                t_cropped,
            ) = self._rotate_and_crop_image(
                orthoimage_stack, camera_yaw_degrees, crop_shape
            )

            # TODO: is this cv2/numpy stuff correct?
            # ESD (cv2 x is width) to SEU (numpy array y is south) (x y might
            # be flipped because cv2)
            t_cropped = np.array(
                (
                    t_cropped[1],
                    t_cropped[0],
                    -t_cropped[2],
                    t_cropped[3],
                )
            )

            # Add query image on top to complete full image stack
            pnp_image_stack = np.dstack((query_img, orthoimage_rotated_stack))

            pnp_image_msg = self._cv_bridge.cv2_to_imgmsg(
                pnp_image_stack, encoding="passthrough"
            )
            pnp_image_msg.header.stamp = image.header.stamp

            pnp_image_msg.header.frame_id = "pnp"

            transform_ortho = messaging.create_transform_msg(
                pnp_image_msg.header.stamp, "reference", "pnp", r_rotated, t_cropped
            )
            self.broadcaster.sendTransform([transform_ortho])

            return pnp_image_msg

        return _pnp_image(
            self.image,
            self.orthoimage,
            self.camera_geopose,
        )

    @staticmethod
    def _get_rotation_matrix(image: np.ndarray, degrees: float) -> np.ndarray:
        height, width = image.shape[:2]
        cx, cy = height // 2, width // 2
        r = cv2.getRotationMatrix2D((cx, cy), degrees, 1.0)
        return r

    @staticmethod
    def _get_translation_matrix(dx, dy):
        t = np.float32([[1, 0, dx], [0, 1, dy]])
        return t

    @classmethod
    def _get_affine_matrix(
        cls, image: np.ndarray, degrees: float, crop_height: int, crop_width: int
    ) -> np.ndarray:
        """Creates affine transformation that rotates around center and then
        center-crops an image.

        .. note::
            Returns matrix in 3D since this matrix will not only be used for rotating
            and cropping the orthoimage rasters but also for converting 3D pose
            estimates in the rotated and cropped orthoimage frame back to the original
            unrotated and uncropped frame (from where it will then be converted to
            geocoordinates).

        Returns affine matrix padded to 3D (4x4 matrix) in the following format:
            [ R11  R12  0   Tx ]
            [ R21  R22  0   Ty ]
            [ 0    0    1   0  ]
            [ 0    0    0   1  ]
        where R11, R12, R21, R22 represents the rotation matrix, and Tx, Ty represent
        the translation along the x and y axis.

        :return: The affine transformation matrix in homogenous format as masked
            numpy array. Masking for use in 2D operations (e.g. cv2.warpAffine).
        """
        r = cls._get_rotation_matrix(image, degrees)
        assert r.shape == (2, 3)
        dx = (image.shape[0] - crop_height) // 2
        dy = (image.shape[1] - crop_width) // 2
        t = cls._get_translation_matrix(dx, dy)
        assert t.shape == (2, 3)

        # Combine rotation and translation to get the final affine transformation
        affine_2d = np.dot(t, np.vstack([r, [0, 0, 1]]))

        # Convert 2D affine matrix to 3D affine matrix
        # Create a 4x4 identity matrix
        affine_3d = np.eye(4)

        # Insert the 2D affine transformation into the 3D matrix
        affine_3d[:2, :2] = affine_2d[:, :2]
        affine_3d[:2, 3] = affine_2d[:, 2]

        assert affine_3d.shape == (4, 4)

        # Translation hack to make cv2.warpAffine warp the image into the top left
        # corner so that the output size argument of cv2.warpAffine acts as a
        # center-crop
        t[:2, 2] = -t[:2, 2][::-1]
        affine_hack = np.dot(t, np.vstack([r, [0, 0, 1]]))

        affine_3d[:2, :2] = affine_hack[:2, :2]
        affine_3d[:2, 3] = affine_hack[:2, 2]
        return affine_3d

    @classmethod
    def _rotate_and_crop_image(
        cls, image: np.ndarray, degrees: float, shape: Tuple[int, int]
    ) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """
        Rotates around center and then center-crops image

        Cached because the same rotated image is expected to be used for multiple
        matches.

        :return: Tuple of rotated and cropped image, and used rotation matrix
            and translation vector
        """
        # Image can have any number of channels
        affine = cls._get_affine_matrix(image, degrees, *shape)
        affine_2d = np.delete(affine, 2, 1)
        affine_2d = affine_2d[:2, :]

        r = affine[:2, :2]
        t = affine[:2, 2]

        # Add the z-axis scaling to the rotation matrix
        r = np.insert(r, 2, 0, axis=1)
        r = np.insert(r, 2, 0, axis=0)
        r[2, 2] = affine[2, 2]

        # Add the z-axis translation (which is zero)
        t = np.append(t, 0)

        return cv2.warpAffine(image, affine_2d, shape[::-1]), r, t
