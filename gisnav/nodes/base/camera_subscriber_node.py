"""Abstract base class for all nodes that subscribe to the camera info and image topics"""
from abc import abstractmethod
from typing import Optional, Tuple

import numpy as np
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import CameraInfo, Image

from .base_node import BaseNode
from gisnav.data import Dim, CameraData
from gisnav.assertions import assert_type


class CameraSubscriberNode(BaseNode):
    """Abstract base class for all nodes that subscribe to the camera info and image topics

    This class is intended to be extended by nodes in the same package and should not be imported directly in any other
    package.

    Extending classes must implement the :class:`sensor_msgs.msg.Image` callback. :class:`sensor_msgs.msg.CameraInfo`
    messages are automatically consumed and the contained information provided through the :py:attr:`.camera_data`
    property for convenience.
    """

    ROS_CAMERA_INFO_TOPIC = 'camera/camera_info'
    """:class:`sensor_msgs.msg.CameraInfo` topic to subscribe to"""

    ROS_IMAGE_TOPIC = 'camera/image_raw'
    """:class:`sensor_msgs.msg.Image` topic to subscribe to"""

    def __init__(self, name: str):
        """Initializes the node

        :param name: Node name
        """
        super().__init__(name)

        self.__camera_info = None
        self.__camera_info_sub = self.create_subscription(CameraInfo,
                                                          self.ROS_CAMERA_INFO_TOPIC,
                                                          self.__camera_info_callback,
                                                          QoSPresetProfiles.SENSOR_DATA.value)

        self.__image_sub = self.create_subscription(Image,
                                                    self.ROS_IMAGE_TOPIC,
                                                    self.image_callback,
                                                    QoSPresetProfiles.SENSOR_DATA.value)

    def __camera_info_callback(self, msg: CameraInfo) -> None:
        """Handles latest :class:`sensor_msgs.msg.CameraInfo` message

        Not intended to be implemented by extending classes. Camera info is stored and provided to extending class
        through :py:attr:`.camera_data` property.

        :param msg: Latest :class:`sensor_msgs.msg.CameraInfo` message
        """
        self.__camera_info = msg

    @abstractmethod
    def image_callback(self, msg: Image) -> None:
        """Handles the latest :class:`sensor_msgs.msg.Image` message

        Must be implemented by extending classes (unlike callback for :class:`sensor_msgs.msg.CameraInfo`).
        """
        pass

    @property
    def img_dim(self) -> Optional[Dim]:
        """Image resolution from latest :class:`px4_msgs.msg.CameraInfo` message, None if not available"""
        if self.camera_data is not None:
            return self.camera_data.dim
        else:
            self.get_logger().warn('Camera data was not available, returning None as declared image size.')
            return None

    @property
    def map_size_with_padding(self) -> Optional[Tuple[int, int]]:
        """Padded map size tuple (height, width) or None if the information is not available.

        Because the deep learning models used for predicting matching keypoints or poses between camera image frames
        and map rasters are not assumed to be rotation invariant in general, the map rasters are rotated based on
        camera yaw so that they align with the camera images. To keep the scale of the map after rotation the same,
        black corners would appear unless padding is used. Retrieved maps therefore have to be squares with the side
        lengths matching the diagonal of the camera frames so that scale is preserved and no black corners appear in
        the map rasters after arbitrary 2D rotation. The height and width will both be equal to the diagonal of the
        declared (:py:attr:`.img_dim`) camera frame dimensions.
        """
        if self.img_dim is None:
            self.get_logger().warn(f'Dimensions not available - returning None as map size.')
            return None
        diagonal = int(np.ceil(np.sqrt(self.img_dim.width ** 2 + self.img_dim.height ** 2)))
        assert_type(diagonal, int)
        return diagonal, diagonal

    @property
    def camera_data(self) -> Optional[CameraData]:
        """Camera intrinsics or None if not available"""
        if not all(hasattr(self.__camera_info, attr) for attr in ['k', 'height', 'width']):
            return None
        else:
            camera_info = self.__camera_info
            return CameraData(camera_info.k.reshape((3, 3)), dim=Dim(camera_info.height, camera_info.width))
