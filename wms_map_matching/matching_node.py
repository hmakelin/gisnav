import rclpy
import os
import traceback

from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from px4_msgs.msg import VehicleLocalPosition
from sensor_msgs.msg import Image, CameraInfo
from owslib.wms import WebMapService
from cv2 import VideoCapture, imwrite, imdecode
import numpy as np
import cv2  # TODO: remove
from cv_bridge import CvBridge

from wms_map_matching.geo import get_bbox


class Matcher(Node):
    # VehicleGlobalPosition not supported by microRTPS bridge - use VehicleLocalPosition instead
    vehicle_local_position_topic = "VehicleLocalPosition_PubSubTopic"
    image_raw_topic = "image_raw"
    camera_info_topic = "camera_info"

    # Determines map size, radius of enclosed circle in meters
    map_bbox_radius = 100

    def __init__(self):
        """Initializes the node."""
        super().__init__('matcher')
        self._init_wms()
        self._image_raw_sub = self.create_subscription(Image, self.image_raw_topic, self._image_raw_callback, 1)
        self._camera_info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self._camera_info_callback,
                                                         1)
        self._vehicle_local_position_sub = self.create_subscription(VehicleLocalPosition,
                                                                    self.vehicle_local_position_topic,
                                                                    self._vehicle_local_position_callback, 1)
        self._cv_bridge = CvBridge()
        self._camera_info = None
        self._image_raw = None
        self._cv_image = None
        self._map = None

    def _init_wms(self):
        """Initializes the Web Map Service (WMS) client used by the node to request map rasters.

        The url and version parameters are required to initialize the WMS client and are therefore set to read only. The
        layer and srs parameters can be changed dynamically.
        """
        # TODO: don't hardcode defaults here, get values from launch file
        self.declare_parameter('url', 'http://localhost:8080/wms', ParameterDescriptor(read_only=True))
        self.declare_parameter('version', '1.1.1', ParameterDescriptor(read_only=True))
        self.declare_parameter('layer', 'WorldImagery')
        self.declare_parameter('srs', 'EPSG:4326')  # TODO: get_bbox currently only supports EPSG:4326

        self._wms = WebMapService(self.get_parameter('url').get_parameter_value().string_value,
                                  version=self.get_parameter('version').get_parameter_value().string_value)

    def _update_map(self):
        """Gets latest map from WMS server and returns it as numpy array."""
        # TODO: raster size? get bbox only supports EPSG:4326 although it might be configurable in the future
        self._map_bbox = get_bbox((self._vehicle_local_position.ref_lat, self._vehicle_local_position.ref_lon),
                                  self.map_bbox_radius)

        if all(i is not None for i in [self._camera_info]):
            img_size = self._camera_info.width, self._camera_info.height
            layer_str = self.get_parameter('layer').get_parameter_value().string_value
            srs_str = self.get_parameter('srs').get_parameter_value().string_value
            self.get_logger().debug('Getting map for bounding box: {}, layer: {}, srs: {}.'.format(self._map_bbox,
                                                                                                   layer_str, srs_str))
            self._map = self._wms.getmap(layers=[layer_str], srs=srs_str, bbox=self._map_bbox, size=img_size,
                                         format='image/png', transparent=True)

            self._map = np.frombuffer(self._map.read(), np.uint8)
            self._map = imdecode(self._map, cv2.IMREAD_UNCHANGED)
        else:
            self.get_logger().debug('Camera info not available - will not yet get map from WMS endpoint.')

    def _image_raw_callback(self, msg):
        """Handles reception of latest image frame from camera."""
        self.get_logger().debug('Camera image callback triggered.')
        stamp = msg.header.stamp.sec
        now = self.get_clock().now().to_msg().sec
        if stamp + 1 < now:
            self.get_logger().debug('Skipping frame older than 1 second ({} vs {})'.format(stamp, second))
            return
        self._image_raw = msg
        self._cv_image = self._cv_bridge.imgmsg_to_cv2(self._image_raw, 'bgr8')
        if all(i is not None for i in [self._image_raw, self._map]):
            self._match()
        else:
            self.get_logger().debug('Map or image not available - not calling NG-RANSAC yet.')

    def _camera_info_callback(self, msg):
        """Handles reception of camera info."""
        self.get_logger().debug('Camera info callback triggered.')
        self._camera_info = msg
        self._camera_info_sub.destroy()  # TODO: check that info was indeed received before destroying subscription

    def _vehicle_local_position_callback(self, msg):
        """Handles reception of latest global position estimate."""
        self.get_logger().debug('Vehicle local position callback triggered.')
        self._vehicle_local_position = msg
        self._update_map()

    def _match(self):
        """Does matching on camera and map images."""
        pass


def main(args=None):
    rclpy.init(args=args)
    matcher = Matcher()
    rclpy.spin(matcher)
    matcher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
