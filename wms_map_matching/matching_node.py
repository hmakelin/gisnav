import rclpy
import sys
import os
import traceback
import xml.etree.ElementTree as ET

from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from px4_msgs.msg import VehicleLocalPosition
from sensor_msgs.msg import Image, CameraInfo
from owslib.wms import WebMapService
from cv2 import VideoCapture, imwrite, imdecode
import numpy as np
import cv2  # TODO: remove
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

from wms_map_matching.geo import get_bbox

# Add the share folder to Python path
from ament_index_python.packages import get_package_share_directory
package_name = 'wms_map_matching'  # TODO: try to read from somewhere (e.g. package.xml)
share_dir = get_package_share_directory(package_name)
superglue_dir = share_dir + '/SuperGluePretrainedNetwork'
sys.path.append(os.path.abspath(superglue_dir))  # need for importing from NG-RANSAC scripts (util, network) below

# Import this after superglue_dir has been added to path
from wms_map_matching.superglue_adapter import SuperGlue

class Matcher(Node):
    # VehicleGlobalPosition not supported by microRTPS bridge - use VehicleLocalPosition instead
    vehicle_local_position_topic = "VehicleLocalPosition_PubSubTopic"
    image_raw_topic = "image_raw"
    camera_info_topic = "camera_info"

    # Determines map size, radius of enclosed circle in meters
    map_bbox_radius = 200

    # SuperGlue config
    match_pairs_script = superglue_dir + '/match_pairs.py'
    images_dir = share_dir + '/images'
    input_pairs = images_dir + '/input_pairs.txt'
    input_dir = images_dir + '/input'
    output_dir = images_dir + '/output'
    img_file_name = 'img.jpg'
    map_file_name = 'map.jpg'
    img_file = input_dir + '/' + img_file_name
    map_file = input_dir + '/' + map_file_name
    output_file = output_dir + '/matches.jpg'


    def __init__(self, use_script=False):
        """Initializes the node."""
        super().__init__('matcher')
        self._init_wms()
        self._use_script = use_script
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
        self._superglue = None

        if self._use_script:
            self._create_dirs()
            self._create_input_pairs_file()
        else:
            self._superglue = SuperGlue(self.output_file, self.get_logger())

    def _create_dirs(self):
        """Creates required directories if they do not exist."""
        for dir in [self.images_dir, self.input_dir, self.output_dir]:
            if not os.path.exists(dir):
                self.get_logger().debug('Creating missing directory {}'.format(dir))
                try:
                    os.mkdir(dir)
                except Exception as e:
                    self.get_logger().error('Could not create directory {} because of error: {}\n{}'\
                                            .format(dir, e, traceback.print_exc()))
                    raise e
            else:
                self.get_logger().debug('Directory {} already exists.'.format(dir))

    def _create_input_pairs_file(self):
        """Creates the input pairs file required by SuperGlue if it does not exist."""
        if not os.path.exists(self.input_pairs):
            self.get_logger().debug('Appending {} and {} to input_pairs.txt file.'.format(self.img_file, self.map_file))
            try:
                with open(self.input_pairs, 'w') as f:
                    f.write(self.img_file_name + ' ' + self.map_file_name + '\n')
            except Exception as e:
                self.get_logger().error('Could not write input pairs file: {}\n{}'.format(e, traceback.print_exc()))
                raise e
        else:
            self.get_logger().debug('Skipping writing input pairs file since it already exists.')

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

        try:
            self._wms = WebMapService(self.get_parameter('url').get_parameter_value().string_value,
                                      version=self.get_parameter('version').get_parameter_value().string_value)
        except Exception as e:
            self.get_logger().error('Could not connect to WMS server.')
            raise e

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

            try:
                self._map = self._wms.getmap(layers=[layer_str], srs=srs_str, bbox=self._map_bbox, size=img_size,
                                             format='image/png', transparent=True)
            except Exception as e:
                self.get_logger().warn('Exception from WMS server query: {}\n{}'.format(e, traceback.print_exc()))
                return

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
            if self._use_script:
                self._match_script()
            else:
                self._match()
        else:
            self.get_logger().debug('Map or image not available: map {}, img {} - not calling matching yet.'\
                                    .format(self._map is not None, self._image_raw is not None))

    def _camera_info_callback(self, msg):
        """Handles reception of camera info."""
        self.get_logger().debug('Camera info callback triggered.')
        self._camera_info = msg
        self.get_logger().debug('Camera info: {}.'.format(msg))
        self._camera_info_sub.destroy()  # TODO: check that info was indeed received before destroying subscription

    def _vehicle_local_position_callback(self, msg):
        """Handles reception of latest global position estimate."""
        self.get_logger().debug('Vehicle local position callback triggered.')
        self._vehicle_local_position = msg
        self._update_map()

    def _match_script(self):
        """Does matching on camera and map images using provided script."""
        imwrite(self.img_file, self._cv_image)
        imwrite(self.map_file, self._map)
        cmd = '{} --input_pairs {} --input_dir {} --output_dir {} --superglue outdoor --viz'\
            .format(self.match_pairs_script, self.input_pairs, self.input_dir, self.output_dir)
        try:
            self.get_logger().debug('Matching image to map.')
            os.system(cmd)
        except Exception as e:
            self.get_logger().warn('Matching returned exception: {}\n{}'.format(e, traceback.print_exc()))

    def _match(self):
        """Does matching on camera and map images."""
        try:
            self.get_logger().debug('Matching image to map.')
            self._superglue.match(self._cv_image, self._map, self._camera_info.k.reshape([3, 3]))
        except Exception as e:
            self.get_logger().warn('Matching returned exception: {}\n{}'.format(e, traceback.print_exc()))


def main(args=None):
    rclpy.init(args=args)
    matcher = Matcher()
    rclpy.spin(matcher)
    matcher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
