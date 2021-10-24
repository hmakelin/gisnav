import rclpy
import sys
import os
import traceback
import xml.etree.ElementTree as ET
import yaml

from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from px4_msgs.msg import VehicleLocalPosition, VehicleGlobalPosition, GimbalDeviceInformation
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64MultiArray
from owslib.wms import WebMapService
from cv2 import VideoCapture, imwrite, imdecode
import numpy as np
import cv2  # TODO: remove
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

from wms_map_matching.util import get_bbox, get_nearest_cv2_rotation, setup_sys_path

# Add the share folder to Python path
share_dir, superglue_dir = setup_sys_path()  # TODO: Define superglue_dir elsewhere? just use this to get share_dir

# Import this after util.setup_sys_path has been called
from wms_map_matching.superglue_adapter import SuperGlue

class Matcher(Node):
    def __init__(self, share_dir, superglue_dir, config='config.yml'):
        """Initializes the node.

        Arguments:
            share_dir - String path of the share directory where configuration and other files are.
            superglue_dir - String path of the directory where SuperGlue related files are.
            config - String path to the config file in the share folder.
        """
        super().__init__('matcher')
        self.share_dir = share_dir  # TODO: make private?
        self.superglue_dir = superglue_dir  # TODO: move this to _setup_superglue? private _superglue_dir instead?
        self._load_config(config)
        self._init_wms()
        self._setup_topics()
        self._cv_bridge = CvBridge()
        self._camera_info = None
        self._vehicle_local_position = None  # TODO: remove the redundant initialization of these from constructor?
        self._vehicle_global_position = None
        self._gimbal_device_information = None
        self._image_raw = None
        self._cv_image = None
        self._map = None
        self._setup_superglue()


    def _setup_superglue(self):
        """Sets up SuperGlue."""  # TODO: make all these private?
        self._superglue = SuperGlue(self._config['superglue'], self.get_logger())

    def _load_config(self, yaml_file):
        """Loads config from the provided YAML file."""
        with open(os.path.join(share_dir, yaml_file), 'r') as f:
            try:
                self._config = yaml.safe_load(f)
                self.get_logger().info('Loaded config:\n{}.'.format(self._config))
            except Exception as e:
                self.get_logger().error('Could not load config file {} because of exception: {}\n{}'\
                                        .format(yaml_file, e, traceback.print_exc()))

    def _use_gimbal_projection(self):
        """Returns True if gimbal projection is enabled for fetching map bbox rasters."""
        return self._config['superglue']['misc']['gimbal_projection']

    def _setup_topics(self):
        """Loads and sets up ROS2 publishers and subscribers from config file."""
        if self._use_gimbal_projection():
            self._sub_gimbal_device_information_topic = self._config['ros2_topics']['sub']['gimbal_device_information']
            self._gimbal_device_information_sub = self.create_subscription(GimbalDeviceInformation,
                                                                           self._sub_gimbal_device_information_topic,
                                                                           self._gimbal_device_information_callback, 10)
        self._sub_vehicle_local_position_topic = self._config['ros2_topics']['sub']['vehicle_local_position']
        self._sub_vehicle_global_position_topic = self._config['ros2_topics']['sub']['vehicle_global_position']
        self._sub_image_raw_topic = self._config['ros2_topics']['sub']['image_raw']
        self._sub_camera_info_topic = self._config['ros2_topics']['sub']['camera_info']
        self._pub_essential_mat_topic = self._config['ros2_topics']['pub']['essential_matrix']
        self._pub_homography_mat_topic = self._config['ros2_topics']['pub']['homography_matrix']
        self._pub_pose_topic = self._config['ros2_topics']['pub']['pose']

        self._image_raw_sub = self.create_subscription(Image, self._sub_image_raw_topic, self._image_raw_callback, 10)
        self._camera_info_sub = self.create_subscription(CameraInfo, self._sub_camera_info_topic, self._camera_info_callback,
                                                         10)
        self._vehicle_local_position_sub = self.create_subscription(VehicleLocalPosition,
                                                                    self._sub_vehicle_local_position_topic,
                                                                    self._vehicle_local_position_callback, 10)
        self._vehicle_global_position_sub = self.create_subscription(VehicleGlobalPosition,
                                                                     self._sub_vehicle_global_position_topic,
                                                                     self._vehicle_global_position_callback, 10)
        self._essential_mat_pub = self.create_publisher(Float64MultiArray, self._pub_essential_mat_topic, 10)
        self._homography_mat_pub = self.create_publisher(Float64MultiArray, self._pub_homography_mat_topic, 10)
        self._pose_pub = self.create_publisher(Float64MultiArray, self._pub_pose_topic, 10)

    def _init_wms(self):
        """Initializes the Web Map Service (WMS) client used by the node to request map rasters.

        The url and version parameters are required to initialize the WMS client and are therefore set to read only. The
        layer and srs parameters can be changed dynamically.
        """
        self.declare_parameter('url', self._config['wms']['url'], ParameterDescriptor(read_only=True))
        self.declare_parameter('version', self._config['wms']['version'], ParameterDescriptor(read_only=True))
        self.declare_parameter('layer', self._config['wms']['layer'])
        self.declare_parameter('srs', self._config['wms']['srs'])

        try:
            self._wms = WebMapService(self.get_parameter('url').get_parameter_value().string_value,
                                      version=self.get_parameter('version').get_parameter_value().string_value)
        except Exception as e:
            self.get_logger().error('Could not connect to WMS server.')
            raise e

    def _update_map(self):
        """Gets latest map from WMS server and returns it as numpy array."""
        if self._use_gimbal_projection():
            raise NotImplementedError  # TODO
        else:
            self._map_bbox = get_bbox((self._vehicle_global_position.lat, self._vehicle_global_position.lon))

        if all(i is not None for i in [self._camera_info]):
            max_dim = max(self._camera_info.width, self._camera_info.height)
            img_size = (max_dim, max_dim)  # Map must be croppable to img dimensions when img is rotated 90 degrees
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
        self._image_raw = msg
        self._cv_image = self._cv_bridge.imgmsg_to_cv2(self._image_raw, 'bgr8')
        if all(i is not None for i in [self._image_raw, self._map]):
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
        """Handles reception of latest local position estimate."""
        self.get_logger().debug('Vehicle local position callback triggered.')
        self._vehicle_local_position = msg

    def _vehicle_global_position_callback(self, msg):
        """Handles reception of latest global position estimate."""
        self.get_logger().debug('Vehicle global position callback triggered.')
        self._vehicle_global_position = msg
        self._update_map()

    def _gimbal_device_information_callback(self, msg):
        """Handles reception of latest gimbal pose."""
        self.get_logger().debug('Gimbal device information callback triggered.')
        self._gimbal_device_information = msg

    def _match(self):
        """Does matching on camera and map images. Publishes estimated e, f, h, and p matrices."""
        try:
            self.get_logger().debug('Matching image to map.')

            # TODO: try to put this code somewhere into the SuperGlue adapter to keep this method cleaner
            # Rotate map to adjust for SuperGlue's rotation non-invariance, then crop the map to match img dimensions
            # Assumes map is square (so that dimensions do not change when rotation by multiples of 90 degrees)
            # When transposing back to NED frame, must account for rotation and cropping in map keypoint coordinates
            rot = get_nearest_cv2_rotation(self._vehicle_local_position.heading)
            self.get_logger().debug('Current heading: {} radians, rotating map by {}.'\
                                    .format(self._vehicle_local_position.heading, rot))
            if rot is not None:
                map_rot = cv2.rotate(self._map, rot)
            else:
                map_rot = self._map

            e, h, r, t = self._superglue.match(self._cv_image, map_rot, self._camera_info.k.reshape([3, 3]))  #self._map

            if all(i is not None for i in (e, h, r, t)):
                self.get_logger().debug('Publishing e, h, and p.')
                p = np.append(np.array(r), np.array(t), axis=1)
                self.get_logger().debug('Pose p=\n{}.\n'.format(p))
                self._essential_mat_pub.publish(e)
                self._homography_mat_pub.publish(h)
                self._pose_pub.publish(p)
            else:
                self.get_logger().warn('Not publishing e, h, nor p since at least one of them was None.')
        except Exception as e:
            self.get_logger().warn('Matching returned exception: {}\n{}'.format(e, traceback.print_exc()))


def main(args=None):
    rclpy.init(args=args)
    matcher = Matcher(share_dir, superglue_dir)
    rclpy.spin(matcher)
    matcher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
