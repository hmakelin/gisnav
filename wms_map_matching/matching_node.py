import rclpy
import sys
import os
import traceback
import xml.etree.ElementTree as ET
import yaml

from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from px4_msgs.msg import VehicleLocalPosition
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

# Import this after superglue_dir has been added to path
from wms_map_matching.superglue_adapter import SuperGlue

class Matcher(Node):
    # Determines map size, radius of enclosed circle in meters
    map_bbox_radius = 200

    def __init__(self, config='config.yml', use_script=False):
        """Initializes the node.

        Arguments:
            config - String path to the config file in the share folder.
            use_script - Boolean flag for whether to use match_pairs.py script from SuperGlue.
        """
        super().__init__('matcher')
        self._load_config(config)
        self._use_script = use_script
        self._init_wms()
        self._setup_topics()
        self._cv_bridge = CvBridge()
        self._camera_info = None
        self._image_raw = None
        self._cv_image = None
        self._map = None
        self._superglue = None
        self._setup_superglue()


    def _setup_superglue(self):
        """Sets up SuperGlue."""
        self.match_pairs_script = os.path.join(self.superglue_dir, 'match_pairs.py')
        self.images_dir = os.path.join(share_dir, 'images')
        self.input_pairs = os.path.join(images_dir, 'input_pairs.txt')
        self.input_dir = os.path.join(images_dir, 'input')
        self.output_dir = os.path.join(images_dir, 'output')
        self.img_file_name = 'img.jpg'
        self.map_file_name = 'map.jpg'
        self.img_file = os.path.join(self.input_dir, self.img_file_name)
        self.map_file = os.path.join(input_dir, self.map_file_name)
        self.output_file = os.path.join(output_dir, 'matches.jpg')
        if self._use_script:
            self._create_superglue_dirs()
            self._create_superglue_input_pairs_file()
        else:
            self._superglue = SuperGlue(self.output_file, self.get_logger())

    def _load_config(self, yaml_file):
        """Loads config from the provided YAML file."""
        with open(os.path.join(share_dir, yaml_file), 'r') as f:
            try:
                self._config = yaml.safe_load(f)
                self.get_logger().info('Loaded config:\n{}.'.format(self._config))
            except Exception as e:
                self.get_logger().error('Could not load config file {} because of exception: {}\n{}'\
                                        .format(yaml_file, e, traceback.print_exc()))

    def _setup_topics(self):
        """Loads and sets up ROS2 publishers and subscribers from config file."""
        sub_vehicle_local_position_topic = self._config['ros2_topics']['sub']['vehicle_local_position']
        sub_image_raw_topic = self._config['ros2_topics']['sub']['image_raw']
        sub_camera_info_topic = self._config['ros2_topics']['sub']['camera_info']
        pub_essential_mat_topic = self._config['ros2_topics']['pub']['essential_matrix']
        pub_fundamental_mat_topic = self._config['ros2_topics']['pub']['fundamental_matrix']
        pub_homography_mat_topic = self._config['ros2_topics']['pub']['homography_matrix']
        pub_pose_topic = self._config['ros2_topics']['pub']['pose']

        self._image_raw_sub = self.create_subscription(Image, self.image_raw_topic, self._image_raw_callback, 10)
        self._camera_info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self._camera_info_callback,
                                                         10)
        self._vehicle_local_position_sub = self.create_subscription(VehicleLocalPosition,
                                                                    self.vehicle_local_position_topic,
                                                                    self._vehicle_local_position_callback, 10)
        self._essential_mat_pub = self.create_publisher(Float64MultiArray, self.pub_essential_mat_topic, 10)
        self._fundamental_mat_pub = self.create_publisher(Float64MultiArray, self.pub_fundamental_mat_topic, 10)
        self._homography_mat_pub = self.create_publisher(Float64MultiArray, self.pub_homography_mat_topic, 10)
        self._pose_pub = self.create_publisher(Float64MultiArray, self.pub_pose_topic, 10)

    def _create_superglue_dirs(self):
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

    def _create_superglue_input_pairs_file(self):
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
        self._map_bbox = get_bbox((self._vehicle_local_position.ref_lat, self._vehicle_local_position.ref_lon),
                                  self.map_bbox_radius)

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
        stamp = msg.header.stamp.sec
        now = self.get_clock().now().to_msg().sec
        if stamp + 1 < now:
            self.get_logger().debug('Skipping frame older than 1 second ({} vs {})'.format(stamp, now))
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
            map_rot = cv2.rotate(self._map, rot)

            e, f, h, p = self._superglue.match(self._cv_image, map_rot, self._camera_info.k.reshape([3, 3]))  #self._map

            if all(i is not None for i in (e, f, h, p)):
                self.get_logger().debug('Publishing e, f, h, and p.')
                self._essential_mat_pub.publish(e)
                self._fundamental_mat_pub.publish(f)
                self._homography_mat_pub.publish(h)
                self._pose_pub.publish(p)
            else:
                self.get_logger().warn('Not publishing e, f, h, nor p since at least one of them was None.')
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
