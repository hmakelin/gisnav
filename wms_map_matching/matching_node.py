import rclpy
import sys
import os
import traceback
import xml.etree.ElementTree as ET
import yaml
import importlib
import math
import json

from enum import Enum
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from owslib.wms import WebMapService
from cv2 import VideoCapture, imwrite, imdecode
import numpy as np
import cv2  # TODO: remove
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from scipy.spatial.transform import Rotation

from wms_map_matching.util import get_bbox, setup_sys_path, convert_fov_from_pix_to_wgs84, \
    write_fov_and_camera_location_to_geojson, get_camera_apparent_altitude, get_camera_lat_lon, BBox,\
    Dimensions, get_camera_lat_lon_alt, MAP_RADIUS_METERS_DEFAULT, padded_map_size, rotate_and_crop_map,\
    visualize_homography, process_matches, uncrop_pixel_coordinates, get_fov, rotate_point, get_camera_distance,\
    get_distance_of_fov_center, altitude_from_gimbal_pitch

# Add the share folder to Python path
share_dir, superglue_dir = setup_sys_path()  # TODO: Define superglue_dir elsewhere? just use this to get share_dir

# Import this after util.setup_sys_path has been called
from wms_map_matching.superglue_adapter import SuperGlue


class Matcher(Node):

    class TopicType(Enum):
        """Enumerates microRTPS bridge topic types."""
        PUB = 1
        SUB = 2

    def __init__(self, share_directory, superglue_directory, config='config.yml'):
        """Initializes the node.

        Arguments:
            share_dir - String path of the share directory where configuration and other files are.
            superglue_dir - String path of the directory where SuperGlue related files are.
            config - String path to the config file in the share folder.
        """
        super().__init__('matcher')
        self.share_dir = share_directory  # TODO: make private?
        self.superglue_dir = superglue_directory  # TODO: move this to _setup_superglue? private _superglue_dir instead?
        self._load_config(config)
        self._init_wms()

        # Dict for storing all microRTPS bridge subscribers and publishers
        self._topics = dict()
        self._setup_topics()

        # Dict for storing latest microRTPS messages
        self._topics_msgs = dict()

        # Convert image_raw to cv2 compatible image and store it here
        self._cv_bridge = CvBridge()
        self._cv_image = None

        # Store map raster received from WMS endpoint here along with its bounding box
        self._map = None
        self._map_bbox = None

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

    def _import_class(self, class_name, module_name):
        """Imports class from module if not yet imported."""
        if module_name not in sys.modules:
            self.get_logger().info('Importing module ' + module_name + '.')
            importlib.import_module(module_name)
        imported_class = getattr(sys.modules[module_name], class_name, None)
        assert imported_class is not None, class_name + ' was not found in module ' + module_name + '.'
        return imported_class

    def _setup_topics(self):
        """Loads and sets up ROS2 publishers and subscribers from config file."""
        for topic_name, msg_type in self._config['ros2_topics']['sub'].items():
            module_name, msg_type = msg_type.rsplit('.', 1)
            msg_class = self._import_class(msg_type, module_name)
            self._init_topic(topic_name, self.TopicType.SUB, msg_class)

        for topic_name, msg_type in self._config['ros2_topics']['pub'].items():
            module_name, msg_type = msg_type.rsplit('.', 1)
            msg_class = self._import_class(msg_type, module_name)
            self._init_topic(topic_name, self.TopicType.PUB, msg_class)

        self.get_logger().info('Topics setup complete with keys: ' + str(self._topics.keys()))

    def _init_topic(self, topic_name, topic_type, msg_type):
        """Sets up rclpy publishers and subscribers and dynamically loads message types from px4_msgs library."""
        if topic_type is self.TopicType.PUB:
            self._topics[topic_name] = self.create_publisher(msg_type, topic_name, 10)
        elif topic_type is self.TopicType.SUB:
            callback_name = '_' + topic_name.lower() + '_callback'
            callback = getattr(self, callback_name, None)
            assert callback is not None, 'Missing callback implementation: ' + callback_name
            self._topics[topic_name] = self.create_subscription(msg_type, topic_name, callback, 10)
        else:
            raise TypeError('Unsupported topic type: {}'.format(topic_type))

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

    def _map_size(self):
        max_dim = max(self._camera_info().width, self._camera_info().height)
        return max_dim, max_dim

    def _camera_info(self):
        return self._topics_msgs.get('camera_info', None)

    def _map_size_with_padding(self):
        return padded_map_size(self._img_dimensions())

    def _map_dimensions_with_padding(self):
        return Dimensions(*self._map_size_with_padding())

    def _declared_img_size(self):
        camera_info = self._camera_info()
        if camera_info is not None:
            return camera_info.height, camera_info.width  # numpy order: h, w, c --> height first
        else:
            self.get_logger().warn('Camera info was not available - returning None as declared image size.')
            return None

    def _img_dimensions(self):
        return Dimensions(*self._declared_img_size())

    def _update_map(self):
        """Gets latest map from WMS server and returns it as numpy array."""
        if self._use_gimbal_projection():
            raise NotImplementedError  # TODO
        else:
            self._map_bbox = get_bbox((self._topics_msgs['VehicleGlobalPosition_PubSubTopic'].lat,
                                       self._topics_msgs['VehicleGlobalPosition_PubSubTopic'].lon))

        if self._camera_info() is not None:
            layer_str = self.get_parameter('layer').get_parameter_value().string_value
            srs_str = self.get_parameter('srs').get_parameter_value().string_value
            self.get_logger().debug('Getting map for bounding box: {}, layer: {}, srs: {}.'.format(self._map_bbox,
                                                                                                   layer_str, srs_str))

            try:
                self._map = self._wms.getmap(layers=[layer_str], srs=srs_str, bbox=self._map_bbox,
                                             size=self._map_size_with_padding(), format='image/png',
                                             transparent=True)
            except Exception as e:
                self.get_logger().warn('Exception from WMS server query: {}\n{}'.format(e, traceback.print_exc()))
                return

            self._map = np.frombuffer(self._map.read(), np.uint8)
            self._map = imdecode(self._map, cv2.IMREAD_UNCHANGED)
            assert self._map.shape[0:2] == self._map_size_with_padding(), 'Converted map is not the specified size.'
        else:
            self.get_logger().debug('Camera info not available - will not yet get map from WMS endpoint.')

    def _image_raw_callback(self, msg):
        """Handles reception of latest image frame from camera."""
        self.get_logger().debug('Camera image callback triggered.')
        self._topics_msgs['image_raw'] = msg
        self._cv_image = self._cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        img_size = self._declared_img_size()
        if img_size is not None:
            assert self._cv_image.shape[0:2] == self._declared_img_size(), 'Converted _cv_image shape {} did not match ' \
                                                                  'declared image shape {}.'.format(
                self._cv_image.shape[0:2], self._declared_img_size())
        self._match()

    def _get_camera_normal(self):
        # TODO: get actual camera normal via RTPS bridge - currently assumes nadir facing camera
        # np.array([[0, 0, 1]])  # TODO: hsould return this instead?
        return np.array([0, 0, 1])

    def _camera_info_callback(self, msg):
        """Handles reception of camera info."""
        self.get_logger().debug('Camera info callback triggered.')
        self._topics_msgs['camera_info'] = msg
        self.get_logger().debug('Camera info: ' + str(msg))
        self._topics['camera_info'].destroy()  # TODO: check that info was indeed received before destroying subscription

    def _vehiclelocalposition_pubsubtopic_callback(self, msg):
        """Handles reception of latest local position estimate."""
        self.get_logger().debug('Vehicle local position callback triggered.')
        self._topics_msgs['VehicleLocalPosition_PubSubTopic'] = msg

    def _vehicleglobalposition_pubsubtopic_callback(self, msg):
        """Handles reception of latest global position estimate."""
        self.get_logger().debug('Vehicle global position callback triggered.')
        self._topics_msgs['VehicleGlobalPosition_PubSubTopic'] = msg
        self._update_map()

    def _gimbaldeviceattitudestatus_pubsubtopic_callback(self, msg):
        """Handles reception of GimbalDeviceAttitudeStatus messages."""
        self.get_logger().debug('Gimbal device attitude status callback triggered: ' + str(msg) + '.')
        self._topics_msgs['GimbalDeviceAttitudeStatus'] = msg

    def _gimbaldevicesetattitude_pubsubtopic_callback(self, msg):
        """Handles reception of GimbalDeviceSetAttitude messages."""
        self.get_logger().debug('Gimbal device set attitude callback triggered: ' + str(msg) + '.')
        self._topics_msgs['GimbalDeviceSetAttitude'] = msg

    def _vehicleattitude_pubsubtopic_callback(self, msg):
        """Handles reception of VehicleAttitude messages."""
        self.get_logger().debug('Vehicle attitude callback triggered: ' + str(msg) + '.')
        self._topics_msgs['VehicleAttitude'] = msg

    def _camera_pitch(self):
        """Returns camera pitch in degrees from GimbalDeviceAttitudeStatus or GimbalDeviceSetAttitude message."""
        #### Print vehicle attitude - is it included in GimbalSetAttitude etc.? so is gimbal attitude relative to NED or relative to vehicle body?####
        #vehicle_attitude = self._topics_msgs.get('VehicleAttitude', None)
        #print('vehicle attitude: ' + str(Rotation.from_quat(vehicle_attitude.q).as_euler('zxy', degrees=True)))
        #######
        attitude_status = self._topics_msgs.get('GimbalDeviceAttitudeStatus', None)
        if attitude_status is None:
            # Try alternative topic
            self.get_logger().warn('GimbalDeviceAttitudeStatus not available. Trying GimbalDeviceSetAttitude instead.')
            attitude_status = self._topics_msgs.get('GimbalDeviceSetAttitude', None)
        if attitude_status is not None:
            assert hasattr(attitude_status, 'q')
            # TODO: need to add vehicle attitude to gimbal attitude to get attitude in NED frame? Warn if vehicleattitude not available and just use gimbal attitude alone instead.
            euler_angles = Rotation.from_quat(attitude_status.q).as_euler('zxy', degrees=True)
            assert len(euler_angles) == 3, 'Unexpected length of euler angles vector: ' + str(len(euler_angles))
            return 180+euler_angles[2]
        else:
            self.get_logger().warn('Camera pitch currently not available.')
            return None

    def _match(self):
        """Matches camera image to map image and computes camera position and field of view."""
        try:
            self.get_logger().debug('Matching image to map.')

            if self._map is None:
                self.get_logger().warn('Map not yet available - skipping matching.')
                return

            local_position = self._topics_msgs.get('VehicleLocalPosition_PubSubTopic', None)
            if local_position is None:
                self.get_logger().warn('VehicleLocalPosition is unknown, cannot get heading. Skipping matching.')
                return

            assert hasattr(local_position, 'heading'), 'Heading information missing from VehicleLocalPosition message.'
            rot = local_position.heading
            assert -math.pi <= rot <= math.pi, 'Unexpected heading value: ' + str(rot) + ' ([-pi, pi] expected).'
            self.get_logger().debug('Current heading: ' + str(rot) + ' radians.')
            map_cropped, map_rotated = rotate_and_crop_map(self._map, rot, self._img_dimensions())  # TODO: return only rotated
            assert map_cropped.shape[0:2] == self._declared_img_size(), 'Rotated and cropped map did not match image shape.'

            mkp_img, mkp_map = self._superglue.match(self._cv_image, map_cropped)

            assert len(mkp_img) == len(mkp_map), 'Matched keypoint counts did not match.'
            if len(mkp_img) < 4:
                self.get_logger().warn('Did not find enough matches. Skipping current matches.')
                return

            h, h_mask, translation_vector, rotation_matrix = process_matches(mkp_img, mkp_map,
                                                                             self._camera_info().k.reshape([3, 3]),
                                                                             self._get_camera_normal(),
                                                                             logger=self._logger,
                                                                             affine=self._config['superglue']['misc']['affine'])

            assert h.shape == (3, 3), 'Homography matrix had unexpected shape: ' + str(h.shape) + '.'
            assert translation_vector.shape == (3, 1), 'Translation vector had unexpected shape: '\
                                                       + str(translation_vector.shape) + '.'
            assert rotation_matrix.shape == (3, 3), 'Rotation matrix had unexpected shape: '\
                                                    + str(rotation_matrix.shape) + '.'

            fov_pix = get_fov(self._cv_image, h)
            visualize_homography('Matches and FoV', self._cv_image, map_cropped, mkp_img, mkp_map, fov_pix) # TODO: separate calculation of fov_pix from their visualization!

            #apparent_alt = get_camera_apparent_altitude(MAP_RADIUS_METERS_DEFAULT, self._map_dimensions_with_padding(), self._camera_info().k)
            map_lat, map_lon = get_camera_lat_lon(BBox(*self._map_bbox))  # TODO: this is just the center of the map which is GPS location with current implementation - need to fix get_camera_lat_lon implementation

            fov_wgs84, fov_uncropped, fov_unrotated = convert_fov_from_pix_to_wgs84(fov_pix, self._map_dimensions_with_padding(), self._map_bbox,
                                                      rot, self._img_dimensions())

            # Compute camera altitude, and distance to principal point using triangle similarity
            camera_distance = get_camera_distance(self._camera_info().k[0], self._img_dimensions().width, get_distance_of_fov_center(fov_wgs84))
            camera_pitch = self._camera_pitch()
            camera_altitude = None
            if camera_pitch is None:
                # TODO: Use some other method to estimate altitude if pitch not available?
                self.get_logger().warn('Camera pitch not available - cannot estimate altitude visually.')
            else:
                camera_altitude = math.sin(math.radians(camera_pitch))*camera_distance
            self.get_logger().debug('Camera pitch, distance to principal point, altitude: {} deg, {} m, {} m.'
                                    .format(camera_pitch, camera_distance, camera_altitude))

            #### TODO: remove this debugging section
            mkp_map_uncropped = []
            for i in range(0, len(mkp_map)):
                mkp_map_uncropped.append(list(uncrop_pixel_coordinates(self._img_dimensions(), self._map_dimensions_with_padding(), mkp_map[i])))
            mkp_map_uncropped = np.array(mkp_map_uncropped)

            mkp_map_unrotated = []
            for i in range(0, len(mkp_map_uncropped)):
                mkp_map_unrotated.append(list(rotate_point(rot, self._map_dimensions_with_padding(), mkp_map_uncropped[i])))
            mkp_map_unrotated = np.array(mkp_map_unrotated)

            h2, h_mask2, translation_vector2, rotation_matrix2 = process_matches(mkp_img, mkp_map_unrotated, # mkp_map_uncropped,
                                                                             self._camera_info().k.reshape([3, 3]),
                                                                             self._get_camera_normal(),
                                                                             logger=self._logger,
                                                                             affine=self._config['superglue']['misc']['affine'])

            fov_pix_2 = get_fov(self._cv_image, h2)
            visualize_homography('Uncropped and unrotated', self._cv_image, self._map, mkp_img, mkp_map_unrotated, fov_pix_2) # TODO: separate calculation of fov_pix from their visualization!
            #### END DEBUG SECTION ###

            # TODO: fix this calculation
            camera_position = get_camera_lat_lon_alt(translation_vector, rotation_matrix, self._img_dimensions(),
                                                     self._map_dimensions_with_padding(), self._map_bbox, rot)  # TODO: the bbox is still for the old padded map, is that OK? should use get map dimensions?

            #write_fov_and_camera_location_to_geojson(fov_wgs84, camera_position, (map_lat, map_lon, apparent_alt))
            write_fov_and_camera_location_to_geojson(fov_wgs84, camera_position, (map_lat, map_lon, camera_distance))
            #self._essential_mat_pub.publish(e)
            #self._homography_mat_pub.publish(h)
            #self._pose_pub.publish(p)
            self._topics['fov'].publish(fov_wgs84)

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
