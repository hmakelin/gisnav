import rclpy
import sys
import os
import traceback
import xml.etree.ElementTree as ET
import yaml
import importlib

from enum import Enum
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from owslib.wms import WebMapService
from cv2 import VideoCapture, imwrite, imdecode
import numpy as np
import cv2  # TODO: remove
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

from wms_map_matching.util import get_bbox, setup_sys_path, convert_fov_from_pix_to_wgs84, \
    write_fov_and_camera_location_to_geojson, get_camera_apparent_altitude, get_camera_lat_lon, BBox,\
    Dimensions, get_camera_lat_lon_alt, MAP_RADIUS_METERS_DEFAULT, get_padding_size_for_rotation, rotate_and_crop_map

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

        self._cv_bridge = CvBridge()
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
            self._topics.update({self.TopicType.SUB: {topic_name: self._init_topic(topic_name, self.TopicType.SUB, msg_class)}})

        for topic_name, msg_type in self._config['ros2_topics']['pub'].items():
            module_name, msg_type = msg_type.rsplit('.', 1)
            msg_class = self._import_class(msg_type, module_name)
            self._topics.update({self.TopicType.PUB: {topic_name: self._init_topic(topic_name, self.TopicType.PUB, msg_class)}})

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
        img_size = (max_dim, max_dim)  # Map must be croppable to img dimensions when img is rotated 90 degrees
        return img_size

    def _camera_info(self):
        return self._topics_msgs.get('camera_info', None)

    def _map_size_with_padding(self):
        return get_padding_size_for_rotation(Dimensions(self._camera_info().width, self._camera_info().height))

    def _map_dimensions_with_padding(self):
        camera_info = self._camera_info()
        return Dimensions(*get_padding_size_for_rotation(Dimensions(camera_info.width, camera_info.height)))  #TODO: getting really messy!

    def _img_size(self):
        camera_info = self._camera_info()
        return camera_info.width, camera_info.height

    def _img_dimensions(self):
        return Dimensions(*self._img_size())

    def _update_map(self):
        """Gets latest map from WMS server and returns it as numpy array."""
        if self._use_gimbal_projection():
            raise NotImplementedError  # TODO
        else:
            self._map_bbox = get_bbox((self._topics_msgs['VehicleGlobalPosition_PubSubTopic'].lat, self._topics_msgs['VehicleGlobalPosition_PubSubTopic'].lon))

        if all(i is not None for i in [self._camera_info()]):
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
        else:
            self.get_logger().debug('Camera info not available - will not yet get map from WMS endpoint.')

    def _image_raw_callback(self, msg):
        """Handles reception of latest image frame from camera."""
        self.get_logger().debug('Camera image callback triggered.')
        self._topics_msgs['image_raw'] = msg
        self._cv_image = self._cv_bridge.imgmsg_to_cv2(msg, 'bgr8')
        if all(i is not None for i in [self._topics_msgs['image_raw'], self._map]):
            self._match()
        else:
            self.get_logger().debug('Map or image not available: map {}, img {} - not calling matching yet.'\
                                    .format(self._map is not None, self._cv_image is not None))


    def _get_camera_normal(self):
        # TODO: get actual camera normal via RTPS bridge - currently assumes nadir facing camera
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
        """Handles reception of latest gimbal pose."""
        self.get_logger().debug('Gimbal device attitude status callback triggered.')
        self._topics_msgs['GimbalDeviceAttitudeStatus'] = msg

    def _match(self):
        """Does matching on camera and map images. Publishes estimated e, f, h, and p matrices."""
        try:
            self.get_logger().debug('Matching image to map.')

            rot = self._topics_msgs['VehicleLocalPosition_PubSubTopic'].heading
            self.get_logger().debug('Current heading: {} radians, rotating map by {}.'\
                                    .format(self._topics_msgs['VehicleLocalPosition_PubSubTopic'].heading, rot))
            if rot is not None:  # TODO: what if it has not been initialized or is smth else than None? See assignemtn above - should be erplaces with some method call that warns/asserts
                map_rot = rotate_and_crop_map(self._map, rot, self._img_dimensions())
            else:
                self.get_logger().warn('Heading is unknown - skipping matching.')
                return

            h, fov_pix, translation_vector, rotation_vector = self._superglue.match(self._cv_image, map_rot, self._camera_info().k.reshape([3, 3]), self._img_dimensions(), self._get_camera_normal())

            # TODO: this part should be a lot different now with the map rotation refactoring, lots to do!
            if all(i is not None for i in (h, fov_pix, translation_vector, rotation_vector)):
                # TODO: should somehow control that self._map_bbox for example has not changed since match call was triggered
                fov_wgs84 = convert_fov_from_pix_to_wgs84(fov_pix, Dimensions(*self._map_size()),  # TODO: used Dimensions named tuple earlier, do not initialize it here
                                                          BBox(*self._map_bbox),  # TODO: convert to 'BBox' instance already much earlier, should already return this class for get_bbox function
                                                          rot, self._img_dimensions(), self.get_logger())

                ### OLD STUFF - COMMENTING OUT - PLAN IS TO GET GET LAT, LON, ALT from HOMOGRAPHY DECOMPOSITION #####
                apparent_alt = get_camera_apparent_altitude(MAP_RADIUS_METERS_DEFAULT, self._map_size(), self._camera_info().k)
                #self.get_logger().debug('Map camera apparent altitude: {}'.format(apparent_alt))  # TODO: do not use default value, use the specific value that was used for the map raster (or remove default altogheter)
                map_lat, map_lon = get_camera_lat_lon(BBox(*self._map_bbox))
                #self.get_logger().debug('Map camera lat lon: {}'.format((map_lat, map_lon)))  # TODO: ensure that same bbox is used as for matching, should be immutable for a matching pair
                ## Handle translation vector for drone camera
                #lat, lon, alt = get_camera_lat_lon_v2(translation_vector, rotation_vector, BBox(*self._map_bbox), Dimensions(*self._get_map_size()), rot, MAP_RADIUS_METERS_DEFAULT)  #TODO: do not use MAP_RADIUS_METERS_DEFAULT, use whaterver was actually used for getching the map raster
                #self.get_logger().debug('Drone lat lon alt: {} {} {}'.format(lat, lon, alt))
                ###########

                camera_position = get_camera_lat_lon_alt(translation_vector, rotation_vector, self._img_dimensions(), self._map_dimensions_with_padding(), BBox(*self._map_bbox), rot)  # TODO: the bbox is still for the old padded map, is that OK? should use get map dimensions?

                write_fov_and_camera_location_to_geojson(fov_wgs84, camera_position, (map_lat, map_lon, apparent_alt))
                #self._essential_mat_pub.publish(e)
                self._homography_mat_pub.publish(h)  # TODO: should remove this too? only lat-lon-alt and fov is published?
                #self._pose_pub.publish(p)
                self._fov_pub.publish(fov_wgs84)
            else:
                self.get_logger().warn('Not publishing h nor fov_pix since at least one of them was None.')
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
