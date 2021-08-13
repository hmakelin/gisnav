import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from px4_msgs.msg import VehicleLocalPosition
from owslib.wms import WebMapService


class Matcher(Node):

    # VehicleGlobalPosition not supported by microRTPS bridge - use VehicleLocalPosition instead
    vehicle_local_position_topic = "VehicleLocalPosition_PubSubTopic"

    def __init__(self):
        """Initializes the node."""
        super().__init__('matcher')
        self._init_wms()
        self._vehicle_local_position_sub = self.create_subscription(VehicleLocalPosition,
                                                                    self.vehicle_local_position_topic,
                                                                    self._vehicle_local_position_callback, 1)

    def _init_wms(self):
        """Initializes the Web Map Service (WMS) client used by the node to request map rasters.

        The url and version parameters are required to initialize the WMS client and are therefore set to read only. The
        layer and srs parameters can be changed dynamically.
        """
        # TODO: don't hardcode defaults here, get values from launch file
        self.declare_parameter('url', 'http://localhost:8080/geoserver/kestrel/wms',
                               ParameterDescriptor(read_only=True))
        self.declare_parameter('version', '1.1.1',
                               ParameterDescriptor(read_only=True))
        self.declare_parameter('layer', 'Ortho-RGB-Helsinki')
        self.declare_parameter('srs', 'EPSG:4326')

        self._wms = WebMapService(self.get_parameter('url').get_parameter_value().string_value,
                                  version=self.get_parameter('version').get_parameter_value().string_value)

    def _vehicle_local_position_callback(self, msg):
        """Handles reception of latest global position estimate."""
        self.get_logger().debug('Vehicle local position callback triggered: {}'.format(msg))
        self._vehicle_local_position = msg


def main(args=None):
    rclpy.init(args=args)
    matcher = Matcher()
    rclpy.spin(matcher)
    matcher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
