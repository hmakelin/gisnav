import rclpy
from rclpy.node import Node
from gisnav_msgs.msg import OrthoImage3D
from sensor_msgs.msg import NavSatFix, Image
from mavros_msgs.msg import HomePosition, GimbalDeviceAttitudeStatus
from geographic_msgs.msg import GeoPose
from mavros_msgs.msg import Altitude

class StateListener(Node):
    """
    A ROS node that subscribes to GISNav's output messages

    :param Node: Inherits from rclpy's Node class
    """

    def __init__(self):
        super().__init__('state_listener')

        # Subscribers for each message type
        self.create_subscription(GeoPose, 'gisnav/gis_node/vehicle/geopose', self.geopose_callback, 10)
        self.create_subscription(Altitude, 'gisnav/gis_node/vehicle/altitude', self.altitude_callback, 10)

    def geopose_callback(self, msg):
        self.get_logger().info('Received GeoPose message')

    def altitude_callback(self, msg):
        self.get_logger().info('Received Altitude message')
