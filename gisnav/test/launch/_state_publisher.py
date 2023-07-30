import rclpy
from rclpy.node import Node
from gisnav_msgs.msg import OrthoImage3D
from sensor_msgs.msg import NavSatFix, Image
from mavros_msgs.msg import HomePosition, GimbalDeviceAttitudeStatus
from geographic_msgs.msg import GeoPose
from mavros_msgs.msg import Altitude

class StatePublisher(Node):
    """
    A ROS node that publishes full GISNav state messages.

    This node publishes messages including NavSatFix, GeoPose, Altitude, HomePosition,
    GimbalDeviceAttitudeStatus, Image, and OrthoImage3D. These messages describe the
    full state of the :term:`vehicle`.

    :param Node: Inherits from rclpy's Node class
    """

    def __init__(self):
        super().__init__('state_publisher')

        # Publishers for each message type
        self.navsatfix_pub = self.create_publisher(NavSatFix, 'mavros/global_position/global', 10)
        self.geopose_pub = self.create_publisher(GeoPose, 'gisnav/gis_node/vehicle/geopose', 10)
        self.altitude_pub = self.create_publisher(Altitude, 'gisnav/gis_node/vehicle/altitude', 10)
        self.home_position_pub = self.create_publisher(HomePosition, 'mavros/home_position/home', 10)
        self.gimbal_attitude_pub = self.create_publisher(GimbalDeviceAttitudeStatus, 'mavros/gimbal_control/device/attitude_status', 10)
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.ortho_image_3d_pub = self.create_publisher(OrthoImage3D, 'gisnav_msgs/OrthoImage3D', 10)

        # Timer to publish messages periodically
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_messages)

    def publish_full_state(self):
        """
        Publishes the full state messages
        """
        # TODO: call on a timer?
        # Create and publish the messages
        # TODO: use generate_state function to create messages
        self.navsatfix_pub.publish(NavSatFix())
        self.geopose_pub.publish(GeoPose())
        self.altitude_pub.publish(Altitude())
        self.home_position_pub.publish(HomePosition())
        self.gimbal_attitude_pub.publish(GimbalDeviceAttitudeStatus())
        self.image_pub.publish(Image())
        self.ortho_image_3d_pub.publish(OrthoImage3D())

