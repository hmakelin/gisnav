import rclpy
from rclpy.node import Node
from gisnav_msgs.msg import OrthoImage3D
from sensor_msgs.msg import NavSatFix, Image
from mavros_msgs.msg import HomePosition, GimbalDeviceAttitudeStatus
from geographic_msgs.msg import GeoPose
from mavros_msgs.msg import Altitude
from rclpy.node import Node

from gisnav import messaging
from gisnav.static_configuration import (
    CV_NODE_NAME,
    GIS_NODE_NAME,
    ROS_NAMESPACE,
    ROS_TOPIC_RELATIVE_CAMERA_QUATERNION,
    ROS_TOPIC_RELATIVE_GROUND_TRACK_ELEVATION,
    ROS_TOPIC_RELATIVE_GROUND_TRACK_GEOPOSE,
    ROS_TOPIC_RELATIVE_ORTHOIMAGE,
    ROS_TOPIC_RELATIVE_VEHICLE_ALTITUDE,
    ROS_TOPIC_RELATIVE_VEHICLE_ESTIMATED_ALTITUDE,
    ROS_TOPIC_RELATIVE_VEHICLE_ESTIMATED_GEOPOSE,
    ROS_TOPIC_RELATIVE_VEHICLE_GEOPOSE,
)
from gisnav_msgs.msg import OrthoImage3D  # type: ignore


class StateListener(Node):
    """
    A ROS node that subscribes to GISNav's output messages

    :param Node: Inherits from rclpy's Node class
    """

    def __init__(self):
        super().__init__('state_listener')

    @property
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_ORTHOIMAGE.replace("~", GIS_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def orthoimage(self) -> Optional[OrthoImage3D]:
        """Subscribed :term:`orthoimage` for :term:`pose` estimation"""

    @property
    @ROS.max_delay_ms(messaging.DELAY_DEFAULT_MS)
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_GROUND_TRACK_ELEVATION.replace("~", GIS_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def ground_track_elevation(self) -> Optional[Altitude]:
        """:term:`Ground track` :term:`elevation`, or None if not available or
        too old
        """

    @property
    @ROS.max_delay_ms(messaging.DELAY_DEFAULT_MS)
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_GROUND_TRACK_GEOPOSE.replace("~", GIS_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def ground_track_geopose(self) -> Optional[GeoPoseStamped]:
        """:term:`Ground track` :term:`geopose`, or None if not available or too old

        Complementary to the :attr:`ground_track_elevation`, includes lat and lon
        in atomic message
        """

    @property
    @ROS.max_delay_ms(messaging.DELAY_DEFAULT_MS)
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_VEHICLE_ALTITUDE.replace("~", GIS_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def altitude(self) -> Optional[Altitude]:
        """Altitude of vehicle, or None if unknown or too old"""

    @property
    # @ROS.max_delay_ms(messaging.DELAY_DEFAULT_MS)
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_CAMERA_QUATERNION.replace("~", GIS_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def camera_quaternion(self) -> Optional[Quaternion]:
        """:term:`Camera` :term:`orientation` as :class:`geometry_msgs.msg.Quaternion`
        message, or None if not available
        """

    @property
    # @ROS.max_delay_ms(messaging.DELAY_DEFAULT_MS)  # TODO: re-enable
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_VEHICLE_GEOPOSE.replace("~", GIS_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def geopose(self) -> Optional[GeoPoseStamped]:
        """Vehicle GeoPoseStamped, or None if not available or too old"""
