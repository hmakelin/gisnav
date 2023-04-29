"""Abstract base class for nodes that publish data to rviz2"""
from collections import deque

from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from pyproj import Transformer
from rclpy.qos import QoSPresetProfiles

from .base_node import BaseNode


class RVizPublisherNode(BaseNode):
    """Abstract base class for nodes that publish data to rviz2"""

    #: Publishing topic for :class:`geometry_msgs.msg.PoseStamped` messages for rviz
    ROS_TOPIC_POSE_STAMPED = "~/pose_stamped"

    #: Publishing topic for :class:`nav_msgs.msg.Path` messages for rviz
    ROS_TOPIC_PATH = "~/path"

    #: Max limit for held :class:`geometry_msgs.msg.PoseStamped` messages
    _MAX_POSE_STAMPED_MESSAGES = 100

    def __init__(self, name: str):
        """
        Initializes the node

        :param name: Node name
        """
        super().__init__(name)

        self._pose_stamped_publisher = self.create_publisher(
            PoseStamped,
            self.ROS_TOPIC_POSE_STAMPED,
            QoSPresetProfiles.SYSTEM_DEFAULT.value,
        )
        self._path_publisher = self.create_publisher(
            Path,
            self.ROS_TOPIC_PATH,
            QoSPresetProfiles.SYSTEM_DEFAULT.value,
        )
        self._pose_stamped_queue: deque = deque(maxlen=self._MAX_POSE_STAMPED_MESSAGES)
        self._transformer = Transformer.from_crs("EPSG:4326", "EPSG:3857")

    def _publish_path(self):
        """Publishes :class:`nav_msgs.msg.Path` for debugging and visualization"""
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"
        path.poses = list(self._pose_stamped_queue)
        self._path_publisher.publish(path)

    def publish_rviz(self, geopose_stamped: GeoPoseStamped, alt_agl: float) -> None:
        """
        Publish :class:`geometry_msgs.msg.PoseStamped` and
        :class:`nav_msgs.msg.Path` messages

        :param geopose_stamped: Vehicle geopose
        :param alt_agl: Vehicle altitude above ground level (RViz config is
            assumed to define a planar ground surface at z=0)
        """
        # Convert latitude, longitude, and altitude to Cartesian coordinates
        x, y = self._transformer.transform(
            geopose_stamped.pose.position.latitude,
            geopose_stamped.pose.position.longitude,
        )
        z = alt_agl

        # Create a PoseStamped message
        pose_stamped = PoseStamped()
        pose_stamped.header = geopose_stamped.header
        pose_stamped.header.frame_id = "map"

        # TODO: x should be easting but re-centered to 0 for ksql_airport.world
        pose_stamped.pose.position.x = x + 13609376

        # TODO: y should be northing but re-centered to 0 for ksql_airport.world
        pose_stamped.pose.position.y = y - 4512349
        pose_stamped.pose.position.z = z
        pose_stamped.pose.orientation = geopose_stamped.pose.orientation

        # Update visualization if some time has passed, but not too soon. This
        # is mainly to prevent the Paths being much shorter in time for nodes
        # that would otherwise publish at much higher frequency (e.g. actual
        # GPS at 10 Hz vs GISNav mock GPS at 1 Hz)
        if len(self._pose_stamped_queue) > 0:
            if (
                pose_stamped.header.stamp.sec
                - self._pose_stamped_queue[-1].header.stamp.sec
                > 1.0
            ):
                self._pose_stamped_queue.append(pose_stamped)
            else:
                # Observation is too recent, return
                return
        else:
            # Queue is empty
            self._pose_stamped_queue.append(pose_stamped)

        assert len(self._pose_stamped_queue) > 0
        self._publish_path()
        self._pose_stamped_publisher.publish(pose_stamped)
