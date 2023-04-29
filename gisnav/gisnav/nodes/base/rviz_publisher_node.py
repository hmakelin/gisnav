"""Abstract base class for nodes that publish data to rviz2"""
from collections import deque

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
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

    def _publish_path(self):
        """Publishes :class:`nav_msgs.msg.Path` for debugging and visualization"""
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"
        path.poses = list(self._pose_stamped_queue)
        self._path_publisher.publish(path)

    def publish_rviz(self, pose_stamped: PoseStamped) -> None:
        """Publish :class:`geometry_msgs.msg.PoseStamped` and
        :class:`nav_msgs.msg.Path` messages"""
        self._pose_stamped_queue.append(pose_stamped)
        self._publish_path()
        self._pose_stamped_publisher.publish(pose_stamped)
