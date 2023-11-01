"""
This module contains a :term:`ROS 2` node for publishing messages to
:term:`RViz`. The node simplifies data visualization from across different
parts of the :term:`core` system, aiding development and debugging.

The below graph depicts how :class:`.RVizNode` publishes the :term:`vehicle` and
:term:`ground track` :term:`path` that can be susbcribed to and visualized by
:term:`RViz`, making it easier to see where GISNav thinks the vehicle is compared
to where the vehicle :term:`navigation filter` thinks it is:
"""
from collections import deque
from typing import Final, Optional

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from pyproj import Transformer
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from rclpy_decorators import ROS

from .. import messaging
from ..static_configuration import (
    POSE_NODE_NAME,
    ROS_NAMESPACE,
    ROS_TOPIC_RELATIVE_CAMERA_ESTIMATED_POSE,
)


class RVizNode(Node):
    """:term:`ROS 2` node that subscribes to GISNav :term:`core` output
    messages and publishes them into :term:`RViz`.
    """

    _MAX_POSE_STAMPED_MESSAGES: Final = 100
    """Max limit for held :class:`geometry_msgs.msg.PoseStamped` messages"""

    ROS_TOPIC_RELATIVE_CAMERA_ESTIMATED_PATH: Final = "~/camera/path"
    """Relative :term:`topic` into which this node publishes
    :attr:`.camera_path`
    """

    def __init__(self, *args, **kwargs):
        """Class initializer

        :param args: Positional arguments to parent :class:`.Node` constructor
        :param kwargs: Keyword arguments to parent :class:`.Node` constructor
        """
        super().__init__(*args, **kwargs)

        # Initialize ROS subscriptions by calling the decorated properties once
        self.camera_estimated_pose

        # Store poses for outgoing path messages in dedicated queues
        self._camera_estimated_path_queue: deque = deque(
            maxlen=self._MAX_POSE_STAMPED_MESSAGES
        )

        # Transforms latitude and longitude into pseudo-meters
        self._transformer = Transformer.from_crs("EPSG:4326", "EPSG:3857")

    @staticmethod
    def _append_pose_to_queue(pose: PoseStamped, queue: deque) -> None:
        """Appends the :class:`.PoseStamped` message to the :class:`.Path`
        queue

        :param pose: :class:`.PoseStamped` message to append
        :param queue: :class:`nav_msgs.msg.Path` queue to append to
        """
        # Update visualization if some time has passed, but not too soon. This
        # is mainly to prevent the Paths being much shorter in time for nodes
        # that would otherwise publish at much higher frequency (e.g. actual
        # GPS at 10 Hz vs GISNav mock GPS at 1 Hz). Also for visualization
        # a very frequent sample rate is not needed.
        if len(queue) > 0:
            if pose.header.stamp.sec - queue[-1].header.stamp.sec > 1.0:
                queue.append(pose)
            else:
                # Observation is too recent, return
                return None
        else:
            # Queue is empty
            queue.append(pose)

    def _append_camera_estimated_pose_to_queue(self, pose: PoseStamped) -> None:
        """Appends the :term:`camera` :term:`pose` message to the camera
         pose Path queue

        :param pose: :class:`.PoseStamped` message to append
        """
        self._append_pose_to_queue(pose, self._camera_estimated_path_queue)
        self.camera_estimated_path

    def _get_path(self, queue: deque) -> Path:
        """Returns :class:`nav_msgs.msg.Path` based on provided queue

        :param queue: Queue to turn into a :class:`nav_msgs.msg.Path` message
        :return: :class:`nav_msgs.msg.Path` message
        """
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"
        path.poses = list(queue)
        return path

    @property
    @ROS.max_delay_ms(messaging.DELAY_DEFAULT_MS)
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_CAMERA_ESTIMATED_POSE.replace("~", POSE_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=_append_camera_estimated_pose_to_queue,
    )
    def camera_estimated_pose(self) -> Optional[PoseStamped]:
        """Subscribed :term:`camera` :term:`geopose`, or None if not available
        or too old
        """

    @property
    @ROS.publish(
        ROS_TOPIC_RELATIVE_CAMERA_ESTIMATED_PATH,
        QoSPresetProfiles.SYSTEM_DEFAULT.value,
    )
    def camera_estimated_path(self) -> Optional[Path]:
        """Published :term:`camera` :term:`global position` estimated :term:`path`,
        or None if not available
        """
        return self._get_path(self._camera_estimated_path_queue)
