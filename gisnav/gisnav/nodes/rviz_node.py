"""Rviz Publisher Node

This module contains a ROS2 node for publishing various messages into rviz2.
It is intended for interfacing with various ROS topics including camera,
MAVROS, GISNode, CVNode, and MockGPSNode. This node facilitates development by
consolidating data from across the system and makes it visualizable.
"""
from collections import deque
from typing import Optional

from geographic_msgs.msg import GeoPointStamped, GeoPoseStamped
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import HomePosition
from nav_msgs.msg import Path
from pyproj import Transformer
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from ..assertions import ROS, narrow_types
from . import messaging


class RVizNode(Node):
    """RViz publisher node"""

    _MAX_POSE_STAMPED_MESSAGES = 100
    """Max limit for held :class:`geometry_msgs.msg.PoseStamped` messages"""

    def __init__(self, name: str):
        """Initializes the node

        :param name: Node name
        """
        super().__init__(name)

        # Initialize ROS subscriptions
        self.home_position
        self.geopose
        self.geopose_estimate
        self.ground_track_geopoint

        self._path_queue: deque = deque(maxlen=self._MAX_POSE_STAMPED_MESSAGES)
        self._path_estimate_queue: deque = deque(maxlen=self._MAX_POSE_STAMPED_MESSAGES)
        self._ground_track_path_queue: deque = deque(
            maxlen=self._MAX_POSE_STAMPED_MESSAGES
        )

        self._transformer = Transformer.from_crs("EPSG:4326", "EPSG:3857")

    def _home_position_callback(self, msg: HomePosition):
        """Callback for :class:`mavros_msgs.msg.HomePosition` messages for
        subscribed to by :attr:`.home_position`

        Stores the conversion from global to local frame whenever home position
        changes (expected to only happen once at the beginning).
        """
        # Convert latitude, longitude, and altitude to Cartesian coordinates
        x, y = self._transformer.transform(
            msg.geo.latitude,
            msg.geo.longitude,
        )

    @property
    @ROS.max_delay_ms(messaging.DELAY_SLOW_MS)
    @ROS.subscribe(
        messaging.ROS_TOPIC_HOME_POSITION,
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=_home_position_callback,
    )
    def home_position(self) -> Optional[HomePosition]:
        """Vehicle :class:`mavros_msgs.msg.HomePosition` (defined as local
        frame origin), or None if unknown or too old
        """

    @property
    @ROS.max_delay_ms(messaging.DELAY_DEFAULT_MS)
    @ROS.subscribe(
        messaging.ROS_TOPIC_VEHICLE_GEOPOSE,
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=lambda pose: _append_geopose_to_path(msg, self._path_queue),
    )
    def geopose(self) -> Optional[GeoPoseStamped]:
        """Vehicle :class:`.geographic_msgs.msg.GeoPoseStamped`, or None if
        not available or too old
        """

    @property
    @ROS.max_delay_ms(messaging.DELAY_DEFAULT_MS)
    @ROS.subscribe(
        messaging.ROS_TOPIC_VEHICLE_GEOPOSE_ESTIMATE,
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=lambda msg: _append_geopose_to_path(msg, self._path_estimate_queue),
    )
    def geopose_estimate(self) -> Optional[GeoPoseStamped]:
        """Vehicle est:class:`.geographic_msgs.msg.GeoPoseStamped` estimate,
        or None if not available or too old
        """

    @property
    @ROS.max_delay_ms(messaging.DELAY_SLOW_MS)
    @ROS.subscribe(
        messaging.ROS_TOPIC_TERRAIN_GEOPOINT,
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=lambda msg: _append_geopose_to_path(
            msg, self._ground_track_path_queue
        ),
    )
    def ground_track_geopoint(self) -> Optional[GeoPointStamped]:
        """Vehicle ground track GeoPointStamped, or None if unknown or too old"""

    @property
    @ROS.publish(
        messaging.ROS_TOPIC_PATH,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def path(self) -> Optional[Path]:
        """Vehicle navigation filter :class:`nav_msgs.msg.Path`, or None if
        not available
        """
        raise NotImplementedError

    @property
    @ROS.publish(
        messaging.ROS_TOPIC_PATH_ESTIMATE,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def path_estimate(self) -> Optional[Path]:
        """Vehicle :class:`nav_msgs.msg.Path` estimate, or None if not available"""
        raise NotImplementedError

    @property
    @ROS.publish(
        messaging.ROS_TOPIC_GROUND_TRACK_PATH,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def ground_track_path(self) -> Optional[Path]:
        """Vehicle ground track :class:`nav_msgs.msg.Path`, or None if not
        available
        """
        raise NotImplementedError

    def _publish_path(self):
        """Publishes :class:`nav_msgs.msg.Path` for debugging and visualization"""
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"
        path.poses = list(self._pose_stamped_queue)
        self.__path_publisher.publish(path)

    # TODO: use local or home altitude, not whatever is in the message, as ground
    # track will be visualized separately! Altitude at local frame origin
    # should be zero to make visualization work nicely
    @narrow_types
    def _geopose_to_local_pose(
        self, geopose: GeoPoseStamped, home_position: HomePosition
    ) -> None:
        """
        Converts :class:`geographic_msgs.msg.GeoPoseStamped` into
        :class:`geometry_msgs.msg.PoseStamped` in a local ENU frame where the
        origin is at :class:`.HomePosition` and units in all axes are in meters.

        :param geopose: :class:`geographic_msgs.msg.GeoPoseStamped` message
            to transform into local visualization frame
        :param home_position: Vehicle :class:`.HomePosition` message representing
            local frame origin
        """
        # Convert latitude, longitude, and altitude to Cartesian coordinates
        x, y = self._transformer.transform(
            geopose_stamped.pose.position.latitude,
            geopose_stamped.pose.position.longitude,
        )

        # TODO: use altitude.local or relative (whichever is positive?) even
        # if slightly out of sync
        z = geopose.altitude

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

        assert len(self._pose_stamped_queue) > 0
        self._publish_path()
        self.__pose_stamped_publisher.publish(pose_stamped)

    @staticmethod
    def _append_pose_to_path(pose: PoseStamped, queue: deque) -> None:
        """Appends the pose message to the Path queue

        :param pose: :class:`.PoseStamped` message to append
        :param queue: :class:`nav_msgs.msg.Path` queue to append to
        """
        # Update visualization if some time has passed, but not too soon. This
        # is mainly to prevent the Paths being much shorter in time for nodes
        # that would otherwise publish at much higher frequency (e.g. actual
        # GPS at 10 Hz vs GISNav mock GPS at 1 Hz). Also for visualization
        # a very frequent sample rate is not needed.
        if len(queue) > 0:
            if pose_stamped.header.stamp.sec - self.queue[-1].header.stamp.sec > 1.0:
                self.queue.append(pose_stamped)
            else:
                # Observation is too recent, return
                return None
        else:
            # Queue is empty
            queue.append(pose_stamped)

    @staticmethod
    def _append_geopose_to_path(geopose: GeoPoseStamped, queue: deque) -> None:
        """Appends the geopose message to the Path queue

        :param pose: :class:`.GeoPoseStamped` message to append
        :param queue: :class:`nav_msgs.msg.Path` queue to append to
        """
        pose = self._geopose_to_local_pose(geopose)
        self._append_pose_to_path(pose, queue)
