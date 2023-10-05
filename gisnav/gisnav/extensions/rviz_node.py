"""
This module contains a :term:`ROS 2` node for publishing messages to
:term:`RViz`. The node simplifies data visualization from across different
parts of the :term:`core` system, aiding development and debugging.

The below graph depicts how :class:`.RVizNode` publishes the :term:`vehicle` and
:term:`ground track` :term:`path` that can be susbcribed to and visualized by
:term:`RViz`, making it easier to see where GISNav thinks the vehicle is compared
to where the vehicle :term:`navigation filter` thinks it is:

.. mermaid::
    :caption: RVizNode computational graph

    graph LR
        subgraph GISNode
            vehicle_geopose[gisnav/gis_node/vehicle/geopose]
            ground_track_geopose[gisnav/gis_node/ground_track/geopose]
        end

        subgraph CVNode
            vehicle_estimated_geopose[gisnav/cv_node/vehicle/estimated/geopose]
        end

        subgraph RVizNode
            vehicle_path[gisnav/rviz_node/vehicle/path]
            vehicle_estimated_path[gisnav/rviz_node/vehicle/estimated/path]
            ground_track_path[gisnav/rviz_node/ground_track/path]
        end

        vehicle_geopose -->|geographic_msgs/GeoPose| RVizNode
        vehicle_estimated_geopose -->|geographic_msgs/GeoPose| RVizNode
        ground_track_geopose -->|geographic_msgs/GeoPose| RVizNode
        vehicle_path -->|nav_msgs/Path| rviz2
        vehicle_estimated_path -->|nav_msgs/Path| rviz2
        ground_track_path -->|nav_msgs/Path| rviz2
"""
from collections import deque
from typing import Final, Optional

import numpy as np
from geographic_msgs.msg import GeoPoseStamped
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import HomePosition
from nav_msgs.msg import Path
from pyproj import Transformer
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from .. import messaging
from .._decorators import ROS, narrow_types
from ..static_configuration import (
    CV_NODE_NAME,
    GIS_NODE_NAME,
    BBOX_NODE_NAME,
    ROS_NAMESPACE,
    ROS_TOPIC_RELATIVE_CAMERA_GEOPOSE,
    ROS_TOPIC_RELATIVE_GROUND_TRACK_GEOPOSE,
    ROS_TOPIC_RELATIVE_VEHICLE_ESTIMATED_GEOPOSE,
    ROS_TOPIC_RELATIVE_VEHICLE_GEOPOSE,
)


class RVizNode(Node):
    """:term:`ROS 2` node that subscribes to GISNav :term:`core` output
    messages and publishes them into :term:`RViz`.
    """

    _MAX_POSE_STAMPED_MESSAGES: Final = 100
    """Max limit for held :class:`geometry_msgs.msg.PoseStamped` messages"""

    ROS_TOPIC_RELATIVE_VEHICLE_PATH: Final = "~/vehicle/path"
    """Relative :term:`topic` into which this node publishes
    :attr:`.vehicle_path`
    """

    ROS_TOPIC_RELATIVE_CAMERA_PATH: Final = "~/camera/path"
    """Relative :term:`topic` into which this node publishes
    :attr:`.camera_path`
    """

    ROS_TOPIC_RELATIVE_VEHICLE_ESTIMATED_PATH: Final = "~/vehicle/estimated/path"
    """Relative :term:`topic` into which this node publishes
    :attr:`.vehicle_estimated_path`
    """

    ROS_TOPIC_RELATIVE_GROUND_TRACK_PATH: Final = "~/ground_track/path"
    """Relative :term:`topic` into which this node publishes
    :attr:`.ground_track_path`
    """

    def __init__(self, *args, **kwargs):
        """Class initializer

        :param args: Positional arguments to parent :class:`.Node` constructor
        :param kwargs: Keyword arguments to parent :class:`.Node` constructor
        """
        super().__init__(*args, **kwargs)

        # Initialize ROS subscriptions by calling the decorated properties once
        self.home_position
        self.vehicle_geopose
        self.camera_geopose
        self.vehicle_estimated_geopose
        self.ground_track_geopose

        # Store poses for outgoing path messages in dedicated queues
        self._vehicle_path_queue: deque = deque(maxlen=self._MAX_POSE_STAMPED_MESSAGES)
        self._camera_path_queue: deque = deque(maxlen=self._MAX_POSE_STAMPED_MESSAGES)
        self._vehicle_estimated_path_queue: deque = deque(
            maxlen=self._MAX_POSE_STAMPED_MESSAGES
        )
        self._ground_track_path_queue: deque = deque(
            maxlen=self._MAX_POSE_STAMPED_MESSAGES
        )

        # Transforms latitude and longitude into pseudo-meters
        self._transformer = Transformer.from_crs("EPSG:4326", "EPSG:3857")

    # TODO: use local or home altitude, not whatever is in the message, as ground
    # track will be visualized separately! Altitude at local frame origin
    # should be zero to make visualization work nicely
    @narrow_types
    def _geopose_to_local_pose(
        self, geopose: GeoPoseStamped, home_position: HomePosition
    ) -> Optional[PoseStamped]:
        """
        Converts :class:`geographic_msgs.msg.GeoPoseStamped` into
        :class:`geometry_msgs.msg.PoseStamped` in a local ENU frame where the
        origin is at :class:`.HomePosition` and units in all axes are in meters.

        :param geopose: :class:`geographic_msgs.msg.GeoPoseStamped` message
            to transform into local visualization frame
        :param home_position: Vehicle :class:`.HomePosition` message representing
            local frame origin
        """
        # Use home position as scaling factor for converting EPSG:3857
        # pseudo-meters into actual meters (simple spherical Earth model
        # appropriate for visualization use)
        scale_factor = 1 / np.cos(home_position.geo.latitude)

        # Convert home position latitude, longitude to local frame Cartesian coordinates
        # TODO: cache these, no need to recompute unless home position has changed
        x_home, y_home = (
            scale_factor * coordinate
            for coordinate in self._transformer.transform(
                home_position.geo.latitude,
                home_position.geo.longitude,
            )
        )

        # Convert vehicle (or ground track) latitude, longitude, and altitude
        # (or elevation) to local frame Cartesian coordinates
        x, y = (
            scale_factor * coordinate
            for coordinate in self._transformer.transform(
                geopose.pose.position.latitude,
                geopose.pose.position.longitude,
            )
        )
        # TODO: use altitude.local or relative (whichever is positive?) even
        # if slightly out of sync
        z = geopose.pose.position.altitude

        # Create a PoseStamped message
        pose_stamped = PoseStamped()
        pose_stamped.header = geopose.header
        pose_stamped.header.frame_id = "map"

        # Re-center easting and northing to home position
        pose_stamped.pose.position.x = x - x_home
        pose_stamped.pose.position.y = y - y_home

        # TODO: Re-center z
        # GeoPose is ellipsoid while home position is AMSL altitude/elevation
        pose_stamped.pose.position.z = z
        pose_stamped.pose.orientation = geopose.pose.orientation

        return pose_stamped

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

    def _append_geopose_to_queue(self, geopose: GeoPoseStamped, queue: deque) -> None:
        """Appends the geopose message to the Path queue

        :param pose: :class:`.GeoPoseStamped` message to append
        :param queue: :class:`nav_msgs.msg.Path` queue to append to
        """
        pose = self._geopose_to_local_pose(geopose, self.home_position)
        if pose is not None:
            self._append_pose_to_queue(pose, queue)
        else:
            self.get_logger().warn(
                "Could not append geopose to queue because "
                "could not convert it into a local pose"
            )

    def _append_vehicle_geopose_to_queue(self, geopose: GeoPoseStamped) -> None:
        """Appends the geopose message to the vehicle pose Path queue

        :param pose: :class:`.GeoPoseStamped` message to append
        """
        self._append_geopose_to_queue(geopose, self._vehicle_path_queue)
        self.vehicle_path

    def _append_camera_geopose_to_queue(self, geopose: GeoPoseStamped) -> None:
        """Appends the geopose message to the camera pose Path queue

        :param pose: :class:`.GeoPoseStamped` message to append
        """
        self._append_geopose_to_queue(geopose, self._camera_path_queue)
        self.camera_path

    def _append_vehicle_estimated_geopose_to_queue(
        self, geopose: GeoPoseStamped
    ) -> None:
        """Appends the geopose message to the vehicle pose estimate Path queue

        :param pose: :class:`.GeoPoseStamped` message to append
        """
        self._append_geopose_to_queue(geopose, self._vehicle_estimated_path_queue)
        self.vehicle_estimated_path

    def _append_ground_track_geopose_to_queue(self, geopose: GeoPoseStamped) -> None:
        """Appends the geopose message to the ground track Path queue

        :param pose: :class:`.GeoPoseStamped` message to append
        """
        self._append_geopose_to_queue(geopose, self._ground_track_path_queue)
        self.ground_track_path

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
    @ROS.max_delay_ms(messaging.DELAY_SLOW_MS)
    @ROS.subscribe(
        messaging.ROS_TOPIC_HOME_POSITION,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def home_position(self) -> Optional[HomePosition]:
        """Subscribed :term:`vehicle` :term:`home` position, or None if not
        available or too old
        """

    @property
    @ROS.max_delay_ms(messaging.DELAY_DEFAULT_MS)
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_VEHICLE_GEOPOSE.replace("~", BBOX_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=_append_vehicle_geopose_to_queue,
    )
    def vehicle_geopose(self) -> Optional[GeoPoseStamped]:
        """Subscribed :term:`vehicle` :term:`geopose`, or None if not available
        or too old
        """

    @property
    @ROS.max_delay_ms(messaging.DELAY_DEFAULT_MS)
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_CAMERA_GEOPOSE.replace("~", BBOX_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=_append_camera_geopose_to_queue,
    )
    def camera_geopose(self) -> Optional[GeoPoseStamped]:
        """Subscribed :term:`camera` :term:`geopose`, or None if not available
        or too old
        """

    @property
    @ROS.max_delay_ms(messaging.DELAY_DEFAULT_MS)
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_VEHICLE_ESTIMATED_GEOPOSE.replace("~", CV_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=_append_vehicle_estimated_geopose_to_queue,
    )
    def vehicle_estimated_geopose(self) -> Optional[GeoPoseStamped]:
        """Subscribed :term:`vehicle` :term:`geopose` estimate, or None if not
        available or too old
        """

    @property
    @ROS.max_delay_ms(messaging.DELAY_SLOW_MS)
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_GROUND_TRACK_GEOPOSE.replace("~", GIS_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=_append_ground_track_geopose_to_queue,
    )
    def ground_track_geopose(self) -> Optional[GeoPoseStamped]:
        """Subscribed :term:`vehicle` :term:`ground track` :term:`geopose`, or None
        if not available or too old

        .. note::
            The :term:`orientation` part of the geopose is not defined for the
            ground track and should be ignored.
        """

    @property
    @ROS.publish(
        ROS_TOPIC_RELATIVE_VEHICLE_PATH,
        QoSPresetProfiles.SYSTEM_DEFAULT.value,
    )
    def vehicle_path(self) -> Optional[Path]:
        """Published :term:`vehicle` :term:`global position` :term:`path`, or None
        if not available
        """
        return self._get_path(self._vehicle_path_queue)

    @property
    @ROS.publish(
        ROS_TOPIC_RELATIVE_CAMERA_PATH,
        QoSPresetProfiles.SYSTEM_DEFAULT.value,
    )
    def camera_path(self) -> Optional[Path]:
        """Published :term:`camera` :term:`global position` :term:`path`, or None
        if not available
        """
        return self._get_path(self._camera_path_queue)

    @property
    @ROS.publish(
        ROS_TOPIC_RELATIVE_VEHICLE_ESTIMATED_PATH,
        QoSPresetProfiles.SYSTEM_DEFAULT.value,
    )
    def vehicle_estimated_path(self) -> Optional[Path]:
        """Published :term:`vehicle` :term:`global position` estimate :term:`path`,
        or None if not available
        """
        return self._get_path(self._vehicle_estimated_path_queue)

    @property
    @ROS.publish(
        ROS_TOPIC_RELATIVE_GROUND_TRACK_PATH,
        QoSPresetProfiles.SYSTEM_DEFAULT.value,
    )
    def ground_track_path(self) -> Optional[Path]:
        """Published :term:`vehicle` :term:`ground track` :term:`path`, or None if
        not available

        .. note::
            The :term:`orientation` part of the :term:`geopose` contained in the
            path is not defined for the ground track and should be ignored.
        """
        return self._get_path(self._ground_track_path_queue)
