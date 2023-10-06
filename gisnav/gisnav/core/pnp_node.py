"""This module contains the :class:`.PnPNode` :term:`ROS` node that solves
the :term:`PnP` problem to produce a :term:`camera` :term:`pose` from matched
keypoints.
"""
from typing import Optional

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
    BBOX_NODE_NAME,
    CV_NODE_NAME,
    GIS_NODE_NAME,
    ROS_NAMESPACE,
    ROS_TOPIC_RELATIVE_CAMERA_GEOPOSE,
    ROS_TOPIC_RELATIVE_GROUND_TRACK_GEOPOSE,
    ROS_TOPIC_RELATIVE_VEHICLE_ESTIMATED_GEOPOSE,
    ROS_TOPIC_RELATIVE_VEHICLE_GEOPOSE,
)


class PnPNode(Node):
    """A :term:`ROS` node that solves the :term:`PnP` problem to produce
    a :term:`camera` :term:`pose` from matched keypoints.
    """

    def __init__(self, *args, **kwargs):
        """Class initializer

        :param args: Positional arguments to parent :class:`.Node` constructor
        :param kwargs: Keyword arguments to parent :class:`.Node` constructor
        """
        super().__init__(*args, **kwargs)

        # Initialize ROS subscriptions by calling the decorated properties once

    @property
    @ROS.max_delay_ms(messaging.DELAY_DEFAULT_MS)
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_CAMERA_ESTIMATED_POSE.replace("~", TORCH_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=_append_vehicle_estimated_geopose_to_queue,
    )
    def camera_estimated_pose(self) -> Optional[GeoPoseStamped]:
        """Subscribed :term:`camera` relative :term:`pose` estimate, or None if
        not available or too old
        """
