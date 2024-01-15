"""This module contains the static configuration of the :term:`ROS` namespace
and node and topic names

Using this module nodes that talk to each other can refer to a single source of truth
"""
from typing import Final

ROS_NAMESPACE: Final = "gisnav"
""":term:`ROS` node namespace"""

GIS_NODE_NAME: Final = "gis_node"
"""Name of :class:`.GISNode` spun up by :func:`.run_gis_node`"""

BBOX_NODE_NAME: Final = "bbox_node"
"""Name of :class:`.BBoxNode` spun up by :func:`.run_bbox_node`"""

POSE_NODE_NAME: Final = "pose_node"
"""Name of :class:`.PoseNode` from gisnav_gpu package."""

TRANSFORM_NODE_NAME: Final = "transform_node"
"""Name of :class:`.TransformNode` spun up by :func:`.run_transform_node`"""

MOCK_GPS_NODE_NAME: Final = "mock_gps_node"
"""Name of :class:`.MockGPSNode` spun up by :func:`.run_mock_gps_node`"""

RVIZ_NODE_NAME: Final = "rviz_node"
"""Name of :class:`.RVizNode` spun up by :func:`.run_rviz_node`"""

QGIS_NODE_NAME: Final = "qgis_node"
"""Name of :class:`.QGISNode` spun up by :func:`.run_qgis_node`"""

ROS_TOPIC_RELATIVE_ORTHOIMAGE: Final = "~/orthoimage"
"""Relative :term:`topic` into which :class:`.GISNode` publishes
:attr:`.GISNode.orthoimage`.
"""

ROS_TOPIC_RELATIVE_GEOTRANSFORM: Final = "~/geotransform"
"""Relative :term:`topic` into which :class:`.GISNode` publishes
:attr:`.GISNode.geotransform`.
"""

ROS_TOPIC_RELATIVE_FOV_BOUNDING_BOX: Final = "~/fov/bounding_box"
"""Relative :term:`topic` into which :class:`.BBoxNode` publishes
:attr:`.BBoxNode.fov_bounding_box`.
"""

ROS_TOPIC_RELATIVE_PNP_IMAGE: Final = "~/image"
"""Relative :term:`topic` into which :class:`.TransformNode` publishes
:attr:`.CVNode.pnp_image`.
"""

ROS_TOPIC_RELATIVE_CAMERA_ESTIMATED_POSE: Final = "~/camera/estimated/pose"
"""Relative :term:`topic` into which :class:`.PnPNode` publishes
:attr:`.PnPNode.camera_estimated_pose`.
"""

MAVROS_TOPIC_TIME_REFERENCE: Final = "/mavros/time_reference"
"""The :term:`MAVROS` time reference topic that has the difference between 
the local system time and the foreign :term:`FCU` time
"""