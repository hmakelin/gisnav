"""This module contains the static configuration of the :term:`ROS` namespace
and node and topic names

Using this module nodes that talk to each other can refer to a single source
of truth

.. warning::
    This module should not import anything from the gisnav package namespace
    to prevent circular imports.
"""
from typing import Final, Literal

ROS_NAMESPACE: Final = "gisnav"
""":term:`ROS` node namespace"""

GIS_NODE_NAME: Final = "gis_node"
"""Name of :class:`.GISNode` spun up by :func:`.run_gis_node`"""

BBOX_NODE_NAME: Final = "bbox_node"
"""Name of :class:`.BBoxNode` spun up by :func:`.run_bbox_node`"""

POSE_NODE_NAME: Final = "pose_node"
"""Name of :class:`.PoseNode` from gisnav_gpu package."""

STEREO_NODE_NAME: Final = "stereo_node"
"""Name of :class:`.StereoNode` spun up by :func:`.run_transform_node`"""

NMEA_NODE_NAME: Final = "nmea_node"
"""Name of :class:`.NMEANode` spun up by :func:`.run_nmea_node`"""

UORB_NODE_NAME: Final = "uorb_node"
"""Name of :class:`.UORBNode` spun up by :func:`.run_uorb_node`"""

QGIS_NODE_NAME: Final = "qgis_node"
"""Name of :class:`.QGISNode` spun up by :func:`.run_qgis_node`"""

ROS_TOPIC_RELATIVE_ORTHOIMAGE: Final = "~/orthoimage"
"""Relative :term:`topic` into which :class:`.GISNode` publishes
:attr:`.GISNode.orthoimage`.
"""

ROS_TOPIC_SENSOR_GPS: Final = "/fmu/in/sensor_gps"
""":term:`Topic` into which :class:`.UORBNode` publishes
:attr:`.GISNode.sensor_gps`.
"""

ROS_TOPIC_RELATIVE_FOV_BOUNDING_BOX: Final = "~/fov/bounding_box"
"""Relative :term:`topic` into which :class:`.BBoxNode` publishes
:attr:`.BBoxNode.fov_bounding_box`.
"""

ROS_TOPIC_RELATIVE_POSE_IMAGE: Final = "~/pose_image"
"""Relative :term:`topic` into which :class:`.StereoNode` publishes
:attr:`.StereoNode.pose_image`.
"""

ROS_TOPIC_RELATIVE_TWIST_IMAGE: Final = "~/twist_image"
"""Relative :term:`topic` into which :class:`.StereoNode` publishes
:attr:`.StereoNode.twist_image`.
"""

ROS_TOPIC_RELATIVE_POSE: Final = "~/pose"
"""Relative :term:`topic` into which :class:`.StereoNode` publishes
:attr:`.StereoNode.pose`.

"""
ROS_TOPIC_RELATIVE_QUERY_TWIST: Final = "~/vo/twist"
"""Relative :term:`topic` into which :class:`.StereoNode` publishes
:attr:`.StereoNode.camera_optical_twist_in_query_frame`.
"""

MAVROS_TOPIC_TIME_REFERENCE: Final = "/mavros/time_reference"
"""The :term:`MAVROS` time reference topic that has the difference between
the local system time and the foreign :term:`FCU` time
"""

ROS_TOPIC_CAMERA_INFO: Final = "/camera/camera_info"
"""Name of ROS topic for :class:`sensor_msgs.msg.CameraInfo` messages"""

ROS_TOPIC_IMAGE: Final = "/camera/image_raw"
"""Name of ROS topic for :class:`sensor_msgs.msg.Image` messages"""

ROS_TOPIC_MAVROS_GLOBAL_POSITION = "/mavros/global_position/global"
"""MAVROS topic for vehicle :class:`.NavSatFix`"""

ROS_TOPIC_MAVROS_LOCAL_POSITION = "/mavros/local_position/pose"
"""MAVROS topic for vehicle :class:`.PoseStamped` in EKF local frame"""

ROS_TOPIC_MAVROS_GIMBAL_DEVICE_ATTITUDE_STATUS = (
    "/mavros/gimbal_control/device/attitude_status"
)
"""MAVROS topic for vehicle :class:`.GimbalDeviceAttitudeStatus` message
(MAVLink Gimbal protocl v2)
"""

DELAY_DEFAULT_MS: Final = 2000
"""Max acceptable delay for things like global position"""

FrameID = Literal[
    "base_link",
    "camera",
    "camera_optical",
    "map",
    "earth",
]
"""Allowed ROS message header ``frame_id``s as specified in :term:`REP 103` and
:term:`REP 105`. The ``odom`` frame is not used by GISNav but may be published e.g. by
:term:`MAVROS`.
"""
