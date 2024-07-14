"""This module contains the static configuration of the ROS namespace and node and
topic names

Using this module, nodes that talk to each other can refer to a single source of truth.

> [!WARNING] Circular imports
> This module should not import anything from the gisnav package namespace to prevent
> circular imports.
"""
from typing import Final, Literal

ROS_NAMESPACE: Final = "gisnav"
"""Namespace for all GISNav ROS nodes"""

GIS_NODE_NAME: Final = "gis_node"
"""Name of :class:`.GISNode` spun up by :func:`.run_gis_node`"""

BBOX_NODE_NAME: Final = "bbox_node"
"""Name of :class:`.BBoxNode` spun up by :func:`.run_bbox_node`"""

POSE_NODE_NAME: Final = "pose_node"
"""Name of :class:`.PoseNode` spun up by :func:`.run_pose_node`."""

TWIST_NODE_NAME: Final = "twist_node"
"""Name of :class:`.TwistNode` spun up by :func:`.run_twist_node`."""

STEREO_NODE_NAME: Final = "stereo_node"
"""Name of :class:`.StereoNode` spun up by :func:`.run_stereo_node`"""

NMEA_NODE_NAME: Final = "nmea_node"
"""Name of :class:`.NMEANode` spun up by :func:`.run_nmea_node`"""

UORB_NODE_NAME: Final = "uorb_node"
"""Name of :class:`.UORBNode` spun up by :func:`.run_uorb_node`"""

WFST_NODE_NAME: Final = "wfst_node"
"""Name of :class:`.WFSTNode` spun up by :func:`.run_wfst_node`"""

ROS_TOPIC_RELATIVE_ORTHOIMAGE: Final = "~/orthoimage"
"""Relative topic into which :class:`.GISNode` publishes :attr:`.GISNode.orthoimage`."""

ROS_TOPIC_SENSOR_GPS: Final = "/fmu/in/sensor_gps"
"""Topic into which :class:`.UORBNode` publishes :attr:`.UORBNode.sensor_gps`."""

ROS_TOPIC_RELATIVE_FOV_BOUNDING_BOX: Final = "~/fov/bounding_box"
"""Relative topic into which :class:`.BBoxNode` publishes
:attr:`.BBoxNode.fov_bounding_box`.
"""

ROS_TOPIC_RELATIVE_POSE_IMAGE: Final = "~/pose_image"
"""Relative topic into which :class:`.StereoNode` publishes
:attr:`.StereoNode.pose_image`.
"""

ROS_TOPIC_RELATIVE_POSE: Final = "~/pose"
"""Relative topic into which :class:`.PoseNode` publishes
:attr:`.PoseNode.pose`.
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
(MAVLink Gimbal protocol v2)
"""

ROS_TOPIC_ROBOT_LOCALIZATION_ODOMETRY = "/robot_localization/odometry/filtered"
"""Topic for filtered odometry from the ``robot_localization`` package EKF node"""

ROS_TOPIC_RELATIVE_MATCHES_IMAGE = "~/dev/matches_image"
"""Relative topic into which :class:`.PoseNode` publishes the keypoint match image."""

ROS_TOPIC_RELATIVE_POSITION_IMAGE = "~/dev/position_image"
"""Relative topic into which :class:`.PoseNode` publishes the camera position image."""

DELAY_DEFAULT_MS: Final = 2000
"""Max acceptable delay for things like global position"""

FrameID = Literal[
    "base_link",
    "camera",
    "camera_optical",
    "base_link_stabilized",
    "camera_frd",
    "map",
    "odom",
    "earth",
    "gisnav_map",
    "gisnav_odom",
    "gisnav_camera_link_optical",
    "gisnav_base_link",
]
"""Allowed ROS message header ``frame_id`` as specified in REP 103 and
REP 105. The ``odom`` frame is not used by GISNav but may be published e.g. by
MAVROS.
"""
