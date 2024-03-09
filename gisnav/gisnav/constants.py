"""This module contains the static configuration of the :term:`ROS` namespace
and node and topic names

Using this module nodes that talk to each other can refer to a single source
of truth

.. note::
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

ROS_TOPIC_SENSOR_GPS: Final = "/fmu/in/sensor_gps"
"""Name of ROS topic for outgoing :class:`px4_msgs.msg.SensorGps` messages
over PX4 DDS bridge"""

ROS_TOPIC_CAMERA_INFO: Final = "/camera/camera_info"
"""Name of ROS topic for :class:`sensor_msgs.msg.CameraInfo` messages"""

ROS_TOPIC_IMAGE: Final = "/camera/image_raw"
"""Name of ROS topic for :class:`sensor_msgs.msg.Image` messages"""

DELAY_DEFAULT_MS: Final = 2000
"""Max acceptable delay for things like global position"""

DELAY_FAST_MS: Final = 500
"""Max acceptable delay for messages with fast dynamics that go "stale"
quickly, e.g.local position and attitude. The delay can be a bit higher
than is intuitive because the vehicle EKF should be able to fuse things
with fast dynamics with higher lags as long as the timestamps are accurate.
"""

FrameID = Literal[
    "reference",
    "reference_%i_%i",
    "base_link",
    "gimbal",
    "camera",
    "camera_pinhole",
    "map",
    "world",
]
"""Allowed ROS header frame_ids (used by :term:`tf2`)

The ``camera_pinhole`` and ``world`` frames are coordinate systems in the cv2
pinhole camera model. The ``camera`` frame follows the convention where the x
axis points to the right from the body of the camera and z-axis forward along
the optical axis, it is not the ``camera_optical`` frame where the x axis
points in the direction of the optical axis.

The ``base_link`` frame is defined as the vehicle body :term:`FRD` frame.

The ``reference`` frame is the :term:`reference` arrays coordinate frame
where the origin is the bottom left (ROS convention, not numpy/cv2 top left
convention). x axis is the width axis, y axis is height. The static
``reference_%i_%i`` frame (in relation to :term:`WGS 84` coordinates)
is intended to be suffixed with the :term:`ROS` second and nanosecond
integer timestamps to allow the tf2 transformation chain to be matched
to the exact same reference frame that was used to for deriving the
:term:`orthoimage`. This is needed because the reference frame is
discontinuous (jumps around) and tf2 default interpolation cannot therefore
be applied to it.
"""
