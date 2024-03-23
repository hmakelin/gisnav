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

STEREO_NODE_NAME: Final = "stereo_node"
"""Name of :class:`.StereoNode` spun up by :func:`.run_transform_node`"""

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
"""Relative :term:`topic` into which :class:`.StereoNode` publishes
:attr:`.StereoNode.pnp_image`.
"""

ROS_TOPIC_RELATIVE_CAMERA_ESTIMATED_POSE: Final = "~/camera_optical/pose"
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
    "query",
    "query_previous",
    "world",
    "base_link",
    "camera",
    "camera_optical",
    "map",
]
"""Allowed ROS header frame_ids.

:term:`REP 103` and :term:`REP 105` defined frames:

* ``camera``
* ``camera_optical``
* ``map``
* ``base_link``
* ``odom`` and ``earth`` not used currently, but could be introduced in the future.

:term:`OpenCV` ``cv2.solvePnP`` defined frames (not REP 103 compliant).

* ?

Introduced by GISNav (**currently** not REP 103 compliant):

* ``reference``
* ``reference_%i_%i``
* ``query``
* ``query_%i_%i``

The ``reference`` frame is the REP 103 compliant version of ``reference_world`` where
the origin is in the bottom-left corner instead of top-left, making it an :term:`ENU`
frame.

``reference_%i_%i`` frame is a timestamped version of the ``reference`` frame
where the first integer is the seconds timestamp and the second integer is the
nanoseconds timestamp of the originating sensor_msgs/Image message containing
 the orthoimage :term:`raster` from :term:`GISNode`. Timestamping is needed
 because the ``reference`` frame is discontinuous, breaking `tf2` interpolation and
 necessitating using timestamps to match poses to correct frames instead.

 .. todo::
    Scale ``reference`` and ``reference_%i_%i`` to SI units (meters) instead of
    the native pixels to be fully compliant with REP 103
"""
