"""This is a ROS 2 package that determines UAV global position by aligning real-time
video with maps from an onboard GIS server.

All ROS 2 nodes are defined in dedicated modules to keep individual file size
down.

The ROS namespace and node names are hard-coded in :mod:`.constants`, and the
static node entrypoints are defined here in the package root namespace.
"""
import cProfile
import io
import pstats
from typing import Optional

import rclpy
from rclpy.node import Node

from .constants import (
    BBOX_NODE_NAME,
    GIS_NODE_NAME,
    NMEA_NODE_NAME,
    POSE_NODE_NAME,
    ROS_NAMESPACE,
    STEREO_NODE_NAME,
    UORB_NODE_NAME,
    WFST_NODE_NAME,
)
from .core import BBoxNode, GISNode, PoseNode, StereoNode

try:
    from .extensions.wfst_node import WFSTNode

    def run_wfst_node():
        """Spins up a :class:`.WFSTNode`

        > [!NOTE] Must install extra
        > Not available if the ``wfst_node`` Python extra has not been installed.

        .. code-block:: bash
            :caption: Install WFSTNode

            cd ~/colcon_ws/src/gisnav/gisnav
            pip install .[wfst_node]
        """
        _run(WFSTNode, WFST_NODE_NAME, **_rclpy_node_kwargs)

except ModuleNotFoundError as e:
    print(f"Could not import WFSTNode because a module was not found: {e}")

try:
    from .extensions.nmea_node import NMEANode

    def run_nmea_node():
        """Spins up a :class:`.NMEANode`

        > [!NOTE] Must install extra
        > Not available if the ``nmea_node`` Python extra has not been installed.

        .. code-block:: bash
            :caption: Install NMEANode

            cd ~/colcon_ws/src/gisnav/gisnav
            pip install .[nmea_node]
        """
        _run(NMEANode, NMEA_NODE_NAME, **_rclpy_node_kwargs)

except ModuleNotFoundError as e:
    print(f"Could not import NMEANode because a module was not found: {e}")

try:
    from .extensions.uorb_node import UORBNode

    def run_uorb_node():
        """Spins up a :class:`.UORBNode`

        > [!NOTE] Must install extra
        > Not available if the ``uorb_node`` Python extra has not been installed.

        .. code-block:: bash
            :caption: Install UORBNode

            cd ~/colcon_ws/src/gisnav/gisnav
            pip install .[uorb_node]
        """
        _run(UORBNode, UORB_NODE_NAME, **_rclpy_node_kwargs)

except ModuleNotFoundError as e:
    print(f"Could not import NMEANode because a module was not found: {e}")


def _run(constructor: rclpy.node.Node, *args, **kwargs):
    """Spins up a ROS 2 node

    :param constructor: Node constructor
    :param *args: Node constructor args
    :param **kwargs: Node constructor kwargs
    :return:
    """
    if __debug__:
        profile = cProfile.Profile()
        profile.enable()
    else:
        profile = None

    node: Optional[Node] = None
    try:
        rclpy.init()
        node = constructor(*args, **kwargs)
        rclpy.spin(node)
    except KeyboardInterrupt as e:
        print(f"Keyboard interrupt received:\n{e}")
        if profile is not None:
            assert __debug__
            # Print out cProfile stats
            profile.disable()
            s = io.StringIO()
            stats = pstats.Stats(profile, stream=s).sort_stats(pstats.SortKey.TIME)
            stats.print_stats(20)
            if node is not None:
                node.get_logger().info(s.getvalue())
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


_rclpy_node_kwargs = {
    "namespace": ROS_NAMESPACE,
    "allow_undeclared_parameters": True,
    "automatically_declare_parameters_from_overrides": True,
}


def run_bbox_node():
    """Spins up a :class:`.BBoxNode`"""
    _run(BBoxNode, BBOX_NODE_NAME, **_rclpy_node_kwargs)


def run_gis_node():
    """Spins up a :class:`.GISNode`"""
    _run(GISNode, GIS_NODE_NAME, **_rclpy_node_kwargs)


def run_stereo_node():
    """Spins up a :class:`.StereoNode`"""
    _run(StereoNode, STEREO_NODE_NAME, **_rclpy_node_kwargs)


def run_pose_node():
    """Spins up a :class:`.PoseNode`"""
    _run(PoseNode, POSE_NODE_NAME, **_rclpy_node_kwargs)
