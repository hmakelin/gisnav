"""Entry points for ROS 2 nodes

All ROS 2 nodes are defined in dedicated modules to keep individual file size
down. They are imported here to package namespace for convenience. For example:

.. code-block::

    #from gisnav.nodes.pose_estimator_node import PoseEstimatorNode
    from gisnav.nodes import PoseEstimatorNode

Node names are hard-coded inside the static public node entrypoints defined
here. Other node initialization arguments are provided via ROS 2 launch
arguments.
"""
import cProfile
import io
import pstats

import rclpy

from .bbox_node import BBoxNode
from .map_node import MapNode
from .mock_gps_node import MockGPSNode
from .pose_estimation_node import PoseEstimationNode


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

    node = None
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
            stats = pstats.Stats(profile, stream=s).sort_stats(
                pstats.SortKey.CUMULATIVE
            )
            stats.print_stats(40)
            print(s.getvalue())
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


def run_mock_gps_node():
    """Spins up a :class:`.MockGPSNode`"""
    _run(MockGPSNode, "mock_gps_node")


def run_bbox_node():
    """Spins up a :class:`.BBoxNode`"""
    _run(BBoxNode, "bbox_node")


def run_map_node():
    """Spins up a :class:`.MapNode`"""
    _run(MapNode, "map_node")


def run_pose_estimation_node():
    """Spins up a :class:`.PoseEstimationNode`"""
    _run(PoseEstimationNode, "pose_estimation_node")
