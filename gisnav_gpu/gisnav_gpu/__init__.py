import cProfile
import io
import pstats
from typing import Optional

import rclpy
from pose_node import PoseNode
from rclpy.node import Node


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
    "namespace": "gisnav",
    "allow_undeclared_parameters": True,
    "automatically_declare_parameters_from_overrides": True,
}


def run_pose_node():
    """Spins up a :class:`.BBoxNode`"""
    _run(PoseNode, "pose_node", _rclpy_node_kwargs)
