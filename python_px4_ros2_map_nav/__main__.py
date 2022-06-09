import io
import cProfile
import pstats
import rclpy

from ament_index_python.packages import get_package_share_directory

from python_px4_ros2_map_nav.nodes.mock_gps_node import MockGPSNode


def main(args=None):
    """Starts and terminates the ROS 2 node.

    Also starts cProfile profiling in debugging mode.

    :param args: Any args for initializing the rclpy node
    :return:
    """
    package_name = __name__.rsplit('.')[-2]  # should match package.xml declared package_name
    #if __debug__:
    #    pr = cProfile.Profile()  # TODO: re-enable
    #    pr.enable()
    #else:
    pr = None
    try:
        rclpy.init(args=args)
        matcher = MockGPSNode('map_nav_node', get_package_share_directory(package_name))
        rclpy.spin(matcher)
    except KeyboardInterrupt as e:
        print(f'Keyboard interrupt received:\n{e}')
        if pr is not None:
            # Print out profiling stats
            pr.disable()
            s = io.StringIO()
            ps = pstats.Stats(pr, stream=s).sort_stats(pstats.SortKey.CUMULATIVE)
            ps.print_stats()
            print(s.getvalue())
    finally:
        matcher.destroy_timers()
        matcher.terminate_wms_pool()
        matcher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
