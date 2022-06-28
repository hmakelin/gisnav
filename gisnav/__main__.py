import io
import cProfile
import pstats
import rclpy

from ament_index_python.packages import get_package_share_directory

from gisnav.nodes.mock_gps_node import MockGPSNode


def main(args=None):
    """Starts and terminates the ROS 2 node.

    Also starts cProfile profiling in debugging mode.

    :param args: Any args for initializing the rclpy node
    :return:
    """
    package_name = __name__.rsplit('.')[-2]  # should match package.xml declared package_name
    if __debug__:
        pr = cProfile.Profile()
        pr.enable()
    else:
        pr = None
    try:
        rclpy.init(args=args)
        mock_gps_node = MockGPSNode('mock_gps_node', get_package_share_directory(package_name))
        rclpy.spin(mock_gps_node)
    except KeyboardInterrupt as e:
        print(f'Keyboard interrupt received:\n{e}')
        if pr is not None:
            # Print out profiling stats
            pr.disable()
            s = io.StringIO()
            ps = pstats.Stats(pr, stream=s).sort_stats(pstats.SortKey.CUMULATIVE)
            ps.sort_stats('cumulative').print_stats(20)
            #print(s.getvalue())
    finally:
        mock_gps_node.destroy_timers()
        mock_gps_node.terminate_pools()
        mock_gps_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
