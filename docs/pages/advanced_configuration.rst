.. toctree::
   :maxdepth: 2

Advanced Configuration
--------------------------------------------
The MapNavNode Base Class
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The MapNavNode is the base node that subscribes to the necessary PX4/ROS 2 bridge topics and estimates and outputs the
position estimate. As a quick overview, these are the topics the nodes subscribes to:

TODO

#. topic 1
#. topic 2
#. topic 3

The base class does not publish anything. How the node is integrates and publishes its output is up to you to decide.
See `Custom Node`_ for instructions on integrating GISNav.

.. _Custom Node:
Custom Node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To integrate GISNav with your solution, you will need to implement the :class:`python_px4_ros2_map_nav.nodes.map_nav_node.MapNavNode` class by writing a :meth:`python_px4_ros2_map_nav.nodes.map_nav_node.MapNavNode.publish` method::

    from python_px4_ros2_map_nav.nodes import MapNavNode
    from python_px4_ros2_map_nav.data import OutputData

    class MyCustomNode(MapNavNode):

        # You can override the __init__ method and do whatever you need here
        ...

        def publish(output_data):
            """Prints the output into console"""
            print(f'Here is the output: {output_data}')


:class:`python_px4_ros2_map_nav.data.OutputData` for what fields are contained in the output data container.

You can see a longer example in source code for the :class:`python_px4_ros2_map_nav.nodes.map_nav_node.MockGPSNode`
class, which creates a :class:`px4_msgs.VehicleGpsPosition` mock GPS (GNSS) message out of the output and publishes
it to the flight control software via the appropriate PX4/ROS 2 bridge topic.


.. _Custom Pose Estimator:
Custom Pose Estimator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Two pose estimators, SuperGlue and SuperGlue derivative LoFTR are provided with LoFTR as the default pose estimator.
These were seen as state-of-the-art image matching algorithms at the time the software was written but newer algorithms
may provide more reliable matching. Note that SuperGlue has restrictive licensing requirements if you are planning to
use it for your own project (see license file in the repository).

You can write your own pytorch based pose estimator by implementing the
:class:`python_px4_ros2_map_nav.matchers.Matcher` interface. If your algorithm is keypoint-based, you may
also use the :class:`python_px4_ros2_map_nav.matchers.KeyPointMatcher` abstract base class, which provides a way to
compute the pose estimate from matched keypoints.

The pose estimator runs in a dedicated process, so you need to implement the static initializer and worker methods,
for example::

    from python_px4_ros2_map_nav.matchers.keypoint_matcher import Matcher

    class MyCustomMatcher(Matcher):

        ...

        def initializer(class_name, guess):
            """Initializes the global matcher variable"""
            global my_custom_matcher
            my_custom_matcher = class_name(*args)

        def worker(image_pair, guess):
            """Estimates pose between image pair"""
            return my_custom_match._match(image_pair)

        def _match(image_pair):
            """Custom matching function"""
            # Do your pose estimation magic here


The class name and arguments passed to the initializer are defined in the YAML config files.