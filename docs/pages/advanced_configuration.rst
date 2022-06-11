Advanced Configuration
--------------------------------------------
The BaseNode Class
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The :class:`.MapNavNode` abstract base class implements a ROS 2 node that produces a vehicle position estimate from
visual inputs without the need for a GNSS (GPS) signal.

PX4-ROS 2 Bridge Topics
"""""""""""""""""""""""""""""""""""""""""""
The node main process subscribes to the telemetry received via the PX4-ROS 2 bridge and defines a callback function for
each topic to handle the received messages on the main thread.

The :class:`.MapNavNode` subscribes to the following telemetry:

    #. :class:`px4_msgs.VehicleGlobalPosition` messages via 'VehicleGlobalPosition_PubSubTopic'
    #. :class:`px4_msgs.VehicleLocalPosition` messages via 'VehicleLocalPosition_PubSubTopic'
    #. :class:`px4_msgs.VehicleAttitude` messages via 'VehicleAttitude_PubSubTopic'
    #. :class:`px4_msgs.Image` messages via 'image_raw'
    #. :class:`px4_msgs.CameraInfo` messages via 'camera_info'

WMS Client
"""""""""""""""""""""""""""""""""""""""""""
The :class:`.MapNavNode` Map rasters from WMS endpoint, requested by embedded :class:`.WMSClient` instance

The :class:`.WMSClient` on the other hand is instantiated
in a dedicated process. A :py:attr:`._wms_timer` periodically requests the :class:`.WMSClient` to fetch a new map based
on criteria defined in :meth:`._should_update_map`. Generally a new map is requested if the field of view (FOV) of the
vehicle's camera no longer significantly overlaps with the previously requested map.

Publish Method and Output
"""""""""""""""""""""""""""""""""""""""""""
The :class:`.MapNavNode` base class defines a :meth:`.publish` abstract method and leaves it to the implementing class
to decide what to do with the computed output. The data provided to the method is defined in :class:`.OutputData`.

.. _Custom Node:
Custom Node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To integrate GISNav with your solution, you will need to implement the :class:`.MapNavNode` class by writing a :meth:`.publish` method::

    from python_px4_ros2_map_nav.nodes import MapNavNode
    from python_px4_ros2_map_nav.data import OutputData

    class MyCustomNode(MapNavNode):

        # You can override the __init__ method and do whatever you need here
        ...

        def publish(output_data):
            """Prints the output into console"""
            print(f'Here is the output: {output_data}')


:class:`.OutputData` for what fields are contained in the output data container.

You can see a longer example in source code for the :class:`.MockGPSNode`
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
:class:`.Matcher` interface. If your algorithm is keypoint-based, you may
also use the :class:`.KeyPointMatcher` abstract base class, which provides a way to
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