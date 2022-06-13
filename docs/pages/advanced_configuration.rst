Advanced Configuration
===================================================
This section provides detailed information and code samples that you can use to try out or integrate GISNav with
your own project.

First, you might be interested in implementing your own `Custom Node`_, or if you are happy with the
provided example nodes, you may also look into making your own `Custom Pose Estimator`_.

ROS Nodes
---------------------------------------------------
The ``ROS 2`` nodes can be found in the :py:mod:`python_px4_ros2_map_nav.nodes` package. Unless you want to use example
:class:`.MockGPSNode`, you will have to implement your own `Custom Node`_.

.. _The BaseNode class:

The BaseNode class
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The :class:`.MapNavNode` abstract base class implements a ROS 2 node that produces a vehicle position estimate from
visual inputs without the need for a GNSS (GPS) signal.

.. _The MockGPSNode class:

The MockGPSNode class
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The :class:`.MockGPSNode` extends the :class:`.MapNavNode` abstract base class to publish a mock GPS message generated
from the output. It is used in the demo as an example of how GISNav can complement and in some cases replace GNSS
navigation.

In order for the :class:`.MockGPSNode` to work, you would need to configure your ``typhoon_h480`` build target to use
the new GPS. This can be either configured before flight in the file ``TODO``, or during flight by setting the
`SENS_GPS_PRIME <https://docs.px4.io/v1.12/en/advanced_config/parameter_reference.html#SENS_GPS_PRIME>`_ parameter with
the `param <https://docs.px4.io/v1.12/en/advanced_config/parameter_reference.html#SENS_GPS_PRIME>`_ command::

    param set SENS_GPS_PRIME 1

You may also want to try setting the following limits to be more tolerant::

    TODO


.. _Custom Node:

Custom Node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
To integrate GISNav with your solution, you must implement the :class:`.MapNavNode` class by writing a :meth:`.publish` method::

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


The Publish Method
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The :class:`.MapNavNode` base class defines a :meth:`.publish` abstract method and leaves it to the implementing class
to decide what to do with the computed output. The data provided to the method is defined in :class:`.OutputData`.


PX4-ROS 2 Bridge Topics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The node main process subscribes to the telemetry received via the PX4-ROS 2 bridge and defines a callback function for
each topic to handle the received messages on the main thread.

The :class:`.MapNavNode` subscribes to the following telemetry:

    #. :class:`px4_msgs.VehicleGlobalPosition` messages via 'VehicleGlobalPosition_PubSubTopic'
    #. :class:`px4_msgs.VehicleLocalPosition` messages via 'VehicleLocalPosition_PubSubTopic'
    #. :class:`px4_msgs.VehicleAttitude` messages via 'VehicleAttitude_PubSubTopic'
    #. :class:`px4_msgs.Image` messages via 'image_raw'
    #. :class:`px4_msgs.CameraInfo` messages via 'camera_info'

You may add more subscribe and publish topics if you decide to implement your own Node. You will need to edit the
``uorb_rtps_message_ids.yaml`` file as described in the
`microRTPS section of the PX4 User Guide <https://docs.px4.io/v1.12/en/middleware/micrortps.html>`_ to ensure your
messages are passed between PX4 and your ROS node.

WMS Client
---------------------------------------------------
The :class:`.MapNavNode` Map rasters from WMS endpoint, requested by embedded :class:`.WMSClient` instance

The :class:`.WMSClient` on the other hand is instantiated
in a dedicated process. A :py:attr:`._wms_timer` periodically requests the :class:`.WMSClient` to fetch a new map based
on criteria defined in :meth:`._should_update_map`. Generally a new map is requested if the field of view (FOV) of the
vehicle's camera no longer significantly overlaps with the previously requested map.

.. _Pose Estimators:

Pose Estimators
---------------------------------------------------
Two pose estimators, SuperGlue and SuperGlue derivative LoFTR are provided with LoFTR as the default pose estimator.
These were seen as state-of-the-art image matching algorithms at the time the software was written but newer algorithms
may provide more reliable matching. Note that SuperGlue has restrictive licensing requirements if you are planning to
use it for your own project (see license file in the repository).

You must extend the :class:`.PoseEstimator` abstract base and write your own :meth:`.estimate_pose` method to implement
your own pose estimator. If your pose estimator is keypoint-based, you may want to extend
:class:`.KeypointPoseEstimator` and implement the :meth:`.find_matching_keypoints` method instead. The base classes
implement the required static initializer and worker methods that are required to make them work with multithreading
and multiprocessing.

.. _Configuration:

Configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
You would then need to create a configuration file ``config/my_custom_pose_estimator.yml`` that tells GISNav
how to initialize your new pose estimator. The configuraiton file will inclue the full path and initialization
arguments::

    class_name: 'python_px4_ros2_map_nav.pose_estimators.my_pose_estimator.MyPoseEstimator'
    args:
      - 15  # _min_matches


.. _Custom Pose Estimator:

Custom Pose Estimator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
You can use the below snippets to get started with your own :class:`.PoseEstimator`::

    from typing import Optional
    from python_px4_ros2_map_nav.pose_estimators.pose_estimator import PoseEstimator
    from python_px4_ros2_map_nav.data import ImagePair, Pose

    class MyPoseEstimator(PoseEstimator):

        def __init__(self, ):
            # TODO

        def estimate_pose(image_pair: ImagePair, guess: Optional[Pose]) -> Optional[Pose]:
            """Custom pose estimation"""
            # Do your pose estimation magic here
            return Pose(r, t)

.. _Custom Keypoint-Based Pose Estimator:

Custom Keypoint-Based Pose Estimator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
If you want to create a :class:`.KeypointPoseEstimator`, you can also start with the below snippet::

    from typing import Optional
    from python_px4_ros2_map_nav.pose_estimators.keypoint_pose_estimator import KeypointPoseEstimator
    from python_px4_ros2_map_nav.data import ImagePair, Pose

    class MyPoseEstimator(KeypointPoseEstimator):

        def __init__(self, ):
            # TODO

        def find_matching_keypoints(image_pair: ImagePair) -> Optional[KeypointPoseEstimator.MatchingKeypoints]:
            """Custom keypoint matching"""
            # Find matching keypoints here

            matching_keypoints = KeypointPoseEstimator.MatchingKeypoints(
                query_keypoints =
                reference_keypoints =
            )
            return matching_keypoints


.. _Kalman Filter:

Kalman Filter
---------------------------------------------------
The SimpleFilter class
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
TODO: Filter abstract base class or interface

An embedded :class:`.SimpleFilter` Kalman filter is included to (1) smooth out the raw output from the
:class:`.PoseEstimator`, and to (2) estimate the standard deviation of the output estimate. The standard deviation
estimates are used for example by the :class:`.MockGPSNode` class to generate a mock `px4_msgs.VehicleGpsPosition`
message, which requires the ``eph`` and ``epv`` values (horizontal and vertical error in meters) to be set.


Custom Kalman or Particle Filter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
TODO