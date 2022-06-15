Extend GISNav
===================================================
This section provides instruction and code samples that you can use to integrate GISNav with your own project by
extending any one of the provided base classes in the :py:mod:`.nodes`, :py:mod:`.pose_estimators`, :py:mod:`.filters`,
and :py:mod:`.wms_clients` packages.

You should start by implementing your own `Custom Node`_, and only move on to the other modules after you have a
running node and your project needs more specific configuration.

ROS Nodes
---------------------------------------------------
The `ROS 2 <https://docs.ros.org/>`_ nodes can be found in the :py:mod:`.python_px4_ros2_map_nav.nodes` package. The package includes the
:class:`.BaseNode` abstract base class which must be extended by all implementing nodes. You may also want to look at
the source code of the example :class:`.MockGPSNode` when implenting your own `Custom Node`_.

.. _The MockGPSNode class:

The BaseNode class
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The :class:`.BaseNode` abstract base class implements a ROS 2 node that produces the vehicle position estimate from
visual inputs. It subscribes to a number of required topics over the `PX4-ROS 2 bridge
<https://docs.px4.io/master/en/ros/ros2_comm.html>`_ and maintains a connection to a WMS endpoint.

ROS parameter server is used to manage configuration of the base node, and an example configuration is provided in
``config/typhoon_h480__ksql_airport.yml``. If you make a `Custom Node`_, you will not need this file since
:class:`.BaseNode` will use its own default values if no ROS parameters are not provided as a YAML file.

.. _PX4-ROS 2 Bridge

PX4-ROS 2 Bridge
***************************************************
The :class:`.BaseNode` subscribes to the following telemetry and other input:

    #. :class:`px4_msgs.VehicleGlobalPosition` messages via 'VehicleGlobalPosition_PubSubTopic'
    #. :class:`px4_msgs.VehicleLocalPosition` messages via 'VehicleLocalPosition_PubSubTopic'
    #. :class:`px4_msgs.VehicleAttitude` messages via 'VehicleAttitude_PubSubTopic'
    #. :class:`px4_msgs.Image` messages via 'image_raw'
    #. :class:`px4_msgs.CameraInfo` messages via 'camera_info' *(not via PX4-ROS 2 bridge in demo)*

You may add more subscribe and publish topics if you decide to implement your own Node. You will need to edit the
``uorb_rtps_message_ids.yaml`` file as described in the
`microRTPS section of the PX4 User Guide <https://docs.px4.io/v1.12/en/middleware/micrortps.html>`_ to ensure your
messages are passed between PX4 and your ROS node.

The dockerized environment used in the ``README.md`` quick start
already handles this for you, so you will not need to configure the bridge if you are running the demo. However, you
may want to susbcribe and publish to additional topics in your `Custom Node`_, in which case you will also need
to configute the PX4-ROS 2 bridge.

Publish Method
***************************************************
The :class:`.BaseNode` base class defines a :meth:`.publish` abstract method and leaves it to the implementing class
to decide what to do with the computed output. The data provided to the method is defined in :class:`.OutputData`. An
example of the contents is provided below::

Initialization
***************************************************
:class:`.BaseNode` supports lazy initialization of the :class:`.pose_estimators.PoseEstimator`,
:class:`.filters.Filter`, and :class:`.wms_clients.WMSClient` modules. You might for example want to provide your own
instance of :class:`.pose_estimator.PoseEstimator` and prevent initializing the default pose estimator when you
create your node. In that case you prevent initializing the default pose estimator by setting the
:param:`.init_pose_estimator` to ``False``::

    from python_px4_ros2_map_nav.nodes import BaseNode

    class MyNode(BaseNode):

        def __init__(self, name, share_dir, *args, **kwargs):
            super().__init__(self, name, share_dir, *args, **kwargs)

            ...

        ...

    my_node = MyNode(init_pose_estimator=False)
    my_node.set_pose_estimator(pose_estimator)

The node will run even without a pose estimator, and will simply keep logging warning messages that a pose estimator is
missing if you choose to initialize it without one, and then to never provide one.

.. _The MockGPSNode class:

The MockGPSNode class
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The :class:`.MockGPSNode` extends the :class:`.BaseNode` abstract base class to publish a mock GPS message generated
from the output. It is used in the `README.md` Quick Start demo as an example of how GISNav can complement and in some
cases replace GNSS navigation.

In order for the :class:`.MockGPSNode` to work, you would need to configure your ``typhoon_h480`` build target to use
the new GPS. This can be either configured before flight in the file ``TODO``, or during flight by setting the
`SENS_GPS_PRIME <https://docs.px4.io/v1.12/en/advanced_config/parameter_reference.html#SENS_GPS_PRIME>`_ parameter with
the `param <https://docs.px4.io/v1.12/en/advanced_config/parameter_reference.html#SENS_GPS_PRIME>`_ command::

    param set SENS_GPS_PRIME 1

You may also want to try configuring the PX4 GPS consistency gates to initially be more tolerant for your PX4 build
target, e.g. in the ``/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/6011_typhoon_h480`` file used by the
example in ``README.md``:

    * `EKF2_GPS_P_GATE <https://dev.px4.io/master/en/advanced/parameter_reference.html#EKF2_GPS_P_GATE>`_
    * `EKF2_GPS_P_NOISE <https://dev.px4.io/master/en/advanced/parameter_reference.html#EKF2_GPS_P_NOISE>`_
    * `EKF2_GPS_V_GATE <https://dev.px4.io/master/en/advanced/parameter_reference.html#EKF2_GPS_V_GATE>`_
    * `EKF2_GPS_V_NOISE <https://dev.px4.io/master/en/advanced/parameter_reference.html#EKF2_GPS_V_NOISE>`_

You will also need to make PX4 receive the :class:`px4_msgs.VehicleGpsMessage` messages over the `PX4-ROS 2 Bridge`_,
as described in the `PX4 User Guide <https://docs.px4.io/v1.12/en/>` for the version of PX4 you are using.

Custom Node
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
To integrate GISNav with your solution, you must implement the :class:`.BaseNode` class by writing a :meth:`.publish`
method::

    from python_px4_ros2_map_nav.nodes.base_node import BaseNode
    from python_px4_ros2_map_nav.data import OutputData

    class MyNode(BaseNode):

        # You can override the __init__ method and do whatever you need here
        ...

        def publish(output_data):
            """Prints the output into console"""
            print(f'Here is the position: {output_data}')

See :class:`.OutputData` for what fields are contained in the output data container.

You can see a longer example in source code for the :class:`.MockGPSNode` class, which creates a
:class:`px4_msgs.VehicleGpsPosition` mock GPS (GNSS) message out of the output and publishes it to the flight control
software via the appropriate PX4/ROS 2 bridge topic.

Once you have implemented your node, you can spin it up in the main script of your ``colcon`` package, as described in
the `ROS tutorial <https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html>`_
(The :class:`.BaseNode` extends the ``rclpy.nodes.Node``)::

    import rclpy

    # Define or import MyNode here

    def main(args=None):
        rclpy.init(args=args)
        my_node = MyNode()
        rclpy.spin(my_node)
        my_node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()

WMS Client
---------------------------------------------------
The :class:`.BaseNode` Map rasters from WMS endpoint, requested by embedded :class:`.WMSClient` instance

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

You can then either provide an instance of your class to your node directly::

    from python_px4_ros2_map_nav.nodes.base_node import BaseNode

    class MyNode(BaseNode):
        ...

    my_node = MyNode()
    my_pose_estimator = MyPoseEstimator()
    my_node.set_pose_estimator(my_pose_estimator)

If you want to setup your :class:`.PoseEstimator` in a separate process, you cannot pass an instance and must pass a
reference to the class name with initargs instead::

    from python_px4_ros2_map_nav.nodes.base_node import BaseNode

    class MyNode(BaseNode):
        ...

    class MyPoseEstimator(PoseEstimator):
        ...

    my_node = MyNode()
    my_node.set_pose_estimator(MyPoseEstimator, initargs=('hello world', 1, 2, 3), use_dedicated_process=True)

If you try to use the ``use_dedicated_process=True`` flag while providing an instance of your class, :class:`.BaseNode`
will simply log a warning and use multithreading in the same process with your :class:`.PoseEstimator` instead. This is
to prevent having to pickle and send large and complex objects over to the initializer of the secondary process.

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

An embedded :class:`.SimpleFilter` Kalman filter is included to (1) smooth out the choppiness of the raw output from
the :class:`.PoseEstimator`, and to (2) estimate the standard deviation of the position estimate. The standard deviation
estimates are used for example by the :class:`.MockGPSNode` class to generate a mock `px4_msgs.VehicleGpsPosition`
message, which requires the ``eph`` and ``epv`` values (horizontal and vertical error in meters) to be set.


Custom Kalman or Particle Filter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
TODO