**************************************************
User Guide
**************************************************
This section provides instruction on how you can integrate GISNav with your own project as well as configure and extend
its functionality so match your use case.

You should start from `Extend BaseNode`_ and only move on to the other sections if your project needs more specific
configuration.


Integrate GISNav
====================================================
The `ROS 2 <https://docs.ros.org/>`_ nodes can be found in the :py:mod:`.python_px4_ros2_map_nav.nodes` package.
The package includes the :class:`.BaseNode` abstract base class which must be extended by all implementing nodes.
The :class:`.MockGPSNode` implementation is provided for demonstration to help you get started with your own node.


.. _Extend BaseNode:

Extend BaseNode
____________________________________________________
The :class:`.BaseNode` abstract base class extends the :class:`rclpy.node.Node` class by providing a new
:meth:`.publish` method. The method provides a dictionary of the airborne drone's position and attitude estimates to
make integration to other systems (e.g. via a ROS publisher) convenient. To integrate GISNav with your solution, you
must implement the :class:`.BaseNode` class by writing your own :meth:`.publish` method:

.. note::
    Currently the attitude of the (gimbal stabilized) camera is returned, not the attitude of the vehicle itself.

.. code-block:: python
    :caption: Example of custom node that prints output to console

    import pprint
    from python_px4_ros2_map_nav.nodes import BaseNode

    class MyNode(BaseNode):

        def __init__(self, name, share_dir):
            self.super().__init__(name, share_dir)
            self.pp = pprint.PrettyPrinter(indent=4)

        def publish(self, output_data):
            self.pp.print(output_data)

..
  Use json below for language to avoid highlight syntax, technically this is not json

.. code-block:: json
    :caption: Sample output data printed to console

    {    'alt_amsl': 125.0,
         'alt_ground': 123.5,
         'attitude': array([1, 0, 0, 0]),
         'crs': 'epsg:4326',
         'lat': 37.5,
         'lon': -122.5,
         'timestamp': 1655373141123,
         'x_sd': 0.8,
         'y_sd': 0.7,
         'z_sd': 0.3}

.. _Publish Format:

Publish Format
____________________________________________________
The dictionary items in the :meth:`.publish` method are in the following formats:

    * Latitude and longitude are provided in `WGS 84 <https://epsg.io/4326>`_.
    * Altitude above mean sea level (AMSL) and above ground is provided in meters.
    * Standard deviations are provided in meters in `ENU <https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates>`_ frame `(x, y := longitude, latitude; z := altitude)`.
    * Attitude quaternion is in (w, x, y, z) format (same as :class:`px4_msgs.VehicleAttitude` format).
    * Timestamp is synchronized with the `PX4 EKF2 reference time <https://github.com/PX4/px4_msgs/blob/master/msg/Ekf2Timestamps.msg>`_.

For more information on the dimensions and units, please see the source code for the :meth:`.Position.to_dict` method.
The :class:`.Position` class is used internally by :class:`.BaseNode` but has dependency to the internal
`GeoPandas <https://geopandas.org/>`_ based :py:mod:`python_px4_ros2_map_nav.nodes.geo` module. Therefore, a dictionary
with primitive types and numpy arrays is used instead for the public API for better accessibility.

.. note::
    If you want to access the :class:`.OutputData` instance instead of the dictionary, you can override the
    private :meth:`._publish` method, which is just a conversion wrapper for the public API

.. code-block::
    :caption: Overriding private :meth:`._publish` method to gain access to :class:`.OutputData` `(not recommended)`

    from python_px4_ros2_map_nav.nodes import BaseNode
    from python_px4_ros2_map_nav.nodes.data import OutputData

    class MyNode(BaseNode):

        ...

        def publish(self, output_data: dict):
            pass

        # Override BaseNode._publish
        def _publish(self, output_data: OutputData):
            print(output_data.xy.crs)

.. code-block::
    :caption: Sample output

    'epsg:4326'

.. _Configure PX4-ROS 2 Bridge:

Configure PX4-ROS 2 Bridge
____________________________________________________
To compute the position and attitude estimates, the :class:`.BaseNode` class automatically subscribes to the following
required telemetry and other input:

    #. :class:`px4_msgs.VehicleGlobalPosition` messages via 'VehicleGlobalPosition_PubSubTopic'
    #. :class:`px4_msgs.VehicleLocalPosition` messages via 'VehicleLocalPosition_PubSubTopic'
    #. :class:`px4_msgs.VehicleAttitude` messages via 'VehicleAttitude_PubSubTopic'
    #. :class:`px4_msgs.Image` messages via 'image_raw'
    #. :class:`px4_msgs.CameraInfo` messages via 'camera_info' *(not via PX4-ROS 2 bridge in demo)*

You may add more subscribe and publish topics if you decide to implement your own Node. You will need to edit the
``uorb_rtps_message_ids.yaml`` file as described in the
`microRTPS section of the PX4 User Guide <https://docs.px4.io/master/en/middleware/micrortps.html>`_ to ensure your
messages are passed between PX4 and your ROS node.

The dockerized environment used in the Read Me quick start has preconfigured these topics. However, you may want to
subscribe and publish to additional topics in your own node, in which case you will also need to configure the
PX4-ROS 2 bridge yourself.

.. seealso::

    `PX4-ROS 2 bridge <https://docs.px4.io/master/en/ros/ros2_comm.html>`_ for further information on PX4-ROS 2 bridge

Modify ROS Parameters
____________________________________________________
ROS parameter server is used to manage the configuration of the :class:`.BaseNode` instance at runtime. An example
configuration is provided in ``config/typhoon_h480__ksql_airport.yml``. :class:`.BaseNode` will use its own default
values so it is not necessary pass this parameter file to your ROS node.

Spin up your own node
____________________________________________________
Once you have `extended BaseNode <Extend BaseNode>`_, you can spin it up in the main script of your ``colcon`` package
(the :class:`.BaseNode` extends the ``rclpy.nodes.Node``):

.. code-block:: python

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

.. seealso::
    `ROS Publisher-Subscriber (Python) tutorial <https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html>`_ for a step-by-step guide on how to implement a ROS node.

.. _The MockGPSNode class:

Example Integration (MockGPSNode)
____________________________________________________
The :class:`.MockGPSNode` extends the :class:`.BaseNode` abstract base class to publish a mock GPS message generated
from the output. It is used in the Read Me Quick Start demo as an example of how GISNav can complement and in some
cases replace GNSS navigation.

In order for the :class:`.MockGPSNode` to work, you would need to configure your ``typhoon_h480`` build target to use
the new GPS. This can be either configured before flight in the file ``TODO``, or during flight by setting the
`SENS_GPS_PRIME <https://docs.px4.io/master/en/advanced_config/parameter_reference.html#SENS_GPS_PRIME>`_ parameter with
the `param <https://docs.px4.io/master/en/advanced_config/parameter_reference.html#SENS_GPS_PRIME>`_ command::

    param set SENS_GPS_PRIME 1

You may also want to try configuring the PX4 GPS consistency gates to initially be more tolerant for your PX4 build
target, e.g. in the ``/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/6011_typhoon_h480`` file used by the
example in ``README.md``:

    * `EKF2_GPS_P_GATE <https://dev.px4.io/master/en/advanced/parameter_reference.html#EKF2_GPS_P_GATE>`_
    * `EKF2_GPS_P_NOISE <https://dev.px4.io/master/en/advanced/parameter_reference.html#EKF2_GPS_P_NOISE>`_
    * `EKF2_GPS_V_GATE <https://dev.px4.io/master/en/advanced/parameter_reference.html#EKF2_GPS_V_GATE>`_
    * `EKF2_GPS_V_NOISE <https://dev.px4.io/master/en/advanced/parameter_reference.html#EKF2_GPS_V_NOISE>`_

You will also need to make PX4 receive the :class:`px4_msgs.VehicleGpsMessage` messages over the `PX4-ROS 2 Bridge`_,
as described in the `PX4 User Guide <https://docs.px4.io/master/en/>` for the version of PX4 you are using.

.. _WMS Client:

WMS Client
===================================================
The :class:`.BaseNode` continuously requests new map rasters from a WMS endpoint when the drone moves away from the
area defined by previous maps. The requests are handled by the :class:`.WMSClient` class.

The :class:`.WMSClient` is by default instantiated in a separate thread, but can also be run in a separate process
since under the hood it uses the :class:`multiprocessing.pool.ThreadPool` multithreading API which is compatible with
the actual multiprocessing :class:`multiprocessing.pool.Pool` API.

A :py:attr:`._wms_timer` periodically requests the :class:`.WMSClient` to fetch a new map based
on criteria defined in :meth:`._should_update_map` to keep unnecessary WMS requests to a minimum. Generally a new map
is requested if the field of view (FOV) of the vehicle's camera no longer significantly overlaps with the previously
requested map. The update behavior can be adjusted via the ROS parameter server.

.. _Pose Estimators:

Pose Estimators
===================================================

.. _SuperGlue & LoFTR:

SuperGlue & LoFTR
____________________________________________________
Two pose estimators, SuperGlue and SuperGlue inspired LoFTR are provided with LoFTR as the default pose estimator.
These were seen as state-of-the-art image matching algorithms at the time the software was written but newer algorithms
may provide more reliable matching. Note that SuperGlue has restrictive licensing requirements if you are planning to
use it for your own project (see license file in the repository), while LoFTR has a permissive license.

.. warning::
    LoFTR uses SuperGlue for *optimal transport* so make sure you use the *dual-softmax* version instead or otherwise
    SuperGlue licensing terms apply.

The initialization parameters for the pose estimators are currently passed via a configuration file, which :class:`.BaseNode`
reads when it initializes the :class:`.PoseEstimator`. See the provided ``config/loftr_params.yml`` and
``config/superglue_params.yml`` files for an example. These files can be provided along with other ROS parameters like
in the example ``config/typhoon_h480__ksql_airport.yml`` file:

.. code-block:: yaml

    map_nav_node:
      ros__parameters:
        pose_estimator:
          #params_file: 'config/superglue_params.yml'  # For parsing args for matcher's static initializer method
          params_file: 'config/loftr_params.yml'

.. _Extend Pose Estimator:

Extend PoseEstimator
____________________________________________________

You must extend the :class:`.PoseEstimator` abstract base and write your own :meth:`.estimate_pose` method to implement
your own pose estimator. If your pose estimator is keypoint-based, you may want to extend
:class:`.KeypointPoseEstimator` and implement the :meth:`.find_matching_keypoints` method instead. The base classes
implement the required static initializer and worker methods that are required to make them work with multithreading
and multiprocessing.

You can then either provide an instance of your class to your node directly:

.. code-block:: python

    from python_px4_ros2_map_nav.nodes.base_node import BaseNode

    class MyNode(BaseNode):
        ...

    my_node = MyNode()
    my_pose_estimator = MyPoseEstimator()
    my_node.set_pose_estimator(my_pose_estimator)

If you want to setup your :class:`.PoseEstimator` in a separate process, you cannot pass an instance and must pass a
reference to the class name with initargs instead:

.. code-block:: python

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

You can use the below snippets to get started with your own :class:`.PoseEstimator`:

.. code-block:: python

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

.. _Keypoint-Based Pose Estimator:

Keypoint-Based Pose Estimator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
If you want to create a :class:`.KeypointPoseEstimator`, you can also start with the below snippet:

.. code-block:: python

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


.. _Configuration:

Configuration
____________________________________________________
You would then need to create a configuration file ``config/my_custom_pose_estimator.yml`` that tells GISNav
how to initialize your new pose estimator. The configuraiton file will inclue the full path and initialization
arguments::

    class_name: 'python_px4_ros2_map_nav.pose_estimators.my_pose_estimator.MyPoseEstimator'
    args:
      - 15  # _min_matches

.. _Dynamic Loading:

Dynamic Loading
____________________________________________________
:class:`.BaseNode` supports dynamic loading of the :class:`.pose_estimators.PoseEstimator`, so for example a
specialized neural net or other model to replace the previous one could be swapped in mid-flight if needed. This would
require setting the new :class:`.pose_estimators.PoseEstimator` initialization arguments via the ROS parameter server
and using a ROS service (NOT IMPLEMENTED) to re-initialize the new pose estimator.

.. _Kalman Filter:

Kalman Filter
===================================================
An embedded :class:`.SimpleFilter` Kalman filter is included to (1) smooth out the choppiness of the raw output from
the :class:`.PoseEstimator`, and to (2) estimate the standard deviation of the position estimate. The standard deviation
estimates are used for example by the :class:`.MockGPSNode` class to generate a mock `px4_msgs.VehicleGpsPosition`
message, which requires the ``eph`` and ``epv`` values (horizontal and vertical error in meters) to be set.
