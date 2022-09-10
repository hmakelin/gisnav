**************************************************
Developer Guide
**************************************************

.. warning::
    GISNav is untested and has only been demonstrated in a software-in-the-loop (SITL) simulation environment.
    Do not use this software for real drone flights.

This section provides instruction on how you can integrate GISNav with your project as well as configure and extend
it to match your use case.

You should start from `Extend BaseNode`_ and only move on to the other sections if your project needs more specific
configuration.


Integrate GISNav
====================================================
The `ROS 2 <https://docs.ros.org/>`_ nodes can be found in the :py:mod:`.gisnav.nodes` package. The package includes
the :class:`.BaseNode` abstract base class which must be extended by all implementing nodes. The :class:`.MockGPSNode`
sample class is provided for demonstration and to help you get started with your own node.


.. _Extend BaseNode:

Extend BaseNode
____________________________________________________
The :class:`.BaseNode` abstract base class extends :class:`rclpy.node.Node` by providing a new
:meth:`.BaseNode.publish` method. The method provides you a :class:`.Position` instance to make integration to other
systems (e.g. via a ROS publisher) convenient.

To integrate GISNav with your project, you must implement the :class:`.BaseNode` class by writing your own
:meth:`.BaseNode.publish` method:

.. code-block:: python
    :caption: Example of custom node that logs the position estimates

    from gisnav.nodes import BaseNode

    class MyNode(BaseNode):

        def __init__(self, name, share_dir):
            self.super().__init__(name, share_dir)

        def publish(self, position):
            self.get_logger().info(f'Estimated WGS 84 lat: {position.xy.lat}, lon: {position.xy.lon}.')

.. _Position Class:

Position Class
____________________________________________________
The attributes in the :class:`.Position` input to the :meth:`.BaseNode.publish` method are in the following formats:

    * Latitude and longitude are provided in `WGS 84 <https://epsg.io/4326>`_, although :class:`.GeoPoint` can provide the values in other CRS as well
    * Altitude above mean sea level (AMSL) and above ground is provided in meters.
    * Standard deviations are provided in meters in `ENU <https://en.wikipedia.org/wiki/Local_tangent_plane_coordinates>`_ frame `(x, y := longitude, latitude; z := altitude)`.
    * Attitude quaternion is in (x, y, z, w) format (SciPy format, :class:`px4_msgs.VehicleAttitude` has different format).
    * Timestamp is synchronized with the `PX4 EKF2 reference time <https://github.com/PX4/px4_msgs/blob/master/msg/Ekf2Timestamps.msg>`_.

.. note::
    Currently the attitude of the (gimbal stabilized) camera is returned, not the attitude of the vehicle itself.

For more information on the dimensions and units, please see the source code for :meth:`.Position`. The x and y
coordinates (in ENU frame) are provided as a :class:`.GeoPoint`, which is a wrapper for :class:`geopandas.GeoSeries`.

.. _ROS 2 Topic Configuration:

ROS 2 Topic Configuration
____________________________________________________
To compute the position and attitude estimates, the :class:`.BaseNode` class automatically subscribes to the following
required telemetry and video input ROS topics:

    #. :class:`px4_msgs.VehicleGlobalPosition` messages via ``VehicleGlobalPosition_PubSubTopic``
    #. :class:`px4_msgs.VehicleLocalPosition` messages via ``VehicleLocalPosition_PubSubTopic``
    #. :class:`px4_msgs.VehicleAttitude` messages via ``VehicleAttitude_PubSubTopic``
    #. :class:`px4_msgs.GimbalDeviceSetAttitude` messages via ``GimbalDeviceSetAttitude_PubSubTopic``
    #. :class:`px4_msgs.Image` messages via ``image_raw``
    #. :class:`px4_msgs.CameraInfo` messages via ``camera_info``

.. note::
    In the Mock GPS Example, ``gscam`` is used to stream the UDP stream to the ``image_raw`` and ``camera_info`` ROS
    topics. They are not broadcast via the PX4-ROS 2 bridge.

You may need to add more subscribe and publish topics if you decide to implement your own node. You may need to edit
the ``uorb_rtps_message_ids.yaml`` file as described in the
`supported UORB messages <https://docs.px4.io/master/en/middleware/micrortps.html#supported-uorb-messages>`_ section of
the PX4 User Guide.

.. seealso::
    `PX4-ROS 2 bridge <https://docs.px4.io/master/en/ros/ros2_comm.html>`_ for more information on the PX4-ROS 2 bridge

Modify ROS Parameters
____________________________________________________
ROS parameter server is used to manage the configuration of the :class:`.BaseNode` instance at runtime. An example
configuration is provided in ``config/typhoon_h480__ksql_airport.yml``. :class:`.BaseNode` has pre-configured default
values for all required parameters, so it is not necessary pass this parameter file to your ROS node. However, it is
likely that you will at least need to edit the `WMS Client`_ URL to get GISNav working. To initialize :class:`.BaseNode`
with your own parameter values, you will need to provide it with the YAML parameter file in your

.. code-block:: bash
    :caption: Launch with ``ros2 run``

    ros2 run gisnav mock_gps_node --ros-args --log-level info --params-file src/gisnav/config/typhoon_h480__ksql_airport.yml


.. code-block:: bash
    :caption: Launch with launch file

    ros2 launch gisnav mock_gps_node.launch.py


Spin up your own node
____________________________________________________
Once you have `extended BaseNode <Extend BaseNode>`_, you can spin it up in the main script of your ``colcon`` package
(:class:`.BaseNode` extends ``rclpy.nodes.Node``):

.. code-block:: python

    import rclpy
    from my_package import MyNode

    def main(args=None):
        rclpy.init(args=args)
        my_node = MyNode()
        rclpy.spin(my_node)
        my_node.destroy_timers()    # BaseNode method, see API reference
        my_node.terminate_pools()   # BaseNode method, see API reference
        my_node.destroy_node()
        rclpy.shutdown()

    if __name__ == '__main__':
        main()

.. seealso::
    `ROS Publisher-Subscriber (Python) tutorial <https://docs.ros.org/en/foxy/Tutorials/Writing-A-Simple-Py-Publisher-And-Subscriber.html>`_ for a step-by-step guide on how to implement a ROS node.

.. _The MockGPSNode class:

Example Integration (MockGPSNode)
____________________________________________________
.. warning::
    The configurations presented in this section are intended for simulation use only. Do not attempt these on a real
    flight.

The :class:`.MockGPSNode` extends the :class:`.BaseNode` abstract base class to publish a mock
:class:`px4_msgs.SensorGps` message to the PX4-ROS 2 bridge ``/fmu/sensor_gps/in`` topic.

You can configure your PX4 to use the new GPS only to simulate loss of primary GPS. This can be either configured
before flight in the file ``~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/6011_typhoon_h480``, or during
flight by setting the `SENS_GPS_PRIME <https://docs.px4.io/master/en/advanced_config/parameter_reference.html#SENS_GPS_PRIME>`_ parameter with
the `param <https://dev.px4.io/master/en/middleware/modules_command.html#param>`_ command:

.. code-block::
    :caption: Use GISNav as primary GPS `(assumes GISNav mock GPS node publishes with ``selection=1``)`

    param set SENS_GPS_PRIME 1

.. note::
    If you disable primary GPS in the file before flight, you will not be able to takeoff in Mission mode since GISNav
    cannot provide a mock GPS fix until the drone is already above the minimum configured flight altitude
    (``misc.min_match_altitude`` ROS parameter).

.. seealso::
    See `SENS_GPS_MASK <https://docs.px4.io/v1.12/en/advanced_config/parameter_reference.html#SENS_GPS_MASK>`_ parameter
    for configuring GPS blending in PX4


You may also want to try configuring the PX4 GPS consistency gates to initially be more tolerant for your build
target, e.g. in the ``6011_typhoon_h480`` file mentioned earlier in this section:

    * `EKF2_GPS_P_GATE <https://dev.px4.io/master/en/advanced/parameter_reference.html#EKF2_GPS_P_GATE>`_
    * `EKF2_GPS_P_NOISE <https://dev.px4.io/master/en/advanced/parameter_reference.html#EKF2_GPS_P_NOISE>`_
    * `EKF2_GPS_V_GATE <https://dev.px4.io/master/en/advanced/parameter_reference.html#EKF2_GPS_V_GATE>`_
    * `EKF2_GPS_V_NOISE <https://dev.px4.io/master/en/advanced/parameter_reference.html#EKF2_GPS_V_NOISE>`_

.. note::
    You must ensure that PX4 is receiving the :class:`px4_msgs.VehicleGpsMessage` messages over the `PX4-ROS 2 Bridge`_.

    You can check that the messages are being published with:

    .. code-block::

        ros2 topic echo VehicleGpsPosition_PubSubTopic


.. _WMS Client:

WMS Client
===================================================
The :class:`.BaseNode` continuously requests new map rasters from a WMS endpoint when the drone or the drone camera's
field of view moves away from the area defined by previous maps. The requests are handled by :class:`.WMSClient`.

The :class:`.WMSClient` class is by default instantiated in a separate process, but can also be run in a separate thread
to reduce serialization overhead, since under the hood it uses the :class:`multiprocessing.pool.Pool` API which is
compatible with the :class:`multiprocessing.pool.ThreadPool` multithreading API.

.. note::
    Multithreading must be enabled in :class:`.BaseNode` source code, currently no configuration parameter for it exists

A :py:attr:`._wms_timer` periodically requests the :class:`.WMSClient` to fetch a new map based
on criteria defined in :meth:`._should_update_map` to keep unnecessary WMS requests to a minimum. Generally a new map
is requested if the field of view (FOV) of the vehicle's camera no longer significantly overlaps with the previously
requested map. The update behavior can be adjusted via the ROS parameter server through the parameters under the
``wms.map_update`` namespace.

.. _Pose Estimators:

Pose Estimators
===================================================

.. _SuperGlue & LoFTR:

SuperGlue & LoFTR
____________________________________________________
Two pose estimators, SuperGlue and SuperGlue-inspired LoFTR, are provided with LoFTR as the default pose estimator.
These were seen as state-of-the-art image matching networks at the time GISNav was written. However, newer networks may
provide better results.

.. note::
    SuperGlue has restrictive licensing requirements (see license file in the repository), while LoFTR has a permissive
    license.

.. warning::
    LoFTR uses SuperGlue for *optimal transport* so make sure you use the *dual-softmax* version instead or otherwise
    SuperGlue licensing terms apply.


.. _Extend Pose Estimator:

Extend PoseEstimator
____________________________________________________
You must extend the :class:`.PoseEstimator` abstract base and write your own :meth:`.PoseEstimator.estimate_pose`
method to implement your own pose estimator. If your pose estimator is keypoint-based, you may want to extend
:class:`.KeypointPoseEstimator` and implement the :meth:`.find_matching_keypoints` method instead. The base classes
implement the required static initializer and worker methods that make them work with Python's
:class:`.multiprocessing.pool.Pool` and :class:`.multiprocessing.pool.ThreadPool` APIs.

You can use the below snippets to get started with your own :class:`.PoseEstimator`:

.. code-block:: python

    from typing import Optional
    from python_px4_ros2_map_nav.pose_estimators.pose_estimator import PoseEstimator

    class MyPoseEstimator(PoseEstimator):

        def __init__(self):
            # TODO: implement initializer
            raise NotImplementedError

        def estimate_pose(query, reference, k, guess = None, elevation_ref = None):
            """Returns pose between query and reference images"""
            # Do your pose estimation magic here
            #r = ...  # np.ndarray of shape (3, 3)
            #t = ...  # np.ndarray of shape (3, 1)
            #return r, t
            raise NotImplementedError

.. note::
    If you can't estimate a pose with the given query and reference frames, you can return ``None`` from your
    :meth:`.PoseEstimator.estimate_pose`

.. _Keypoint-Based Pose Estimator:

Keypoint-Based Pose Estimator
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
If you want to create a :class:`.KeypointPoseEstimator`, you can also start with the below snippet:

.. code-block:: python

    from gisnav.pose_estimators.keypoint_pose_estimator import KeypointPoseEstimator

    class MyPoseEstimator(KeypointPoseEstimator):

        def __init__(self):
            # TODO: implement initializer
            raise NotImplementedError

        def find_matching_keypoints(query, reference):
            """Returns matched keypoints between query and reference images"""
            # Find matching keypoints here
            #mkp_qry = ...
            #mkp_ref = ...
            #return mkp_qry, mkp_ref
            raise NotImplementedError

.. _Configuration:

Configuration
____________________________________________________
After you have implemented your pose estimator, you need to tell :class:`.BaseNode` where to find its initialization
arguments in your ROS YAML parameter file:

.. code-block::

    my_node:
        ros__parameters:
            pose_estimator:
              params_file: 'config/my_node_params.yaml'

See the provided ``loftr_params.yaml`` and ``superglue_params.yaml`` for examples on how to format the file.

Testing
====================================================
Unit & ROS 2 integration tests
____________________________________________________
First you must install the dev dependencies for your workspace:

.. code-block:: bash

    python3 -m pip install -r requirements-dev.txt

You can then run existing tests in the ``test`` folder with:

.. code-block:: bash

    cd ~/px4_ros_com_ros2
    launch_test src/gisnav/test/test_mock_gps_node.py

For code coverage you can use ``coverage.py``. See the
`official instructions <https://coverage.readthedocs.io/en/6.4.1/source.html>`_ on how to configure what source files
to measure:

.. code-block:: bash

    cd ~/px4_ros_com_ros2
    python3 -m coverage run --branch --include */site-packages/gisnav/* src/gisnav/test/test_mock_gps_node.py
    python3 -m coverage report

SITL tests
____________________________________________________
SITL tests are under the ``test/sitl`` folder. They are simple Python scripts:

.. code-block:: bash

    cd ~/px4_ros_com_ros2/src/gisnav/test/sitl
    python sitl_test_mock_gps_node.py

Flight Log Analysis
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The flight log generated by the SITL test can be analyzed with the Jupyter notebooks in ``test/ulog_analysis`` folder.
You must first start ``jupyter-notebook``:

.. code-block:: bash

    cd ~/px4_ros_com_ros2/src/gisnav/test/sitl/ulog_analysis
    jupyter-notebook

The notebook documents the analysis and displays the results.

Documentation
====================================================

If you have the development environment setup, you can generate this GISNav documentation yourself with Sphinx:

.. code-block:: bash

    cd ~/colcon_ws/src/gisnav
    python3 -m pip install -r requirements-dev.txt
    cd docs
    make html


The HTML documentation will then appear in the ``_build/`` folder.
