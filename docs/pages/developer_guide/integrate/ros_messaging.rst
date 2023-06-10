ROS messaging
____________________________________________________
The natural way to integrate GISNav with other systems is via `ROS 2 <https://docs.ros.org/>`_. GISNav depends on ROS 2
for both external and internal communication. If you have a ROS 2 node, you can talk to GISNav.

For simple integrations you might only be interested in the :ref:`Aircraft GeoPose estimate topics`. For an overview of
all available topics, see :ref:`Remapping ROS 2 topics`.


Core data flow graph
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. mermaid::
    :caption: WIP: Target core data flow graph

    graph LR
        subgraph Camera
            image_raw[camera/image_raw]
            camera_info[camera/camera_info]
        end

        subgraph MAVROS
            pose[mavros/local_position/pose]
            global[mavros/global_position/global]
            home[mavros/home_position/home]
            attitude[mavros/gimbal_control/device/attitude_status]
        end

        subgraph GISNode
            geopose[gisnav/vehicle/geopose]
            altitude[gisnav/vehicle/altitude]
            geopoint_track[gisnav/ground_track/geopoint]
            altitude_track[gisnav/ground_track/altitude]
            orthoimage[gisnav/orthoimage]
        end

        subgraph CVNode
            geopose_estimate[gisnav/vehicle/geopose/estimate]
            altitude_estimate[gisnav/vehicle/altitude/estimate]
        end

        pose -->|geometry_msgs/Pose| GISNode
        global -->|sensor_msgs/NavSatFix| GISNode
        home -->|mavros_msgs/HomePosition| GISNode
        attitude -->|mavros_msgs/GimbalDeviceAttitudeStatus| CVNode
        attitude -->|mavros_msgs/GimbalDeviceAttitudeStatus| GISNode
        geopose -->|geographic_msgs/GeoPose| CVNode
        altitude -->|mavros_msgs/Altitude| CVNode
        camera_info -->|sensor_msgs/CameraInfo| GISNode
        camera_info -->|sensor_msgs/CameraInfo| CVNode
        orthoimage -->|gisnav_msgs/OrthoImage3D| CVNode
        altitude_track -->|mavros_msgs/Altitude| CVNode
        geopoint_track -->|geographic_msgs/GeoPoint| CVNode
        image_raw -->|sensor_msgs/Image| CVNode


Motivation for the data flow graph design:

1. **Unidirectional Flow:**

  The system is designed to avoid bidirectional loops that could potentially
  cause significant delays. This streamlined, one-way flow facilitates real-time
  operation and enhances performance.

2. **Modular Structure:**

  Rather than having a monolithic single node, which can be challenging to
  maintain, the architecture is broken down into multiple specialized nodes.
  This modular approach allows for focused expertise within each node, such as
  the dedicated OpenCV node for image processing and the GIS library node for
  geographic information processing.

3. **Application Integration:**

  The architecture includes specific nodes for application integration, such as
  the MockGPSNode. This design facilitates integration with various applications
  and aids in the maintainability and extensibility of the system.


Remapping ROS 2 topics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
To integrate GISNav with your own ROS system, you will likely have to do some topic name remapping. See the examples
below on how to :ref:`Launch from ROS launch file` and :ref:`Run individual node` with remapped topic names:

.. tab-set::

    .. tab-item:: Launch file
        :selected:

        The below diff is an example remapping for the camera topics for :class:`.PoseEstimationNode`:

        .. literalinclude:: ../../../../launch/examples/base_camera_topic_remap.launch.py
            :diff: ../../../../launch/base.launch.py
            :caption: Camera topic name remap in a launch file
            :language: python

        To launch the example base configuration (without :class:`.MockGPSNode` nor any :class:`.AutopilotNode`)

        .. code-block:: bash
            :caption: Launch topic name remap configuration

            ros2 launch gisnav examples/base_camera_topic_remap.launch.py

    .. tab-item:: Run individual node

        The below command launches camera topics for :class:`.PoseEstimationNode`:

        .. code-block:: bash
            :caption: Camera topic name remapping example using ``ros2 run``

            cd ~/colcon_ws
            ros2 run gisnav pose_estimation_node --ros-args --log-level info \
                --params-file src/gisnav/launch/params/pose_estimation_node.yaml \
                 -r camera/camera_info:=camera_info \
                 -r camera/image_raw:=image

All\* default ROS topic names used by GISNav are listed in the :py:mod:`.messaging` module:

.. note::
    \* :class:`.CameraSubscriberNode` currently uses its own hard-coded topic names and does not use the
    :py:mod:`.messaging` module.

.. literalinclude:: ../../../../gisnav/nodes/messaging.py
    :caption: :py:mod:`.messaging` module ROS topic name defaults
    :start-after: # region ROS topic names
    :end-before: # endregion ROS topic names
    :language: python

Camera topics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
GISNav's nodes use the :class:`.CameraSubscriberNode` abstract base class to subscribe to
:class:`sensor_msgs.msg.CameraInfo` and :class:`sensor_msgs.msg.Image` topics. The topic default names are stored in
the :py:attr:`.CameraSubscriberNode.ROS_CAMERA_INFO_TOPIC` and :py:attr:`.CameraSubscriberNode.ROS_IMAGE_TOPIC`
class constants.

The :class:`.CameraSubscriberNode` parent class handles the :class:`sensor_msgs.msg.CameraInfo` message and provides
the info through the :py:attr:`.CameraSubscriberNode.camera_data` property, but leaves the handling of the
:class:`sensor_msgs.msg.Image` to the extending classes:

.. literalinclude:: ../../../../gisnav/nodes/base/camera_subscriber_node.py
    :caption: :py:meth:`.CameraSubscriberNode.__camera_info_callback` method
    :pyobject: CameraSubscriberNode.__camera_info_callback

.. literalinclude:: ../../../../gisnav/nodes/base/camera_subscriber_node.py
    :caption: :py:meth:`.CameraSubscriberNode.camera_data` property
    :pyobject: CameraSubscriberNode.camera_data

.. literalinclude:: ../../../../gisnav/nodes/base/camera_subscriber_node.py
    :caption: :py:meth:`.CameraSubscriberNode.image_callback` abstract method
    :pyobject: CameraSubscriberNode.image_callback

.. note::
    In the KSQL airport SITL demo, ``gscam`` was used in earlier versions of GISNav to publish the
    :class:`sensor_msgs.msg.CameraInfo` and :class:`sensor_msgs.msg.Image` messages. Newer versions use the Gazebo
    ROS camera plugin which is also based on gstreamer. The camera topics are not published over the PX4-ROS 2 bridge.

Aircraft GeoPose estimate topics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
:class:`.PoseEstimationNode` is responsible for outputting the aircraft's geographical pose estimate to the
:py:attr:`.messaging.ROS_TOPIC_VEHICLE_GEOPOSE_ESTIMATE` and :py:attr:`.messaging.ROS_TOPIC_VEHICLE_ALTITUDE_ESTIMATE`
topics. The altitude topic provides `additional altitude types such as above-mean-sea-level (AMSL)`_ altitude. You can
see the message types defined in the class's ``__init__`` method:

.. _additional altitude types such as above-mean-sea-level (AMSL): https://ardupilot.org/copter/docs/common-understanding-altitude.html

.. literalinclude:: ../../../../gisnav/nodes/pose_estimation_node.py
    :caption: :meth:`.PoseEstimationNode.__init__` publishers assignment
    :start-after: # region publishers
    :end-before: # endregion publishers
    :language: python

These two messages are used by :class:`.MockGPSNode` to generate and publish the mock GPS message that the autopilot
will use for navigation.
