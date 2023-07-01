Remap ROS topics
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
            geopose[gisnav/gis_node/vehicle/geopose]
            altitude[gisnav/gis_node/vehicle/altitude]
            geopose_track[gisnav/gis_node/ground_track/geopose]
            altitude_track[gisnav/gis_node/ground_track/altitude]
            orthoimage[gisnav/gis_node/orthoimage]
        end

        subgraph CVNode
            geopose_estimate[gisnav/cv_node/vehicle/estimated/geopose]
            altitude_estimate[gisnav/cv_node/vehicle/estimated/altitude]
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
        geopose_track -->|geographic_msgs/GeoPoint| CVNode
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
geographic information processing. For the GIS node, optimization is concerned
more about efficient IO and multithreading, while for the CV node optimization
may mean minimizing unnecessary image array transformations and using
multiprocessing.

3. **Extensibility:**

The architecture allows for :term:`extended functionality` via ROS messaging.
This design facilitates integration with various applications and helps with
the maintainability of the :term:`core` system.

Remapping ROS 2 topics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
To integrate GISNav with your own ROS system, you will likely have to do some
topic name remapping. See the examples below on how to :ref:`Use ROS 2 launch system`
and :ref:`Run individual ROS node` with remapped topic names:

.. tab-set::

    .. tab-item:: ROS 2 launch with topic name remapping
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

    .. tab-item:: Run individual ROS node with topic name remapping

        The below command launches camera topics for :class:`.PoseEstimationNode`:

        .. code-block:: bash
            :caption: Camera topic name remapping example using ``ros2 run``

            cd ~/colcon_ws
            ros2 run gisnav pose_estimation_node --ros-args --log-level info \
                --params-file src/gisnav/launch/params/pose_estimation_node.yaml \
                 -r camera/camera_info:=camera_info \
                 -r camera/image_raw:=image

Note on camera topics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

:term:`GSCam` was used in earlier versions of GISNav to publish the
:class:`sensor_msgs.msg.CameraInfo` and :class:`sensor_msgs.msg.Image` messages.
Newer versions use the :term:`Gazebo` ROS camera plugin which is also based on
:term:`GStreamer`. The camera topics are not published over the :term:`MAVROS`
middleware.
