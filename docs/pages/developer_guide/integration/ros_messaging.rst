Remap ROS topics
____________________________________________________
The natural way to integrate GISNav with other systems is via `ROS 2
<https://docs.ros.org/>`_. GISNav depends on ROS 2 for both external and
internal communication. If you have a ROS 2 node on the same network, you can
talk to GISNav.

Core data flow graph
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Take a look at the :term:`core` node ROS topography diagram to understand
how the ROS messages flow through the application:

.. mermaid::
    :caption: Nodes and published messages

    graph TB
        MAVROS -->|"NavSatFix"| BBoxNode
        MAVROS -->|"GimbalDeviceAttitudeStatus"| BBoxNode

        subgraph core["GISNav core nodes"]
            BBoxNode -->|"BoundingBox"| GISNode
            GISNode -->|"OrthoImage"| StereoNode
            StereoNode -->|"MonocularStereoImage"| PoseNode
            StereoNode -->|"OrthoStereoImage"| PoseNode
        end

        subgraph extension["GISNav extension nodes"]
            MockGPSNode["MockGPSNode"]
        end

        subgraph robot_localization
            ekf["ekf_localization_node"] -->|"Odometry"| MockGPSNode
        end

        gscam ---->|"Image"| StereoNode

        PoseNode -->|"PoseStamped"| ekf


.. todo::

    * From BBoxNode, publish map to ``base_link`` and ``base_link`` to ``camera``
      transformations separately to simplify implementation and reduce amount
      of maintained code.
    * Implement :term:`REP 105` properly (currently only partially implemented).

Remapping ROS 2 topics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
To integrate GISNav with your own :term:`ROS` system, you will likely have to do
some topic name remapping. See the examples below on how to :ref:`launch
<Use ROS 2 launch system>` and :ref:`run <Run individual ROS nodes>` GISNav ROS
nodes with remapped topic names:

.. tab-set::

    .. tab-item:: ros2 launch
        :selected:

        The below diff is an example remapping for the camera topics for :class:`.StereoNode`:

        .. literalinclude:: ../../../../gisnav/launch/examples/base_camera_topic_remap.launch.py
            :diff: ../../../../gisnav/launch/base.launch.py
            :caption: Camera topic name remap in a launch file
            :language: python

        To launch the example base configuration with remapped topics:

        .. code-block:: bash
            :caption: Launch topic name remap configuration

            ros2 launch gisnav examples/base_camera_topic_remap.launch.py

    .. tab-item:: ros2 run

        The below command launches camera topics for :class:`.StereoNode`:

        .. code-block:: bash
            :caption: Camera topic name remapping example using ``ros2 run``

            cd ~/colcon_ws
            ros2 run gisnav transform_node --ros-args --log-level info \
                --params-file src/gisnav/launch/params/transform_node.yaml \
                 -r camera/camera_info:=camera_info \
                 -r camera/image_raw:=image

Note on camera topics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

:term:`GSCam` is in GISNav to publish the :class:`sensor_msgs.msg.CameraInfo`
and :class:`sensor_msgs.msg.Image` messages. The camera topics are not published
over the :term:`MAVROS` nor :term:`micro-ROS-agent` middleware.
