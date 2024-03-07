Remap ROS topics
____________________________________________________
The natural way to integrate GISNav with other systems is via `ROS 2
<https://docs.ros.org/>`_. GISNav depends on ROS 2 for both external and
internal communication. If you have a ROS 2 node on the same network, you can
talk to GISNav.

Core data flow graph
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Take a look at the :term:`core` node ROS topography diagrams to understand
how the ROS messages flow through the application:

* :ref:`BBoxNode`
* :ref:`GISNode`
* :ref:`TransformNode`
* :ref:`PoseNode`

Motivation for the data flow graph design:

* **Unidirectional Flow:**

  The system is designed to avoid bidirectional loops that could potentially
  cause significant delays. This streamlined, one-way flow facilitates real-time
  operation and enhances performance.

* **Modular Structure:**

  Rather than having a monolithic single node, which can be challenging to
  maintain, the architecture is broken down into multiple specialized nodes.
  This modular approach allows for focused expertise within each node, such as
  the dedicated OpenCV node for image processing and the GIS library node for
  geographic information processing. For the GIS node, optimization is concerned
  more about efficient IO and multithreading, while for the CV node optimization
  may mean minimizing unnecessary image array transformations and using
  multiprocessing.

* **Extensibility:**

  The architecture allows for :term:`extended functionality` via ROS messaging.
  This design facilitates integration with various applications and helps with
  the maintainability of the :term:`core` system.

.. note::
    :term:`tf2` is used extensively in GISNav now. Earlier versions of GISNav
    did not use on it and relied on custom topics for publishing transformations.

.. mermaid::

    graph TB
        MAVROS -->|"NavSatFix"| BBoxNode
        MAVROS -->|"GimbalDeviceAttitudeStatus"| BBoxNode

        subgraph core["GISNav core nodes"]
            BBoxNode -->|"BoundingBox"| GISNode
            GISNode -->|"Image"| TransformNode
            TransformNode -->|"Image"| PoseNode
        end

        subgraph tf["tf topic"]
            camera_pinhole_to_world["camera_pinhole --> world"]
            reference_to_world["reference_[timestamp] --> world"]
            map_to_camera["map --> camera"]
            camera_to_reference["camera --> reference_[timestamp]"]

        end

        subgraph tf_static["tf_static topic"]
            camera_to_camera_pinhole["camera --> camera_pinhole"]
        end

        subgraph extension["GISNav extension nodes"]
            MockGPSNode["MockGPSNode"]
        end

        gscam ---->|"Image"| TransformNode

        GISNode -->|"PointCloud2\nreference_[timestamp]"| MockGPSNode
        camera_to_reference -->|"TransformStamped"| MockGPSNode

        PoseNode --->|"TransformStamped"| camera_pinhole_to_world
        PoseNode -->|"TransformStamped"| camera_to_camera_pinhole
        TransformNode -->|"TransformStamped"| reference_to_world
        BBoxNode -->|"TransformStamped"| map_to_camera


.. note::
    The reason for publishing the ``PointCloud2`` message separately is that
    tf2 does not support non-rigid transforms (transform from reference frame
    to :term:`WGS 84` involves scaling). The timestamp in the
    ``reference_[timestamp]`` frame is used to ensure that a transformation
    chain ending in that frame is coupled with the correct ``PointCloud2``
    message.

.. todo::

    * From BBoxNode, publish map to ``base_link`` and ``base_link`` to ``camera``
      transformations separately to simplify implementation and reduce amount
      of maintained code.
    * In MockGPSNode, retrieve ``base_link`` (vehicle) to ``reference_[timestamp]``
      transform instead of ``camera`` to ``reference_[timestamp]``.
    * Try not to mix REP 105 and OpenCV PnP problem frame names.

Remapping ROS 2 topics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
To integrate GISNav with your own :term:`ROS` system, you will likely have to do
some topic name remapping. See the examples below on how to :ref:`launch
<Use ROS 2 launch system>` and :ref:`run <Run individual ROS nodes>` GISNav ROS
nodes with remapped topic names:

.. tab-set::

    .. tab-item:: ros2 launch
        :selected:

        The below diff is an example remapping for the camera topics for :class:`.TransformNode`:

        .. literalinclude:: ../../../../gisnav/launch/examples/base_camera_topic_remap.launch.py
            :diff: ../../../../gisnav/launch/base.launch.py
            :caption: Camera topic name remap in a launch file
            :language: python

        To launch the example base configuration with remapped topics:

        .. code-block:: bash
            :caption: Launch topic name remap configuration

            ros2 launch gisnav examples/base_camera_topic_remap.launch.py

    .. tab-item:: ros2 run

        The below command launches camera topics for :class:`.TransformNode`:

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
