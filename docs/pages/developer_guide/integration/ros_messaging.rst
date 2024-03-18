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
    :caption: Nodes and published topics

    graph TB
        MAVROS -->|"NavSatFix"| BBoxNode
        MAVROS -->|"GimbalDeviceAttitudeStatus"| BBoxNode

        subgraph core["GISNav core nodes"]
            BBoxNode -->|"BoundingBox"| GISNode
            GISNode -->|"Image"| StereoNode
            StereoNode -->|"Image"| PoseNode
        end

        subgraph tf2
            tf["tf"]
            tf_static["tf_static"]
        end

        subgraph extension["GISNav extension nodes"]
            MockGPSNode["MockGPSNode"]
        end

        gscam ---->|"Image"| StereoNode

        GISNode -->|"PointCloud2\nreference_%i_%i->WGS 84"| MockGPSNode
        tf -->|"TransformStamped\nreference_%i_%i->base_link"| MockGPSNode

        PoseNode -->|"TransformStamped\ncamera_optical->world"| tf
        PoseNode -->|"TransformStamped\ncamera_optical->camera"| tf_static
        StereoNode -->|"TransformStamped\nreference_%i_%i->world"| tf
        StereoNode -->|"TransformStamped\nquery_%i_%i->query_%i_%i"| tf

        classDef tfClass fill:transparent,stroke-dasharray:5 5;
        class tf2 tfClass

tf2 transformations tree
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The below diagram depicts how the frames specified in :class:`.FrameID` relate
to each other.

.. mermaid::
    :caption: ROS frames

    graph TB
        subgraph rep["REP 103 & 105 (units in meters)"]
            earth["earth\nnot used"]
            odom["odom\nnot used"]
            map1["map"] --> base_link
            camera
            camera_optical
        end

        subgraph gisnav["GISNav (normalized image plane units, not compliant with REP 103)"]
            world
            reference
            reference_ts -.->|"Not Implemented\n(this floats because reference frame\nscaling is not compliant with REP 103)"| map1
            world -->|"StereoNode"| reference["reference"]
            world -->|"StereoNode"| reference_ts["reference_%i_%i"]
            query_ts["query_%i_%i"] -->|"PoseNode"| query_ts
            query_ts -->|"PoseNode"| world
            query -->|"PoseNode"| world
        end

        camera -->|"BBoxNode"| base_link
        camera_optical -->|"PoseNode"| camera
        camera_optical -->|"PoseNode"| world


.. note::
    * The reason for publishing the ``PointCloud2`` message separately is that
      tf2 does not support non-rigid transforms (transform from reference frame
      to :term:`WGS 84` involves scaling). The timestamp in the
      ``reference_%i_%i`` frame is used to ensure that a transformation
      chain ending in that frame is coupled with the correct ``PointCloud2``
      message.
    * The ``reference`` frame is published together with the latest
      ``reference_%i_%i`` frame to make debugging e.g. in RViz more convenient.
      This enables looking at the world to reference frame relative transformation
      in isolation.
    * :term:`tf2` is used extensively in GISNav now. Earlier versions of GISNav
      did not use on it and relied on custom topics for publishing
      transformations.
    * The ``query_%i_%i`` frames are chained up via successive iterations of
      :term:`VO`. Link from the query frame to ``reference_%i_%i`` frame is
      then found via a suitable timestamped query frame that also links to the
      ``world`` frame.
    * ``query`` and ``reference`` are aliases for the latest ``query_%i_%i``
      and ``reference_%i_%i`` frames.

.. todo::

    * From BBoxNode, publish map to ``base_link`` and ``base_link`` to ``camera``
      transformations separately to simplify implementation and reduce amount
      of maintained code.
    * Link up the GISNav tf tree with the REP 105 tf tree, possibly via suggested
      paths in diagram. Scale GISNav frames to meters.
    * Try not to mix REP 105 and OpenCV PnP problem frame names.
    * Replace ``PointCloud2`` message with JSON formatted ``String`` message?
      Choice of ``PointCloud2`` to represent an affine transform (3-by-3 matrix)
      feels arbitrary.

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
