ROS messaging
____________________________________________________
The natural way to integrate GISNav with other systems is via `ROS 2 <https://docs.ros.org/>`_. GISNav depends on ROS 2
for both external and internal communication. If you have a ROS 2 node, you can talk to GISNav.

For simple integrations you might only be interested in the :ref:`Aircraft GeoPose estimate topics`. For an overview of
all available topics, see :ref:`Remapping ROS 2 topics`.

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

Autopilot topics
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Depending on the launch configuration, :class:`.PX4Node` or :class:`.ArduPilotNode` is launched along with a number of
other nodes. These two nodes subscribe to your autopilot's specific ROS message types and translate them to a set of
common GISNav internal message types.

You can see the subscribed topic names and message types in the ``__init__`` methods of the two nodes:

.. note::
    See :ref:`Remapping ROS 2 topics` to adapt these nodes to your system. For example, you must remap the PX4 topic
    names from  ``fmu/out/*`` to ``fmu/*/out`` if you are using PX4 v1.13.X instead of v1.14.0-beta1.

    .. _name remapping: https://design.ros2.org/articles/static_remapping.html

.. tab-set::

    .. tab-item:: PX4
        :selected:

        .. literalinclude:: ../../../../gisnav/nodes/px4_node.py
            :caption: :py:meth:`.PX4Node.__init__` method
            :pyobject: PX4Node.__init__

    .. tab-item:: ArduPilot

        .. literalinclude:: ../../../../gisnav/nodes/ardupilot_node.py
            :caption: :py:meth:`.ArduPilotNode.__init__` method
            :pyobject: ArduPilotNode.__init__

The parent :class:`.AutopilotNode` defines the common GISNav internal message types that the extending
:class:`.PX4Node` and :class:`.ArduPilotNode` must implement. The parent class also defines publish methods that the
extending nodes call when they want to publish the GISNav internal messages. Typically this happens whenever they
receive a relevant message from the autopilot:

.. literalinclude:: ../../../../gisnav/nodes/base/autopilot_node.py
    :caption: :class:`.AutopilotNode` public properties and publish hooks
    :start-after: # region publish hooks
    :end-before: # endregion publish hooks
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
