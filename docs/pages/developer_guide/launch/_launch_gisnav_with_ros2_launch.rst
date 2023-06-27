Finally, after you have your supporting services deployed you would typically
:ref:`use the ROS 2 launch system <Use ROS 2 launch system>` to launch your
locally installed development version of GISNav:

.. tab-set::

    .. tab-item:: PX4
        :selected:

        .. code-block:: bash
            :caption: Launch local development version of GISNav using ROS 2 launch system

            ros2 launch gisnav px4.dev.launch.py

    .. tab-item:: ArduPilot
        :selected:

        .. code-block:: bash
            :caption: Launch local development version of GISNav using ROS 2 launch system

            ros2 launch gisnav ardupilot.dev.launch.py
