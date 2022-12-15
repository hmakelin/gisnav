Launch tests
____________________________________________________
`Launch tests <https://index.ros.org/p/launch_testing/>`_ are provided for smoke testing launch files for common
launch configurations in the :py:mod:`test.launch` package:

.. tab-set::

    .. tab-item:: PX4
        :selected:

        .. code-block:: bash
            :caption: Run ROS launch tests for PX4 (Fast DDS) launch configuration

            cd ~/colcon_ws
            launch_test src/gisnav/test/launch/test_px4_launch.py

    .. tab-item:: ArduPilot

        .. code-block:: bash
            :caption: Run ROS launch tests for ArduPilot (MAVROS) launch configuration

            cd ~/colcon_ws
            launch_test src/gisnav/test/launch/test_ardupilot_launch.py
