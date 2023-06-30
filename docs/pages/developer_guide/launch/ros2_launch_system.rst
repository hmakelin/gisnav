Use ROS 2 launch system
____________________________________________________

:term:`ROS 2` provides a launch system for deploying applications consisting of
multiple nodes with preconfigured parameters.

This page describes how GISNav uses the launch system to help you when
developing locally or when making custom deployments.

Prerequisites
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. include:: _prerequisites_ros.rst

.. include:: _prerequisites_gisnav.rst

.. include:: _prerequisites_source_workspace.rst

Build colcon workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. tab-set::

    .. tab-item:: Entire workspace
        :selected:

        .. code-block:: bash
            :caption: Build entire colcon workspace

            mkdir -p ~/colcon_ws
            colcon build
            source install/setup.bash

    .. tab-item:: GISNav package only
        :selected:

        .. code-block:: bash
            :caption: Build GISNav package

            mkdir -p ~/colcon_ws
            colcon build --packages-select gisnav
            source install/setup.bash

Launch GISNav for local development
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Assuming you have already installed the ``gisnav`` colcon package, you can launch with a single command:

.. tab-set::

    .. tab-item:: PX4
        :selected:

        .. code-block:: bash
            :caption: Launch GISNav for PX4

            ros2 launch gisnav px4.dev.launch.py


    .. tab-item:: ArduPilot

        .. code-block:: bash
            :caption: Launch GISNav for ArduPilot

            ros2 launch gisnav ardupilot.dev.launch.py

.. seealso::
    See the `Creating Launch Files`_ tutorial for more information on the ROS 2 launch system

    .. _Creating Launch Files: https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html
