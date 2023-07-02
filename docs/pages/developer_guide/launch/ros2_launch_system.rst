Use ROS 2 launch system
____________________________________________________

:term:`ROS 2` provides a launch system for deploying applications consisting of
multiple nodes with preconfigured parameters.

This page describes how GISNav uses the launch system to help you when
developing locally or when making custom deployments.

Prerequisites
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. include:: _prerequisites_install_locally.rst

Build colcon workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. include:: _build_colcon_workspace.rst

Launch GISNav for local development
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Assuming you have already installed the ``gisnav`` colcon package, you can launch with a single command:

.. include:: _launch_gisnav_with_ros2_launch.rst

.. seealso::
    See the `Creating Launch Files`_ tutorial for more information on the ROS 2 launch system

    .. _Creating Launch Files: https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html
