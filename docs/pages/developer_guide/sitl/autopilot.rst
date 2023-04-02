Autopilots
==================================================
GISNav supports PX4 and ArduPilot autopilots via `MAVROS`_. Other MAVLink compatible
software may work too but have not been tested.

.. warning::
    ArduPilot is licensed under `GPLv3`_ which is more restrictive than PX4's `BSD`_ license

.. _GPLv3: https://ardupilot.org/dev/docs/license-gplv3.html
.. _BSD: https://docs.px4.io/main/en/contribute/licenses.html
.. _MAVROS: https://ardupilot.org/dev/docs/ros-connecting.html

Setup PX4
___________________________________________________

Follow the PX4 instructions to setup your `Ubuntu Development Environment`_.

.. _Ubuntu Development Environment: https://docs.px4.io/master/en/simulation/ros_interface.html

Build Gazebo simulation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Once you have installed PX4 Autopilot, you can try out the `PX4 Gazebo simulation`_ to make sure everything is working
correctly. The following commands should pop out a Gazebo window with a Typhoon H480 drone sitting somewhere in the
vicinity of San Carlos (KSQL) airport:

.. _PX4 Gazebo simulation: https://docs.px4.io/main/en/simulation/gazebo.html


.. code-block:: bash
    :caption: Run PX4 Gazebo SITL simulation at KSQL airport

    cd ~/PX4-Autopilot
    make px4_sitl gazebo_typhoon_h480__ksql_airport

.. note::
    The initial build may take several minutes

PX4 parameter configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
.. warning::
    Do not use this configuration for real drone flights. This configuration is intended for simulation use only.

To make GISNav potentially work better, you can adjust the following PX4 parameters either at runtime through the PX4
shell or the `QGroundControl Parameters screen`_, or before building the simulation in the
``~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/6011_typhoon_h480`` file :

.. _QGroundControl Parameters screen: https://docs.qgroundcontrol.com/master/en/SetupView/Parameters.html

.. code-block::
    :caption: PX4 parameter defaults for GISNav

    param set-default NAV_ACC_RAD 20.0
    param set-default MPC_YAWRAUTO_MAX 10.0
    param set-default COM_POS_FS_DELAY 5

    param set-default EKF2_GPS_P_NOISE 10
    param set-default EKF2_GPS_V_NOISE 3

    param set-default SENS_GPS_MASK 2

.. note::
    This is a sample configuration that seems to work, but you may want to experiment with the parameters.

    The first three parameters make the waypoint turns softer and reduces the yaw rate. This makes the field of view
    move and rotate more slowly especially if the camera has some pitch (is not completely nadir-facing). A slower
    moving camera field of view makes it easier for GISNav to keep track of position at tight turns and prevent the
    position delay failsafe from triggering.

    Increasing the position failsafe delay may help if your GPU is slower or GISNav for some reason cannot produce a
    position estimate for a number of subsequent frames. However as a failsafe parameter it should not be made
    unreasonably large.

    The two EKF2 parameters increase tolerance for variation in the GPS position estimate. GISNav in its
    default configuration `seems to be more accurate in estimating vertical position than horizontal position`_, so this
    configuration example also has lower tolerance for vertical position error.

    The final parameter should make PX4 blend GPS based on horizontal position accuracy.

    .. _seems to be more accurate in estimating vertical position than horizontal position: https://github.com/hmakelin/gisnav/blob/master/test/sitl/ulog_analysis/variance_estimation.ipynb

Video streaming with gscam
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The ``typhoon_h480`` build target for Gazebo SITL simulation supports UDP `video streaming`_. Here we will use
``gscam`` to publish the UDP video stream to ROS 2 to make it accessible for GISNav:

.. _video streaming: https://docs.px4.io/master/en/simulation/gazebo.html#video-streaming

Install ``gscam`` and dependencies:

.. code-block:: bash
    :caption: Install gscam and dependencies

    sudo apt-get install -y gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl ros-foxy-gscam

Use the sample camera and gstreamer configuration files in the GISNav repository to run ``gscam`` in a dedicated shell:

.. code-block:: bash
    :caption: Run gscam_node with example configuration files

    cd ~/colcon_ws
    ros2 run gscam gscam_node --ros-args --params-file src/gisnav/test/assets/gscam_params.yaml \
        -p camera_info_url:=file://$PWD/src/gisnav/test/assets/camera_calibration.yaml

.. seealso::
    See `How to Calibrate a Monocular Camera`_ on how to create a custom camera calibration file if you do not want to
    use the provided example

    .. _How to Calibrate a Monocular Camera: https://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration

Setup ArduPilot
___________________________________________________
The following tutorials should help you setup an ArduPilot SITL simulation environment:

* `Setting up SITL on Linux`_
* `Using Gazebo simulator with SITL`_
* `Connecting with ROS`_

.. _Setting up SITL on Linux:  https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html
.. _Using Gazebo simulator with SITL: https://ardupilot.org/dev/docs/using-gazebo-simulator-with-sitl.html
.. _Connecting with ROS: https://ardupilot.org/dev/docs/ros-connecting.html

The ``gazebo-iris`` model in the ArduPilot SITL simulation included in the `gisnav-docker`_ ``sitl`` service currently
has a static camera that faces directly down from the aircraft body (the ``typhoon_h480`` model in the PX4 simulation
has a proper simulated 2-axis gimbal). Because the camera is not stabilized, it possibly won't be reliable enough to
act as a full replacement for GPS in ArduPilot's mission mode, while loitering without GPS may work.

.. _gisnav-docker: https://github.com/hmakelin/gisnav-docker

.. note::
    *Unverified*: You may have to `enable virtual joystick`_ from QGroundControl settings and have it centered to
    maintain altitude in ArduPilot's Loiter mode in the SITL simulation.

    .. _enable virtual joystick: https://docs.qgroundcontrol.com/master/en/SettingsView/VirtualJoystick.html
