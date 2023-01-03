Autopilots
==================================================
GISNav supports PX4 via the `RTPS/DDS`_ interface and ArduPilot via `MAVROS`_.

.. warning::
    ArduPilot is licensed under `GPLv3`_ which is more restrictive than PX4's `BSD`_ license

.. _GPLv3: https://ardupilot.org/dev/docs/license-gplv3.html
.. _BSD: https://docs.px4.io/main/en/contribute/licenses.html
.. _RTPS/DDS: https://docs.px4.io/main/en/middleware/micrortps.html
.. _MAVROS: https://ardupilot.org/dev/docs/ros-connecting.html

Setup PX4
___________________________________________________
**These instructions are written for PX4 version >=1.13.0, <=1.14.0-beta1**

Follow the PX4 instructions to setup your `Ubuntu Development Environment`_ with `Fast DDS`_ and `PX4-ROS 2 Bridge`_
support.

.. _Ubuntu Development Environment: https://docs.px4.io/master/en/simulation/ros_interface.html
.. _Fast DDS: https://docs.px4.io/main/en/dev_setup/fast-dds-installation.html
.. _PX4-ROS 2 Bridge: https://docs.px4.io/main/en/ros/ros2_comm.html

Build Gazebo simulation
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Once you have installed PX4 Autopilot, you can try out the `PX4 Gazebo simulation`_ to make sure everything is working
correctly. The following commands should pop out a Gazebo window with a Typhoon H480 drone sitting somewhere in the
vicinity of San Carlos (KSQL) airport:

.. _PX4 Gazebo simulation: https://docs.px4.io/main/en/simulation/gazebo.html

.. tab-set::

    .. tab-item:: v1.14.0-beta1
        :selected:

        .. note::
            You must `create the micro-ros-agent`_ for the following command to work

        .. _create the micro-ros-agent: https://micro.ros.org/docs/tutorials/core/first_application_linux/

        .. code-block:: bash
            :caption: Run ``micro-ros-agent``

            cd ~/colcon_ws
            ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888


        .. code-block:: bash
            :caption: Run PX4 Gazebo SITL simulation at KSQL airport

            cd ~/PX4-Autopilot
            make px4_sitl gazebo_typhoon_h480__ksql_airport


    .. tab-item:: v1.13.X

        Run the microRTPS agent in a separate shell:

        .. code-block:: bash
            :name: Launch microRTPS agent
            :caption: Run microRTPS agent

            cd ~/colcon_ws
            micrortps_agent -t UDP

        Then build your simulation:

        .. code-block:: bash
            :caption: Run PX4 Gazebo SITL simulation at KSQL airport

            cd ~/PX4-Autopilot
            make px4_sitl_rtps gazebo_typhoon_h480__ksql_airport

.. note::
    The initial build may take several minutes

PX4-ROS 2 bridge topic configuration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
For GISNav the PX4-ROS 2 bridge must be configured to send the :class:`px4_msgs.msg.GimbalDeviceSetAttitude` message
and receive the :class:`px4_msgs.msg.SensorGps` message:

.. tab-set::

    .. tab-item:: v1.14.0-beta1
        :selected:

        Edit the ``~/PX4-Autopilot/src/modules/microdds_client/microdds_topics.yaml`` file by adding the following
        entries:

        .. code-block:: yaml
            :caption: PX4-Autopilot/src/modules/microdds_client/microdds_topics.yaml

            publications:

              - topic: /fmu/out/gimbal_device_set_attitude
                type: px4_msgs::msg::GimbalDeviceSetAttitude

            subscriptions:

              - topic: /fmu/in/sensor_gps
                type: px4_msgs::msg::SensorGps

    .. tab-item:: v1.13.X

        See the `ROS 2 Offboard Control Example`_ for example on how to edit the ``urtps_bridge_topics.yaml`` file in
        the ``~/PX4-Autopilot/msg/tools`` and ``~/colcon_ws/src/px4_ros_com/templates`` folders. Add the following
        entries to the files:

        .. _ROS 2 Offboard Control Example: https://docs.px4.io/main/en/ros/ros2_offboard_control.html#ros-2-offboard-control-example

        .. list-table:: ``urtps_bridge_topics.yaml``
           :header-rows: 1

           * - PX4-Autopilot/msg/tools
             - px4_ros_com_ros2/src/px4_ros_com/templates
           * - .. code-block:: yaml

                    - msg: gimbal_device_set_attitude
                      send: true

                    - msg: sensor_gps
                      receive: true

             - .. code-block:: yaml

                    - msg: GimbalDeviceSetAttitude
                      send: true

                    - msg: SensorGps
                      receive: true

After you have configured the topics, you can :ref:`Build Gazebo simulation` again.

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
