Mock GPS messages
===================================================
:class:`.MockGPSNode` uses the read-only ROS parameter ``px4_micrortps`` to determine whether to publish a
:class:`px4_msgs.msg.SensorGps` (PX4) or a :class:`mavros_msgs.msg.GPSINPUT` (ArduPilot)* message from the received
:class:`geographic_msgs.msg.GeoPoseStamped` and :class:`mavros_msgs.msg.Altitude` messages.

.. note::
    \* Currently :class:`.MockGPSNode` does not use the MAVROS GPS_INPUT plugin to publish a
    :class:`mavros_msgs.msg.GPSINPUT` message and publishes the `MAVLink GPS_INPUT`_ message directly over UDP instead

    .. _MAVLink GPS_INPUT: https://mavlink.io/en/messages/common.html#GPS_INPUT

Autopilot specific considerations
____________________________________________________
.. warning::
    The configurations presented in this section are intended for simulation use only. Do not use these on real drone
    flights

PX4
****************************************************
The :ref:`PX4 parameter configuration` section introduced some GPS related PX4 parameters and how and where to modify
them. This section introduces more parameters that may be relevant to you depending on how you want PX4 to use
GISNav's mock GPS messages.

Configure primary GPS and blending:

* `SENS_GPS_PRIME`_ for configuring primary GPS
* `SENS_GPS_MASK`_ for configuring GPS blending criteria

.. _SENS_GPS_PRIME:  https://docs.px4.io/master/en/advanced_config/parameter_reference.html#SENS_GPS_PRIME
.. _SENS_GPS_MASK: https://docs.px4.io/v1.12/en/advanced_config/parameter_reference.html#SENS_GPS_MASK

.. note::
    In an `earlier version of the GISNav mock GPS demo for PX4`_, primary GPS loss was simulated by manipulating the
    `SENS_GPS_PRIME`_ parameter with the `param`_ command mid-flight with GPS blending disabled. With the introduction
    of :class:`px4_msgs.msg.SensorGps` and the retirement of :class:`px4_msgs.VehicleGpsMessage` messages in later
    versions of PX4, the demo has also transitioned to simulating GPS failure the `canonical way`_.

    .. _earlier version of the GISNav mock GPS demo for PX4: https://www.youtube.com/watch?v=JAK2DPZC33w
    .. _param: https://dev.px4.io/master/en/middleware/modules_command.html#param
    .. _canonical way: https://docs.px4.io/main/en/simulation/failsafes.html#sensor-system-failure

You may also want to modify the PX4 GPS consistency gates to initially be more tolerant for your build
target:

    * `EKF2_GPS_P_GATE <https://dev.px4.io/master/en/advanced/parameter_reference.html#EKF2_GPS_P_GATE>`_
    * `EKF2_GPS_P_NOISE <https://dev.px4.io/master/en/advanced/parameter_reference.html#EKF2_GPS_P_NOISE>`_
    * `EKF2_GPS_V_GATE <https://dev.px4.io/master/en/advanced/parameter_reference.html#EKF2_GPS_V_GATE>`_
    * `EKF2_GPS_V_NOISE <https://dev.px4.io/master/en/advanced/parameter_reference.html#EKF2_GPS_V_NOISE>`_

Ardupilot
****************************************************
See the `GPS Input page in ArduPilot official documentation`_ for instructions on configuring the GPSInput module for
ArduPilot.

.. _GPS Input page in ArduPilot official documentation: https://ardupilot.org/mavproxy/docs/modules/GPSInput.html

Below is an example for loading and configuring the module to listen on port ``25101`` on SITL simulation startup:

.. code-block:: bash
    :caption: ArduPilot GPSInput module configuration

    cd ~/ardupilot
    python3 Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris -L KSQL_Airport \
        -m '--cmd="module load GPSInput; GPSInput.port=25101"'

.. note::
    The ``KSQL_Airport`` location is not included by default, you have to `configure the starting location`_

    .. _configure the starting location: https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html#setting-vehicle-start-location

.. seealso::
    `List of ArduPilot GPS parameters`_ (does not include parameters prefixed ``SIM_GPS*``) and ArduPilot's
    `instructions on how to test GPS failure`_

    .. _List of ArduPilot GPS parameters: https://ardupilot.org/copter/docs/parameters.html#gps-parameters
    .. _instructions on how to test GPS failure: https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html#testing-gps-failure