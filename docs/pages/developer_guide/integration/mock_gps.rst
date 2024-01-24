Mock GPS messages
===================================================

.. warning::
    The configurations presented in this section are intended for simulation
    use only. Do not use these on real drone flights.

The :class:`.MockGPSNode` creates mock :class:`mavros_msgs.msg.GPSINPUT` (
:term:`ArduPilot`\*) and :class:`px4_msgs.msg.SensorGps` (:term:`PX4`) messages
from the based on the estimated :attr:`.MockGPSNode.geotransform` and
``camera`` to ``reference`` frame transformations.

.. note::
    \* Currently :class:`.MockGPSNode` does not use the MAVROS GPS_INPUT plugin
    to publish the :class:`mavros_msgs.msg.GPSINPUT` message and publishes the
    `MAVLink GPS_INPUT`_ message directly over UDP instead. This is because the
    ROS middleware possibly does not transmit the GPS_INPUT message to ArduPilot
    in the expected way.

    .. _MAVLink GPS_INPUT: https://mavlink.io/en/messages/common.html#GPS_INPUT

PX4 integration
****************************************************

The :ref:`PX4 parameter configuration` page introduces some GPS related PX4
parameters and how and where to modify them. This page introduces more
parameters that may be relevant to you depending on how you want PX4 to use
GISNav's mock GPS messages.

Configure primary GPS and blending:

* `SENS_GPS_PRIME`_ for configuring primary GPS
* `SENS_GPS_MASK`_ for configuring GPS blending criteria

.. _SENS_GPS_PRIME:  https://docs.px4.io/master/en/advanced_config/parameter_reference.html#SENS_GPS_PRIME
.. _SENS_GPS_MASK: https://docs.px4.io/master/en/advanced_config/parameter_reference.html#SENS_GPS_MASK

.. note::
    In an `earlier version of the GISNav mock GPS demo for PX4`_, primary GPS loss was simulated by manipulating the
    `SENS_GPS_PRIME`_ parameter with the `param`_ command mid-flight with GPS blending disabled. With the introduction
    of :class:`px4_msgs.msg.SensorGps` and the retirement of :class:`px4_msgs.VehicleGpsMessage` messages in later
    versions of PX4, the demo has also transitioned to simulating GPS failure the `canonical way`_.

    .. _earlier version of the GISNav mock GPS demo for PX4: https://www.youtube.com/watch?v=JAK2DPZC33w
    .. _param: https://dev.px4.io/master/en/middleware/modules_command.html#param
    .. _canonical way: https://docs.px4.io/main/en/simulation/failsafes.html#sensor-system-failure

You may also want to modify the PX4 GPS consistency gates to initially be more
tolerant of especially horitontal variance:

* `EKF2_GPS_P_GATE <https://dev.px4.io/master/en/advanced/parameter_reference.html#EKF2_GPS_P_GATE>`_
* `EKF2_GPS_P_NOISE <https://dev.px4.io/master/en/advanced/parameter_reference.html#EKF2_GPS_P_NOISE>`_
* `EKF2_GPS_V_GATE <https://dev.px4.io/master/en/advanced/parameter_reference.html#EKF2_GPS_V_GATE>`_
* `EKF2_GPS_V_NOISE <https://dev.px4.io/master/en/advanced/parameter_reference.html#EKF2_GPS_V_NOISE>`_

Ardupilot integration
****************************************************

.. todo::
    ArduPilot integration is currently broken. Needs work in both Docker images
    and in :class:`.MockGPSNode`. In the meantime, use the PX4 examples.

See the `GPS Input page in ArduPilot official documentation`_ for instructions
on how to configure the GPSInput module for ArduPilot.

.. _GPS Input page in ArduPilot official documentation: https://ardupilot.org/mavproxy/docs/modules/GPSInput.html

Below is an example for loading and configuring the module to listen on port
``25101`` on :term:`SITL` simulation startup:

.. code-block:: bash
    :caption: ArduPilot GPSInput module configuration

    cd ~/ardupilot
    python3 Tools/autotest/sim_vehicle.py -v ArduCopter -f gazebo-iris -L KSQL_Airport \
        -m '--cmd="module load GPSInput; GPSInput.port=25101"'

.. note::
    The ``KSQL_Airport`` location is not included by default, you have to
    `configure the starting location`_

    .. _configure the starting location: https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html#setting-vehicle-start-location

.. seealso::
    `List of ArduPilot GPS parameters`_ (does not include parameters prefixed
    ``SIM_GPS*``) and ArduPilot's `instructions on how to test GPS failure`_

    .. _List of ArduPilot GPS parameters: https://ardupilot.org/copter/docs/parameters.html#gps-parameters
    .. _instructions on how to test GPS failure: https://ardupilot.org/dev/docs/using-sitl-for-ardupilot-testing.html#testing-gps-failure
