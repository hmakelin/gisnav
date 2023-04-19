Service orchestration
____________________________________________________
To launch GISNav with supporting services, use the ``Makefile`` in the
``docker`` folder. The Makefile defines phony make targets used to build
various configurations based on the services defined in docker-compose.yaml.

For example, to run GISNav with all supporting services required for the mock
GPS demo, use the command:

.. code-block:: bash
    :caption: Run GISNav mock GPS demo

    cd ~/colcon_ws/src/gisnav
    make -C docker demo


Generally, the ``Makefile`` targets follow the
``<build>-<onboard/offboard>-<sitl/hil>-<middleware/test/dev>-<px4/ardupilot>``
scheme, with the ``<build>`` and ``<middleware/test/dev>`` options being optional:

.. code-block:: bash
    :caption: Run a generic GISNav deployment

    cd ~/colcon_ws/src/gisnav
    make -C docker <build>-<onboard/offboard>-<sitl/hil>-<middleware/dev/test/...>-<px4/ardupilot>

.. warning::
    If your offboard computer (not onboard companion computer like Jetson Nano) is based on ``arm64``, the ``Makefile``
    targets and the ``mapserver`` Docker image especially may currently not work. See below for more information on
    what is meant by *offboard*, *onboard*, and *companion*.

There are some constraints: for example the target ``fmu`` is always ``onboard``,
and the autopilot can be simulated ``onboard`` or ``offboard`` for ``hil`` or
``sitl``, respectively.

A jargon list explaining each term is provided below.

- ``Onboard``: Refers to anything carried by the drone that would draw power from the drone battery, including the Flight Management Unit (FMU) and the onboard companion computer (e.g., Nvidia Jetson Nano). Synonym for companion for the targets in the ``Makefile``.
- ``Offboard``: Refers to any computer (e.g., a more powerful desktop) that is not carried by the drone and does not draw power from the drone battery.
- ``FMU``: Flight Management Unit, the onboard flight controller computer board (e.g., Pixhawk).
- ``Companion``: Refers to the onboard companion computer that is separate from the FMU (e.g., Nvidia Jetson Nano).
- ``Autopilot``: Refers to the supported autopilot software, such as PX4 or Ardupilot. Not the FMU itself.
- ``SITL``: Software-In-The-Loop autopilot simulation that runs offboard.
- ``HIL``: Hardware-In-The-Loop autopilot simulation that runs onboard the FMU.
- ``Middleware``: Software that sits between the autopilot and GISNav. In HIL, this runs on the same companion as GISNav, although it is not strictly necessary if there are multiple companions.
- ``Test``: Services that are used to support automated testing.
- ``Dev``: Services that are used to support development.
- ``Build``: Targets that only build Docker Compose services instead of spinning them up.
