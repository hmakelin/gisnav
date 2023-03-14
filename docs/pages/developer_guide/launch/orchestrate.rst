GISNav service orchestration
____________________________________________________
The ``Makefile`` in the ``docker`` folder defines phony make targets that can be used to launch GISNav along with
supporting services in various configurations. This builds on top of the services defined in the
``docker/docker-compose.yaml`` file to define groups of related services that should be spun up together depending on
use case.

.. warning::
    If your offboard computer (not onboard companion computer like Jetson Nano) is based on ``arm64``, the ``Makefile``
    targets and the ``mapserver`` Docker image especially may currently not work. See below for more information on
    what is meant by *offboard*, *onboard*, and *companion*.

For example, to build and run GISNav with all supporting services required for the mock GPS demo, you could do:

.. code-block:: bash

    cd ~/colcon_ws/src/gisnav
    make -C docker demo

Generally the targets are named using the following scheme to suggest how they should be used:

.. code-block:: bash

    cd ~/colcon_ws/src/gisnav
    make -C docker <up/down>-<companion/offboard>-<sitl/hil>-<middleware/dev/test/...>-<px4/ardupilot>

The second to last group with options like "middleware", "dev" etc. is optional and is not used by all make targets.

Explanation on what the different options for the make targets mean is provided below:

.. list-table:: GISNav service orchestration jargon
   :header-rows: 1
   :widths: 30 70

   * - Option
     - Explanation
   * - onboard
     - Anything carried by the drone that in production use would draw power from the drone battery, including the fmu
       and the companion.
   * - offboard
     - Any computer (e.g. more powerful desktop) that is not carried by the drone in production use and that does not
       draw power from the drone battery.
   * - fmu
     - Flight Management Unit (FMU), the onboard flight controller computer board such as Pixhawk.
   * - companion
     - Onboard companion computer that is separate from the fmu. E.g. Nvidia Jetson Nano.
   * - autopilot
     - Supported autopilot software such as PX4 or Ardupilot. Not the fmu.
   * - sitl
     - Software-In-The-Loop (SITL) autopilot simulation offboard.
   * - hil
     - Hardware-In-The-Loop (HIL) autopilot simulation (onboard) on fmu. Not "HITL" to better distinguish from SITL.
   * - middleware
     - Software that sits between the autopilot and gisnav. In hil this is assumed to run on the same companion as
       gisnav although it is not strictly necessary if there are multiple companions.
   * - test
     - Services that are used to support automated testing.
   * - dev
     - Services that are used to support development.
   * - up
     - Targets that spin up docker compose services (docker compose "up").
   * - build
     - Targets that build docker compose services (docker compose "build").

For example: 'fmu' is always 'onboard', 'autopilot' can be simulated 'onboard' or 'offboard' depending on whether it is
'sitl' or 'hil', if 'autopilot' is simulated in 'hil', then it follows that it runs on 'fmu', etc.

Below you can take a look at what's included in the Makefile:

.. literalinclude:: ../../../../docker/Makefile
    :caption: Container orchestration with Make!
    :start-after: # docker compose section start
    :end-before: # docker compose section end
    :language: make
