Pixhawk & Jetson Nano HIL
____________________________________________________

This section provides an example on how to run GISNav on a :term:`Jetson Nano`
in a :term:`PX4` :term:`HIL` simulation on a :term:`Pixhawk` :term:`FMU`. This
example uses the `NXP FMUK66-E board`_ as an example but any `PX4 supported board`_
should work.

.. _NXP FMUK66-E board: https://docs.px4.io/main/en/flight_controller/nxp_rddrone_fmuk66.html
.. _PX4 supported board: https://px4.io/autopilots/

.. warning::
    Keep the propellers **off** your drone throughout the HIL simulation.

Prerequisites
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. include:: ../launch/_prerequisites_gisnav.rst

Connect Jetson Nano and Pixhawk
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In this example we will power the Pixhawk from the development computer via USB
and the Jetson Nano from a wall socket via a DC adapter to avoid having to handle
LiPo batteries. In a more realistic setup you would supply power to both boards
from the :term:`onboard` battery.

Follow the below steps and diagram to setup your HIL simulation hardware:

* **Install a bootloader on your Pixhawk board if your board does not yet have
  one.** See your board manufacturer's instructions on how to load one onto your
  specific board.
* Connect your Jetson Nano to your development computer via Ethernet cable
  (see :ref:`Jetson Nano SITL` for more information).
* Connect your Jetson Nano to a wall socket using a micro-USB power adapter.
* Connect your Pixhawk board to your development computer via micro-USB cable.
* Connect your Pixhawk board to your Jetson Nano via TELEM1 (set PX4
  ``XRCE_DDS_0_CFG`` parameter value to ``101``) using a USB to UART converter.

.. tab-set::

    .. tab-item:: Diagram
        :selected:

        .. mermaid::

          graph TB
            subgraph "FMUK66-E (FMU)"
                subgraph "TELEM1"
                    FMU_TELEM1_RX[RX]
                    FMU_TELEM1_TX[TX]
                    FMU_TELEM1_GND[GND]
                end
                FMU_USB[micro-USB Port]
            end
            subgraph "Development computer"
                Laptop_ETH[Ethernet Port]
                Laptop_USB[USB Port]
            end
            subgraph "Jetson Nano"
                Nano_USB[USB Port x4]
                Nano_micro_USB[Micro USB Port]
                Nano_HDMI[HDMI Port]
                Nano_ETH[Ethernet]
            end
            subgraph "USB to UART Converter"
                Converter_RX[RX]
                Converter_TX[TX]
                Converter_GND[GND]
                Converter_USB[USB]
            end
            Socket[Wall Socket]
            subgraph "Optional (can also use RDP or VNC)"
                Display[External Display]
                Mouse[USB Mouse]
                Keyboard[USB Keyboard]
            end
            FMU_TELEM1_TX ---|To UART RX| Converter_RX
            FMU_TELEM1_RX ---|To UART TX| Converter_TX
            FMU_TELEM1_GND ---|To UART GND| Converter_GND
            FMU_USB ---|To Dev Computer USB| Laptop_USB
            Converter_USB ---|To Nano USB| Nano_USB
            Nano_micro_USB ---|Micro USB Power| Socket
            Nano_HDMI ---|HDMI| Display
            Nano_USB ---|USB| Mouse
            Nano_USB ---|USB| Keyboard
            Nano_ETH ---|To Dev Computer ETH| Laptop_ETH

    .. tab-item:: Picture

        .. figure:: ../../../_static/img/gisnav_hil_fmuk66-e_setup.jpg

        * NXP FMUK66-E (FMU) board connected to laptop via micro-USB and to Jetson
          Nano via TELEM1.

        * FMU draws power from laptop via micro-USB, and Jetson Nano from wall
          socket via dedicated micro-USB DC adapter, so no LiPo batteries needed.

        * Connection from FMU to Jetson Nano via TELEM1 serial port using USB to
          UART converter. See `FMUK66-E revision C pin layout`_ for how to wire
          the TELEM1 JST-GH connector (only GND, RX and TX used here).

        * Other wires as per `manufacturer's instructions`_, except for missing
          telemetry radio.

        .. note::
            The TX from one board connects to the RX of the other board and vice
            versa.

        .. _manufacturer's instructions: https://nxp.gitbook.io/hovergames/userguide/assembly/connecting-all-fmu-wires
        .. _FMUK66-E revision C pin layout: https://nxp.gitbook.io/hovergames/rddrone-fmuk66/connectors/telemetry-1


Install QEMU emulators
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Install `QEMU`_ emulators on your Jetson Nano to make ``linux/amd64`` images
run on the ``linux/arm64`` Jetson Nano:

.. code-block:: bash
    :caption: Install QEMU emulators

     docker run --privileged --rm tonistiigi/binfmt --install all

.. _QEMU: https://docs.docker.com/build/building/multi-platform/#building-multi-platform-images

Upload PX4 firmware
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
See the `PX4 uploading firmware instructions`_ for how to upload your development version of PX4 onto your Pixhawk
board. To find the ``make`` target for your specific board, list all options with the ``make list_config_targets``
command:

.. _PX4 uploading firmware instructions: https://docs.px4.io/main/en/dev_setup/building_px4.html#uploading-firmware-flashing-the-board

.. code-block:: bash
    :caption: List PX4 supported FMU boards

    cd ~/colcon_ws/src/gisnav/docker
    export COMPOSE_PROJECT_NAME=gisnav
    docker compose run px4 make list_config_targets

Then choose your appropriate board for the following examples. We are going to
choose ``nxp_fmuk66-e_default`` for this example:

.. code-block:: bash
    :caption: Upload PX4 firmware to FMU

    export COMPOSE_PROJECT_NAME=gisnav
    #docker compose run px4 git submodule update --recursive
    docker compose run px4 make distclean
    docker compose run px4 make nxp_fmuk66-e_default upload

Deploy offboard services
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Follow the steps in `PX4 HIL simulation instructions`_ to deploy the :term:`offboard`
services. These example commands should match the instructions:

.. _PX4 HIL simulation instructions: https://docs.px4.io/main/en/simulation/hitl.html

.. code-block:: bash
    :caption: Deploy HIL simulation offboard services

    export COMPOSE_PROJECT_NAME=gisnav
    #docker compose run px4 make clean
    docker compose run px4 DONT_RUN=1 make px4_sitl gazebo___ksql_airport
    docker compose run px4 source Tools/simulation/gazebo/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
    docker compose run px4 gazebo Tools/simulation/gazebo/sitl_gazebo/worlds/hitl_iris_ksql_airport.world

    # Important: Start QGroundControl last
    docker compose up qgc

After deploying the HIL simulation, adjust the settings via the :term:`QGC`
application as follows:

* Precisely match the ``COM_RC_IN_MODE`` parameter setting if mentioned in the
  instructions.
* Ensure that you have HITL enabled in QGC Safety settings.
* Ensure you have the virtual joystick enabled in QGC General settings.

Deploy onboard services
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Once you have the HIL simulation running, login to your Jetson Nano and deploy
the onboard services:

.. code-block:: bash
    :caption: Deploy HIL simulation onboard services

    mkdir -p ~/colcon_ws/src
    cd ~/colcon_ws/src
    git clone https://github.com/hmakelin/gisnav.git
    cd ~/colcon_ws/src/gisnav
    make -C docker up-onboard-hil-px4
