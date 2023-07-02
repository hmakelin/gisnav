Jetson Nano SITL
____________________________________________________

This page contains instructions on how to to run a GISNav on an :term:`onboard`
:term:`Jetson Nano` computer while a :term:`SITL` simulation runs on a development
computer to get a better idea of performance in a real use case.

Connect to your Jetson Nano
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

These instructions assume that your Jetson Nano is connected to the same network
as your desktop computer. This could be done e.g. by connecting them to the
same WiFi (need dongle for Jetson Nano), or by connecting them by Ethernet cable.

For connecting via Ethernet cable, see the below example. The below example is
compatible with the :ref:`Pixhawk HIL` example that exapnds on it by adding an
:term:`FMU`.

.. tab-set::

    .. tab-item:: Diagram
        :selected:

        .. mermaid::

          graph TB
            subgraph "Laptop"
                Laptop_ETH[Ethernet Port]
            end
            subgraph "Jetson Nano"
                Nano_USB_1[USB Port]
                Nano_USB_2[USB Port]
                Nano_micro_USB[Micro USB Port]
                Nano_HDMI[HDMI Port]
                Nano_ETH[Ethernet]
            end
            Socket[Wall Socket]
            Display[External Display]
            Mouse[USB Mouse]
            Keyboard[USB Keyboard]
            Nano_micro_USB ---|Micro USB Power| Socket
            Nano_HDMI ---|HDMI| Display
            Nano_USB_1 ---|USB| Mouse
            Nano_USB_2 ---|USB| Keyboard
            Nano_ETH ---|To Laptop ETH| Laptop_ETH

    .. tab-item:: Picture

         .. figure:: ../../../_static/img/gisnav_hil_jetson_nano_setup.jpg

            Jetson Nano connected to laptop via micro-USB and Ethernet. Power supply from wall socket.

.. seealso::
    Example instructions with screenshots for connecting via both WiFi and Ethernet
    cable can also be found e.g. `here <https://www.forecr.io/blogs/connectivity/how-to-share-internet-from-computer-to-nvidia-jetson-modules>`_.

Deploy offboard services
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Log into your development computer and deploy the services required for the
SITL simulation:

.. code-block:: bash
    :caption: Deploy Gazebo SITL simulation on development computer

    cd ~/colcon_ws/src/gisnav
    make -C docker up-offboard-sitl-px4

Deploy onboard services
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Then log into your Jetson Nano and install `QEMU`_ emulators to make the
``linux/amd64`` images run on the ``linux/arm64`` Jetson Nano:

.. code-block:: bash
    :caption: Enable running ``amd64`` images on ``arm64`` architecture

     docker run --privileged --rm tonistiigi/binfmt --install all

.. _QEMU: https://docs.docker.com/build/building/multi-platform/#building-multi-platform-images

Then deploy the onboard services on the Jetson Nano:

.. code-block:: bash
    :caption: Run GISNav and GIS server on onboard computer

    cd ~/colcon_ws/src/gisnav
    make -C docker up-onboard-sitl-px4

You should now have the SITL simulation and QGgroundControl running on your
offboard development computer, while ``gisnav``, ``mapserver``, ``autoheal``,
and the autopilot specific middleware (``micro-ros-agent`` and/or ``mavros``)
run on your Jetson Nano. If you have your network setup correctly, the middleware
on the Jetson Nano will connect to the simulated autopilot on your development
computer and receive the needed ROS messages for GISNav to consume.
