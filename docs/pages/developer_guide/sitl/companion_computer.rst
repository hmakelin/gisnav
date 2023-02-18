Onboard computer
____________________________________________________
This section contains instructions on how to take advantage of Docker composable services to run ``gisnav`` and a
GIS server on an onboard computer while the SITL simulation itself runs on a more powerful (desktop) computer to get a
better idea of performance in a real use case.

Jetson Nano
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

These instructions assume that your Jetson Nano is connected to the same network as your desktop computer. This
could be done e.g. by connecting them to the same WiFi (need dongle for Jetson Nano), or by connecting them by
Ethernet cable. Example instructions with screenshots for both WiFi and Ethernet cable can be found through the
following link:

* https://www.forecr.io/blogs/connectivity/how-to-share-internet-from-computer-to-nvidia-jetson-modules

See also the below screenshot:

 .. figure:: ../../../_static/img/gisnav_hil_jetson_nano_setup.jpg

    Jetson Nano connected to laptop via micro-USB and Ethernet. Power supply from wall socket.

Log into your desktop computer and build and run the services required for the SITL simulation:

.. code-block:: bash
    :caption: Run Gazebo SITL simulation on desktop

    cd ~/colcon_ws/src/gisnav
    make -C docker build-offboard-sitl-px4
    make -C docker up-offboard-sitl-px4

Then log into your Jetson Nano and build and run the onboard services:

.. code-block:: bash
    :caption: Run GISNav and GIS server on onboard computer

    cd ~/colcon_ws/src/gisnav
    make -C build-companion-sitl-px4
    make -C up-companion-sitl-px4

You should now have the SITL simulation and QGgroundControl running on your offboard workstation, while ``gisnav``,
``mapserver`` and the autopilot specific middleware run on your Jetson Nano. If you have your network setup correctly,
the middleware on the onboard companion computer will connect to the simulated autopilot on your workstation and pipe
the telemetry and video feed to ROS for GISNav to consume.
