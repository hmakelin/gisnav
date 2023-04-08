Docker Compose
===================================

The ``docker`` folder contains scripts for generating SITL simulation environments for development, testing, and
demonstration purposes.

The ``docker-compose.yaml`` file defines the following services:

+---------------------+-----------------------------------------------------------------------------------------------+
| Service             | Description                                                                                   |
+=====================+===============================================================================================+
| ``ardupilot``       | ArduPilot Gazebo SITL simulation. Starts the `gazebo-iris` model with added static down       |
|                     | (FRD frame) facing camera at the KSQL Airport.                                                |
+---------------------+-----------------------------------------------------------------------------------------------+
| ``px4``             | PX4 Gazebo SITL simulation. Starts `typhoon_h480` model at the KSQL Airport.                  |
+---------------------+-----------------------------------------------------------------------------------------------+
| ``mavros``          | MAVROS. Used for PX4 and ArduPilot SITL.                                                              |
+---------------------+-----------------------------------------------------------------------------------------------+
| ``micro-ros-agent`` | Micro-ROS agent. Used for PX4 SITL.                                                           |
+---------------------+-----------------------------------------------------------------------------------------------+
| ``qgc``             | QGroundControl ground control software for controlling simulated drones.                      |
+---------------------+-----------------------------------------------------------------------------------------------+
| ``torch-serve``     | Deep-learning service that handles image matching.                                            |
+---------------------+-----------------------------------------------------------------------------------------------+
| ``mapserver``       | WMS server with self-hosted NAIP and OSM Buildings data, covering KSQL airport.               |
+---------------------+-----------------------------------------------------------------------------------------------+
| ``mapproxy``        | WMS proxy for existing remote tile-based imagery endpoint. Alternative for `mapserver` when   |
|                     | an imagery layer needs to cover multiple flight regions.                                      |
+---------------------+-----------------------------------------------------------------------------------------------+
| ``gisnav``          | GISNav ROS 2 package for demo purposes. Launches GISNav with the PX4 configuration by default.|
|                     | Intended to be used with the `px4` service. Can be launched for `ardupilot`.                  |
+---------------------+-----------------------------------------------------------------------------------------------+

A number of Docker Compose overrides are also included in the folder. They are
used by the ``Makefile`` described in :ref:`Service orchestration`.

Prerequisites
-------------
You must install `Docker`_ and the `Docker Compose plugin`_ to run the following
example commands. If you have an NVIDIA GPU on your host machine, ensure you
have `NVIDIA Container Toolkit installed`_.

.. _Docker: https://docs.docker.com/engine/install/
.. _Docker Compose plugin: https://docs.docker.com/compose/install/linux/
.. _NVIDIA Container Toolkit installed: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html

PX4 SITL (Mock GPS Demo)
------------------------

Follow these instructions to launch the SITL simulation used in the
`mock GPS demo`_.

.. _mock GPS demo: https://github.com/hmakelin/gisnav/blob/master/README.md#mock-gps-example

.. note::
    Run the below commands without the ``gisnav`` service if you want to
    develop or test a local copy of GISNav.


Build
^^^^^^^^^^^^^^^^^^^

To build the ``mapserver``, ``px4``,  ``micro-ros-agent``,  ``mavros``,
``torch-serve`` and ``gisnav`` services, run the following command:

.. code-block:: bash

    docker compose build mapserver px4 micro-ros-agent torch-serve qgc gisnav


Run
^^^^^^^^^^^^^^^^^^^^

Run the PX4 SITL simulation with GISNav:

.. code-block:: bash

    docker compose up mapserver px4 micro-ros-agent mavros torch-serve qgc gisnav


Shutdown
^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

    docker compose down


ArduPilot SITL
---------------
Build and run the SITL simulation environment with ArduPilot instead of PX4.

Build
^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

    docker compose \
      -f docker-compose.yaml \
      -f docker-compose.gisnav-ardupilot.yaml \
      build mapserver ardupilot mavros qgc torch-serve gisnav


Run
^^^^^^^^^^^^^^^^^^^^

.. code-block:: bash

    docker compose \
      -f docker-compose.yaml \
      -f docker-compose.gisnav-ardupilot.yaml \
      up mapserver ardupilot mavros qgc torch-serve gisnav


Mapproxy
--------
Run the SITL simulation with a WMS proxy instead of locally hosted maps.

.. note::

    Replace the example ``MAPPROXY_TILE_URL`` string below with your tile-based
    endpoint URL (e.g. WMTS). See `MapProxy configuration examples`_ for more
    information on how to format the string.

    .. _MapProxy configuration examples: https://mapproxy.org/docs/latest/configuration_examples.html


.. code-block:: bash

    docker compose build \
      --build-arg MAPPROXY_TILE_URL="https://<your-map-server-url>/tiles/%(z)s/%(y)s/%(x)s" \
      mapproxy px4 micro-ros-agent gisnav qgc torch-serve gisnav
    docker compose up mapproxy px4 micro-ros-agent qgc torch-serve gisnav


Troubleshooting
---------------

Expose ``xhost``
^^^^^^^^^^^^^^^^^^^^

If the Gazebo and QGroundControl windows do not appear on your screen soon after running your container, you may need to
expose your ``xhost`` to your Docker container. Refer to the `ROS GUI Tutorial`_ for details.

.. _ROS GUI Tutorial: http://wiki.ros.org/docker/Tutorials/GUI

.. code-block:: bash

    export containerId=$(docker ps -l -q)
    xhost +local:$(docker inspect --format='{{ .Config.Hostname }}' $containerId)


Headless mode
^^^^^^^^^^^^^^^^^^^^

You may want to run Gazebo in headless mode when doing automated testing (e.g., with mavsdk).

.. code-block:: bash
micro
    docker compose -f docker-compose.headless.yaml up px4


Disable SharedMemory for Fast DDS
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you are not able to establish ROS communication between the ``mavros`` or
``micro-ros-agent`` container and the host, or receive the above error when
using ``--network host``, try disabling SharedMemory for Fast DDS
**on your host**. You can do so by creating an XML configuration (e.g.,
``disable_shared_memory.xml``) as described in `this comment`_
or discussion `here`_ and restarting ROS 2 daemon with the new configuration:

.. _this comment: https://github.com/eProsima/Fast-DDS/issues/1698#issuecomment-778039676
.. _here: https://stackoverflow.com/questions/65900201/troubles-communicating-with-ros2-node-in-docker-container

.. code-block:: bash

    export FASTRTPS_DEFAULT_PROFILES_FILE=disable_fastrtps.xml
    ros2 daemon stop
    ros2 daemon start

Disable AppArmor for ArduPilot SITL
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Possibly needed if using ``--network host``: If QGroundControl or Gazebo do
not seem to be starting when running the containers, you may need to run them
image with ``--security-opt apparmor:unconfined`` or ``--privileged`` options.

Run shell inside container
^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you need to do debugging on the images with GUI applications enabled (e.g.,
Gazebo inside ``px4``), run bash inside the container using the following command:

.. code-block:: bash

    docker run -it \
      --env="DISPLAY" \
      --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
      --volume "/dev/shm:/dev/shm" \
      --volume="/dev/dri:/dev/dri" \
      --gpus all \
      --tty \
      --network host \
      --entrypoint="/bin/bash" \
      gisnav
