Deploy with Docker Compose
____________________________________________________

GISNav uses :term:`Docker Compose` to define the services that constitute its
different deployment configurations. The services are
:ref:`orchestrated by a Makefile <Deploy using Makefile>` for convenience and
for increased ease of adoption.

This page describes how these services are built and deployed individually to
help you customize GISNav's deployments beyond what the Makefile offers.

Prerequisites
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. include:: ../_shared/prerequisites/docker.rst

.. include:: ../_shared/prerequisites/gisnav.rst

.. include:: ../_shared/prerequisites/compose_project_name_env_variable.rst

List of services
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The ``docker/docker-compose.yaml`` file defines all services used to support
GISNav deployments. Provided below is a list of the service names along with
a brief description of their intended use.

.. dropdown:: See YAML source code
    :icon: code

    .. literalinclude:: ../../../../docker/docker-compose.yaml
        :caption: Docker Compose base services
        :language: yaml

+---------------------+-----------------------------------------------------------------------------------------------+
| Service             | Description                                                                                   |
+=====================+===============================================================================================+
| ``ardupilot``       | :term:`ArduPilot` :term:`Gazebo` :term:`SITL` simulation. Starts the ``gazebo-iris`` model    |
|                     | with added static down (:term:`FRD`) facing :term:`camera` at the :term:`KSQL` Airport.       |
+---------------------+-----------------------------------------------------------------------------------------------+
| ``px4``             | :term:`PX4` Gazebo SITL simulation. Starts the ``typhoon_h480`` model at the KSQL Airport.    |
+---------------------+-----------------------------------------------------------------------------------------------+
| ``mavros``          | :term:`MAVROS` middleware. Used as autopilot middleware with both PX4 and ArduPilot.          |
+---------------------+-----------------------------------------------------------------------------------------------+
| ``micro-ros-agent`` | :term:`Micro-ROS agent` middleware. Used for PX4 SITL for outgoing :class:`SensorGps`         |
|                     | :term:`messages`.                                                                             |
+---------------------+-----------------------------------------------------------------------------------------------+
| ``qgc``             | :term:`QGroundControl` ground control software for controlling the :term:`vehicle`.           |
+---------------------+-----------------------------------------------------------------------------------------------+
| ``mapserver``       | :term:`GIS` server with self-hosted :term:`NAIP` and :term:`OSM` Buildings data covering      |
|                     | KSQL airport.                                                                                 |
+---------------------+-----------------------------------------------------------------------------------------------+
| ``autoheal``        | Monitors :term:`Docker` container health and restarts containers marked as unhealthy. Used in |
|                     | the :term:`onboard` :term:`HIL` deployment configuration.                                     |
+---------------------+-----------------------------------------------------------------------------------------------+
| ``gscam``           | Bridge for integrating video stream from camera into :term:`ROS` via :term:`GStreamer`.       |
+---------------------+-----------------------------------------------------------------------------------------------+
| ``qgis``            | :term:`QGIS` client :term:`GUI` preconfigured with ``mapserver`` WMS connection for           |
|                     | inspecting and managing onboard maps.                                                         |
+---------------------+-----------------------------------------------------------------------------------------------+
| ``gisnav``          | GISNav :term:`ROS 2` package for demonstration use only. Launches GISNav with the PX4         |
|                     | configuration by default. Can also be launched for ArduPilot.                                 |
+---------------------+-----------------------------------------------------------------------------------------------+
| ``postgres``        | :term:`PostGIS` server for as backend to ``mapserver`` and :class:`.QGISNode`.                |
+---------------------+-----------------------------------------------------------------------------------------------+

External interfaces
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The below diagram describes the system architecture through the external
interfaces between the Docker Compose services:

.. mermaid::

    graph BT
        subgraph ground_control_layer["Ground Control Layer"]
            qgc[QGC]
        end

        subgraph autopilot_layer["Autopilot Layer"]
            px4[PX4]
            ardupilot[ArduPilot]
        end

        subgraph middleware_layer["Middleware Layer"]
            micro_ros_agent[Micro-ROS-Agent]
            mavros[MAVROS]
            gscam[gscam]
        end

        subgraph application_layer["Application Layer"]
            gisnav[GISNav]
            rviz[RViz]
            autoheal[Autoheal]
        end

        subgraph gis_layer["GIS Layer"]
            mapserver[Mapserver]
            qgis[QGIS]
        end

        subgraph data_layer["Data Layer"]
            postgres[Postgres]
        end

        postgres -->|Database Connection| mapserver
        postgres -->|Database Connection| qgis

        mapserver -.-> gisnav
        gisnav -->|WMS| mapserver
        gisnav -->|ROS Connection| mavros
        gisnav -->|ROS Connection| micro_ros_agent
        gisnav -->|ROS Connection| gscam

        mavros -->|Mavlink| ardupilot
        mavros -->|Mavlink| px4
        micro_ros_agent -->|"TCP UORB Bridge"| px4
        px4 -->|Mavlink| qgc
        ardupilot -->|Mavlink| qgc

Example deployments
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Mock GPS demo
********************************************

To deploy the mock :term:`GPS` demonstration introduced on the :ref:`Get Started`
page locally, ensure your development system satisfies all the :ref:`prerequisites
<Prerequisites>` and then follow the below steps to create, start, and shutdown
the required containers.


.. code-block:: bash
    :caption: Build images and create containers

    cd ~/colcon_ws/src/gisnav/docker
    docker compose -p gisnav create --build \
        mapserver \
        micro-ros-agent \
        mavros \
        qgc \
        px4 \
        gisnav


.. include:: ../_shared/expose_xhost.rst

.. code-block:: bash
    :caption: Start containers

    cd ~/colcon_ws/src/gisnav/docker
    docker compose -p gisnav start \
        mapserver \
        micro-ros-agent \
        mavros \
        qgc \
        px4 \
        gisnav


.. include:: ../_shared/docker_compose_shutdown.rst

Local development
********************************************

When deploying for local development, the difference to
:ref:`deploying the Get Started demonstration <Deploy demonstration>` is that
we do not include the ``gisnav`` service which is assumed to be
:ref:`launched separately from a local development version <Launch GISNav>`:

.. code-block:: bash
    :caption: Build images and create containers

    cd ~/colcon_ws/src/gisnav/docker
    docker compose create --build \
        mapserver \
        micro-ros-agent \
        mavros \
        qgc \
        rviz \
        px4 \
        qgis \
        postgres

.. include:: ../_shared/expose_xhost.rst

.. code-block:: bash
    :caption: Start containers

    cd ~/colcon_ws/src/gisnav/docker
    docker compose start \
        mapserver \
        micro-ros-agent \
        mavros \
        qgc \
        rviz \
        px4 \
        qgis \
        postgres

After you have your supporting services deployed you would typically
:ref:`use the ROS 2 launch system <Use ROS 2 launch system>` to launch your
locally installed development version of GISNav:

.. include:: ../_shared/launch_gisnav_with_ros2_launch.rst


.. include:: ../_shared/docker_compose_shutdown.rst
