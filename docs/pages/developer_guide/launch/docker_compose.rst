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

.. include:: _prerequisites_docker.rst

.. include:: _prerequisites_gisnav.rst

.. include:: _prerequisites_compose_project_name_env_variable.rst

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
|                     | with added static down (:term:`FRD`) facing :term:`camera` at the KSQL Airport.               |
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
| ``torch-serve``     | :term:`Torch` based deep learning service that handles image matching as part of :term:`pose` |
|                     | estimation.                                                                                   |
+---------------------+-----------------------------------------------------------------------------------------------+
| ``mapserver``       | :term:`GIS` server with self-hosted :term:`NAIP` and :term:`OSM` Buildings data covering      |
|                     | KSQL airport.                                                                                 |
+---------------------+-----------------------------------------------------------------------------------------------+
| ``autoheal``        | Monitors Docker container health and restarts containers marked as unhealthy. Used in the     |
|                     | :term:`onboard` :term:`HIL` deployment configuration.                                         |
+---------------------+-----------------------------------------------------------------------------------------------+
| ``gisnav``          | GISNav :term:`ROS 2` package for demonstration use only. Launches GISNav with the PX4         |
|                     | configuration by default. Can also be launched for ArduPilot.                                 |
+---------------------+-----------------------------------------------------------------------------------------------+

Deploy demonstration
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To deploy the mock :term:`GPS` demonstration introduced on the :ref:`Get Started`
page locally, ensure you have the :ref:`prerequisites <Prerequisites>` installed and
then follow the below steps to create, deploy, and shutdown the required services.

Build and create
***********

.. include:: _build_and_create_docker_intro.rst

.. code-block:: bash
    :caption: Build images and create containers

    cd ~/colcon_ws/src/gisnav/docker
    docker compose create --build \
        mapserver \
        torch-serve \
        micro-ros-agent \
        mavros \
        qgc \
        rviz \
        px4 \
        gisnav

Expose xhost
***********

.. include:: _expose_xhost.rst

Deploy
***********

.. include:: _deploy_docker_intro.rst

.. tab-set::

    .. tab-item:: Foreground
        :selected:

        .. code-block:: bash
            :caption: Deploy containers

            cd ~/colcon_ws/src/gisnav/docker
            docker compose up \
                mapserver \
                torch-serve \
                micro-ros-agent \
                mavros \
                qgc \
                rviz \
                px4 \
                gisnav

    .. tab-item:: Detached

        .. code-block:: bash
            :caption: Run demo services

            cd ~/colcon_ws/src/gisnav/docker
            docker compose up -d \
                mapserver \
                torch-serve \
                micro-ros-agent \
                mavros \
                qgc \
                rviz \
                px4 \
                gisnav

Shutdown
***********

.. include:: _docker_compose_shutdown.rst


Deploy for local development
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Build and create
***********

.. include:: _build_and_create_docker_intro.rst

When deploying for local development, the difference to
:ref:`deploying the Get Started demonstration <Deploy demonstration>` is that
we do not include the ``gisnav`` service which is assumed to be
:ref:`launched separately from a local development version <Launch GISNav>`:

.. code-block:: bash
    :caption: Build images and create containers

    cd ~/colcon_ws/src/gisnav/docker
    docker compose create --build \
        mapserver \
        torch-serve \
        micro-ros-agent \
        mavros \
        qgc \
        rviz \
        px4

Expose xhost
***********

.. include:: _expose_xhost.rst

Deploy
***********

.. include:: _deploy_docker_intro.rst

.. tab-set::

    .. tab-item:: Foreground
        :selected:

        .. code-block:: bash
            :caption: Deploy containers

            cd ~/colcon_ws/src/gisnav/docker
            docker compose up \
                mapserver \
                torch-serve \
                micro-ros-agent \
                mavros \
                qgc \
                rviz \
                px4

    .. tab-item:: Detached

        .. code-block:: bash
            :caption: Run demo services

            cd ~/colcon_ws/src/gisnav/docker
            docker compose up -d \
                mapserver \
                torch-serve \
                micro-ros-agent \
                mavros \
                qgc \
                rviz \
                px4

Launch GISNav
***********

After you have your supporting services deployed you would typically
:ref:`use the ROS 2 launch system <Use ROS 2 launch system>` to launch your
locally installed development version of GISNav:

.. include:: _launch_gisnav_with_ros2_launch.rst

Shutdown
***********

.. include:: _docker_compose_shutdown.rst
