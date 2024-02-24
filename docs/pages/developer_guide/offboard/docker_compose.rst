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


.. include:: ../../../../docker/DOCKER.rst


Overview of services
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The |Docker Compose file|_ defines all services used to support GISNav deployments.
The diagram below describes the system architecture through the external
interfaces and Docker bridge networks shared between the Docker Compose services.

The Docker bridge networks have in-built DNS which means the container names
resolve to their respective IP addresses. The container name will equal the
service name prefixed with ``gisnav-`` and suffixed with ``-1``. So for example
deploying the ``mapserver`` service using the Compose file will start a
Docker container with the hostname ``gisnav-mapserver-1``.

.. todo::
    Split mavlink network into mavlink and ROS networks. For ROS the intention
    is to use the shared memory device instead of serializing and going through
    the network stack since we will be passing a lot of images around.

.. note::
    The application services have access to both ``gis`` and ``mavlink`` networks.

.. mermaid::

    graph TD
        subgraph mavlink ["mavlink"]
            mavlink_qgc[qgc]
            subgraph simulation ["Simulation Services"]
                simulation_px4[px4]
                simulation_ardupilot[ardupilot]
            end
            subgraph middleware ["Middleware Services"]
                middleware_mavros[mavros]
                middleware_micro_ros_agent[micro-ros-agent]
                middleware_gscam[gscam]
            end
        end

        subgraph gis_mavlink ["gis & mavlink"]
            subgraph application ["Application Services"]
                application_gisnav[gisnav]
                application_autoheal[autoheal]
            end
        end

        subgraph gis ["gis"]
            subgraph gis_services ["GIS Services"]
                gis_mapserver[mapserver]
                gis_qgis[qgis]
            end
            subgraph data_services ["Data Services"]
                gis_postgres[postgres]
                gis_maps_volume[maps-volume]
            end
        end

        subgraph admin ["admin"]
            homepage[homepage]
        end

        subgraph admin_gis ["admin & gis"]
            fileserver[fileserver]
        end

        mavlink_qgc -->|14550/udp\nMAVLink| simulation_px4
        simulation_px4 -->|14540/udp\nMAVLink| middleware_mavros
        simulation_px4 -->|8888/udp\nDDS-XRCE | middleware_micro_ros_agent
        simulation_px4 -->|5600/udp\nRTP H.264 Video| middleware_gscam
        middleware_mavros -->|/dev/shm\nROS 2 Fast DDS| application_gisnav
        middleware_micro_ros_agent -->|/dev/shm\nROS 2 Fast DDS| application_gisnav
        middleware_gscam -->|/dev/shm\nROS 2 Fast DDS| application_gisnav
        application_gisnav -->|5432/tcp| gis_postgres

        application_gisnav -->|80/tcp\nHTTP WMS| gis_mapserver
        gis_mapserver -->|80/tcp\nHTTP WMS| gis_qgis
        gis_qgis -->|5432/tcp| gis_postgres
        gis_mapserver ---|/etc/mapserver| gis_maps_volume
        fileserver ---|/etc/mapserver/maps| gis_maps_volume

        homepage ---|TCP| fileserver

        classDef network fill:transparent,stroke-dasharray:5 5;
        class mavlink,gis,gis_mavlink,admin,admin_gis network


Example deployments
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The interdependencies between different services are hard-coded into the
|Docker Compose file|_ and typically you will need to explicitly start
only a few services unless you also want to start optional services.

Mock GPS demo
********************************************

To deploy the mock :term:`GPS` demonstration introduced on the :ref:`Get Started`
page locally, ensure your development system satisfies all the :ref:`prerequisites
<Prerequisites>` and then follow the below steps to create, start, and shutdown
the required containers.


.. code-block:: bash
    :caption: Build images and create containers

    cd ~/colcon_ws/src/gisnav/docker
    docker compose -p gisnav create --build gisnav

.. include:: ../_shared/expose_xhost.rst

.. code-block:: bash
    :caption: Start containers

    cd ~/colcon_ws/src/gisnav/docker
    docker compose -p gisnav start gisnav

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
        px4 \
        rviz \
        qgis

.. include:: ../_shared/expose_xhost.rst

.. code-block:: bash
    :caption: Start containers

    cd ~/colcon_ws/src/gisnav/docker
    docker compose start \
        px4 \
        rviz \
        qgis

After you have your supporting services deployed you would typically
:ref:`use the ROS 2 launch system <Use ROS 2 launch system>` to launch your
locally installed development version of GISNav:

.. include:: ../_shared/launch_gisnav_with_ros2_launch.rst

.. include:: ../_shared/docker_compose_shutdown.rst
