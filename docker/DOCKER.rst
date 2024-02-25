Docker build contexts
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

In the ``docker/`` folder you can find a collection of directories roughly
corresponding to :term:`Docker` build contexts, and a number of
:term:`Docker Compose` files defining services that use these build contexts. In
case of multi-stage builds multiple service images can be created from the same
build context. The ``docker/Makefile`` may define additional phony targets for
commonly needed tasks such as exposing the X server to containers that have a
:term:`GUI` component.

External interfaces
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
    * Split mavlink network into mavlink and ROS networks. For ROS the intention
      is to use the shared memory device instead of serializing and going through
      the network stack since we will be passing a lot of images around.
    * Build and expose static docs to home page - possibly no need for a server

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
            end
        end

        subgraph admin ["admin"]
            homepage[homepage]
            fileserver[fileserver]
        end

        subgraph volumes ["User managed shared volumes"]
            gscam_volume[gscam_volume]
            gis_maps_volume[maps-volume]
            application_gisnav_volume[gisnav-volume]
        end
        application_docs_volume[docs-volume]

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
        application_gisnav ---|/opt/colcon_ws/install/gisnav/share/gisnav/launch/params| application_gisnav_volume
        application_docs_volume ---|/path/to/built/docs| application_gisnav
        homepage ---|3000/tcp| fileserver
        fileserver ---|"/var/www/filegator/"| volumes
        gscam_volume ---|/etc/gscam| middleware_gscam

        application_docs_volume ---|/path/to/docs:ro| homepage

        subgraph host ["host"]
            monitoring["monitoring"]
            docker_host["docker host"]
        end

        monitoring ---|61208/tcp| homepage
        docker_host ---|/var/run/docker.sock| monitoring

        classDef network fill:transparent,stroke-dasharray:5 5;
        class mavlink,gis,gis_mavlink,admin,admin_gis,host network
