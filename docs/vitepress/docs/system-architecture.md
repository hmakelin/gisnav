# System architecture

The GISNav ROS 2 package receives upstream inputs from the autopilot via MAVLink and transforms them in multiple sequential steps to downstream outputs, the most important of which are the mock GPS messages. In GISNav this data processing algorithm is expressed as a distributed unidirectional network of ROS nodes.

The GISNav ROS application also has many external interfaces and effective development requires a simulation environment. GISNav defines and [deploys that simulation environment with Docker Compose](/deploy-with-docker-compose).

This page provides an overview of both the topography of the GISNav ROS 2 package itself as well as how these different Docker Compose services that consitute the simulation environment relate to each other.

## ROS topography

The core ROS topography diagram below depicts how ROS messages flow through GISNav. The [API reference](/reference/) has more detailed information on the purpose and design of each ROS node.

::: info Todo
- From BBoxNode, publish map to `base_link` and `base_link` to `camera` transformations separately to simplify implementation and reduce amount of maintained code.
- Implement REP 105 properly (currently only partially implemented).
- Include default topic names in diagram.
:::

```mermaid
graph TB

    subgraph ros_middleware["ROS middleware"]
        MAVROS
        gscam
    end

    MAVROS -->|"sensor_msgs/NavSatFix"| BBoxNode
    MAVROS -->|"geometry_msgs/PoseStamped"| BBoxNode
    MAVROS -->|"mavros_msgs/GimbalDeviceAttitudeStatus"| BBoxNode
    gscam ---->|"sensor_msgs/Image"| BBoxNode

    subgraph core["GISNav core nodes"]
        BBoxNode -->|"geographic_msgs/BoundingBox"| GISNode
        GISNode -->|"gisnav_msgs/OrthoImage"| StereoNode
        StereoNode -->|"gisnav_msgs/MonocularStereoImage"| PoseNode
        StereoNode -->|"gisnav_msgs/OrthoStereoImage"| PoseNode
    end

    subgraph extension["GISNav extension nodes"]
        NMEANode["NMEANode"]
        UORBNode["UORBNode"]
    end

    subgraph robot_localization["robot_localization ROS package"]
        ekf["ekf_localization_node"] -->|"nav_msgs/Odometry"| UORBNode
        ekf["ekf_localization_node"] -->|"nav_msgs/Odometry"| NMEANode
    end

    PoseNode -->|"geometry_msgs/PoseWithCovarianceStamped"| ekf
    UORBNode -->|"px4_msgs/SensorGps"| micro_ros_agent[" "]

    style micro_ros_agent fill-opacity:0, stroke-opacity:0;

```

## Service architecture

The `docker/docker-compose.yaml` file defines the Docker Compose services that support GISNav deployments. The diagram below describes the system architecture through the external interfaces between the services.

### Docker build contexts

In the `docker/` folder, you can find a collection of directories roughly corresponding to Docker build contexts, and a number of Docker Compose files defining services that use these build contexts. In case of multi-stage builds, multiple service images can be created from the same build context. The `docker/Makefile` may define additional phony targets for commonly needed tasks such as exposing the X server to containers that have a GUI component.

It is important to share layers to keep the overall size of the system down.

::: info Todo
- Update simulation services to use `ubuntu:jammy` and `ros:humble` instead of the older `ubuntu:focal` and `ros:foxy`.
- Ensure we use the same ubuntu base image for `glances` (and not e.g. alpine)
- Update `qgc` from `focal` to `jammy`
- Ensure we use our ubuntu base image also for `qgis`
:::

```mermaid
graph TD
    subgraph base_images ["Pulled base images (Docker Hub)"]
        ubuntu_jammy["ubuntu:jammy"]
        ubuntu_focal["ubuntu:focal"]
        ros_humble["ros:humble"]
        ros_foxy["ros:foxy"]
        postgres
        autoheal
        nginx
        micro-ros-agent
        glances
        qgis
    end

    subgraph built_images ["Built GISNav images"]
        subgraph apache_ctx[apache]
            apache
            mapserver
            fileserver
        end
        subgraph mavros_ctx[mavros]
            gisnav
            mavros
            mavros-msgs
        end
        gscam
        px4
        ardupilot
        qgc
    end


    ubuntu_focal --> ros_foxy
    ubuntu_jammy --> apache
    apache --> fileserver
    apache --> mapserver
    ubuntu_jammy --> ros_humble
    ros_humble --> mavros-msgs
    mavros-msgs --> mavros
    mavros-msgs --> gisnav
    ros_humble --> gscam
    ubuntu_focal --> px4
    ros_foxy --> ardupilot
    ubuntu_focal --> qgc

```

### Network isolation

Isolation of groups of directly unrelated services is provided by allocating dedicated Docker bridge networks to groups of related services. Docker bridge networks have in-built DNS which means the container names resolve to their respective IP addresses on the bridge networks. The container name will equal the service name prefixed with `gisnav-` and suffixed with `-1`. For example, deploying the `mapserver` service using the Compose file will thereby start a container that can be found by any other container on the same network by its hostname `gisnav-mapserver-1`.

### Shared volumes

Configuration files, orthoimagery and DEM rasters and other data that is intended to be managed by end users are bind-mounted via separate volumes that are exposed to end-user administration tools.

### Service topography

The diagram below depicts the GISNav services topography for SITL simulation that is implemented by Docker Compose.

Some notes on the service topography:

- The Application services, Simulation services, Middleware services, GIS services, Dev services, and Admin services terms are an attempt to identify and give descriptive names to layers that have naturally emerged in the architecture. They have no further description besides that only neighboring layers should talk to each other - there should be no jumps across layers in communication.
- `socat` could also be considered a middleware service but runs non-containerized on the Docker host so it is not included in the Middleware services in the diagram.
- GISnav uses `gscam` to publish the ROS `sensor_msgs.msg.CameraInfo` and `sensor_msgs.msg.Image` messages. The camera topics are not published over the MAVROS middleware.
- ROS middleware does not have to be in the same network as `gisnav`. The shared memory device is used instead of going through the network stack since we will be passing a lot of (pointers to) images around.
- `qgis` is a GUI app that is part of the companion computer grouping but should probably only be run on the simulation host during development i.e. when using the simulation host to also run the companion computer services.

::: info Todo
- `docs-volume` not yet implemented, but is intended to contain static documentation.
- There will probably be need for a web based viewer (=not QGIS) for the maps in the GIS server that can be accessed from the admin network / via nginx
- Make shared memory transport work. It does not work possibly because of the note on privileged uses [here](https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/shared_memory/shared_memory.html#segment). A `dds` network is included to allow Fast DDS to communicate via UDP over the network stack instead (`dds` network included in diagram and it includes the `gisnav` service).
- `px4` shares the host network stack (along with `qgc`) because receiving communication from `uxrce_dds_agent` seems to depend on ephemeral ports  (unverified) -> try to isolate the simulator and QGC in a `mavlink` network
:::

```mermaid
graph TB

    subgraph simulation_host["Simulation host"]
        subgraph mavlink ["host"]
            mw_qgc[qgc]
            subgraph simulation ["Simulation Services"]
                simulation_px4[px4]
                simulation_ardupilot[ardupilot]
            end
        end
    end
    simulation_px4 ----->|"/dev/ttyS4 (px4 GPS 2)\ntcp:15000 (socat bridge)\n/dev/ttyS1 (gisnav NMEA)"| application_gisnav

    browser["Web browser"] -->|"80/http (443/https not currently supported)"| nginx

    subgraph companion["Companion computer (or simulation host)"]
        application_autoheal[autoheal]

        subgraph dds
            subgraph middleware ["Middleware Services"]
                middleware_mavros[mavros]
                middleware_micro_ros_agent["micro-ros-agent\n(uXRCE-DDS Agent)"]
                middleware_gscam[gscam]
            end
        end
        subgraph gis
            subgraph application ["Application Services"]
                application_gisnav[gisnav]
            end
            subgraph gis_services ["GIS Services"]
                gis_postgres["postgres\n(PostGIS)"]
                gis_mapserver[mapserver]
            end
            subgraph dev_services ["Dev Services"]
                gis_qgis["qgis"]
            end
        end

        subgraph admin
            subgraph admin_services ["Admin services"]
                homepage[homepage]
                fileserver["fileserver\n(FileGator)"]
                monitoring["monitoring\n(Glances)"]
            end
            nginx
        end

        subgraph volumes ["User managed\nshared volumes"]
            gscam_volume[gscam-volume]
            gis_maps_volume[maps-volume]
            application_gisnav_volume[gisnav-volume]
        end
        application_docs_volume[docs-volume]
    end

    mw_qgc -->|14550/udp\nMAVLink| simulation_px4
    simulation_px4 ----->|14540/udp\nMAVLink| middleware_mavros
    simulation_px4 ----->|8888/udp\nDDS-XRCE | middleware_micro_ros_agent
    simulation_px4 ----->|5600/udp\nRTP H.264 Video| middleware_gscam
    middleware_mavros -->|/dev/shm\nROS 2 Fast DDS| application_gisnav
    middleware_micro_ros_agent -->|/dev/shm\nROS 2 Fast DDS| application_gisnav
    middleware_gscam -->|/dev/shm\nROS 2 Fast DDS| application_gisnav
    application_gisnav -->|5432/tcp| gis_postgres

    application_gisnav -->|80/tcp\nHTTP WMS| gis_mapserver
    gis_postgres -->|5432/tcp| gis_qgis
    gis_mapserver -->|80/tcp\nHTTP WMS| gis_qgis
    gis_mapserver ---|/etc/mapserver| gis_maps_volume
    application_gisnav_volume ---|/etc/gisnav| application_gisnav
    application_docs_volume ---|/path/to/built/docs| application_gisnav
    nginx ---|3000/tcp| homepage
    nginx ---|80/tcp| fileserver
    nginx ---|61208/tcp| monitoring
    nginx ---|/path/to/built/docs| application_docs_volume
    fileserver ---|"/var/www/filegator/"| volumes
    gscam_volume ---|/etc/gscam| middleware_gscam

    dds -.-|dds| application_gisnav

    classDef network fill:transparent,stroke-dasharray:10 5;
    class mavlink,gis,admin,dds network

    classDef host fill:transparent,stroke:10;
    class simulation_host,companion host
```

### Service descriptions

The Homepage labels in the `docker/docker-compose.yaml` file have brief descriptions of the purpose of each individual service. These labels are picked up by the `homepage` service and displayed on the [admin portal](/admin-portal).
