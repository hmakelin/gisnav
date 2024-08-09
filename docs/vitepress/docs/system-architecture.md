# System architecture

The GISNav ROS 2 package receives upstream inputs from the autopilot via MAVLink and transforms them in multiple sequential steps to downstream outputs, the most important of which are the mock GPS messages. In GISNav this data processing algorithm is expressed as a distributed unidirectional network of ROS nodes.

The GISNav ROS application also has many external interfaces and effective development requires a simulation environment. GISNav defines and [deploys that simulation environment with Docker Compose](/deploy-with-docker-compose).

This page provides an overview of both the topography of the GISNav ROS 2 package itself as well as how these different Docker Compose services that consitute the simulation environment relate to each other.

## ROS topography

The core ROS topography diagram below depicts how ROS messages flow through GISNav. The [API reference](/reference/) has more detailed information on the purpose and design of each ROS node.

```mermaid
graph TB
    subgraph ros_middleware["ROS middleware"]
        MAVROS
        gscam
    end

    MAVROS -->|"sensor_msgs/NavSatFix"| BBoxNode
    MAVROS -->|"geometry_msgs/PoseStamped"| BBoxNode
    MAVROS -->|"mavros_msgs/GimbalDeviceAttitudeStatus"| BBoxNode
    gscam ---->|"sensor_msgs/CameraInfo"| BBoxNode
    gscam ---->|"sensor_msgs/Image"| TwistNode

    subgraph core["GISNav core nodes"]
        BBoxNode -->|"geographic_msgs/BoundingBox"| GISNode
        GISNode -->|"gisnav_msgs/OrthoImage"| StereoNode
        StereoNode -->|"gisnav_msgs/OrthoStereoImage\n(rotated & cropped reference image + query image SIFT keypoints passed through)"| PoseNode
        TwistNode -->|"sensor_msgs/PointCloud2\n(SIFT keypoints)"| StereoNode
    end

    subgraph extension["GISNav extension nodes"]
        UBXNode["UBXNode"]
        UORBNode["UORBNode"]
        NMEANode["NMEANode"]
    end

    odometry["/robot_localization/odometry/filtered"] -->|"nav_msgs/Odometry"| UORBNode
    odometry -->|"nav_msgs/Odometry"| UBXNode
    odometry -->|"nav_msgs/Odometry"| NMEANode

    subgraph robot_localization["robot_localization ROS package"]
        ekf_global_node
        ekf_local_node
    end
    ekf_global_node -->|"nav_msgs/Odometry"| odometry
    ekf_local_node -->|"nav_msgs/Odometry"| odometry
    TwistNode -->|"geometry_msgs/PoseWithCovarianceStamped\n(local pose, odom frame)"| ekf_local_node

    PoseNode -->|"geometry_msgs/PoseWithCovarianceStamped\n(global pose, map frame)"| ekf_global_node
    UORBNode -->|"px4_msgs/SensorGps"| micro_ros_agent[" "]
    UBXNode -->|"ublox_msgs/NavPVT"| ubx[" "]
    NMEANode -->|"nmea_msgs/Sentence"| nmea[" "]

    ekf_global_node -->|"nav_msgs/TransformStamped\n(map->odom)"| tf
    ekf_local_node -->|"nav_msgs/TransformStamped\n(odom->base_link)"| tf

    style micro_ros_agent fill-opacity:0, stroke-opacity:0;
    style ubx fill-opacity:0, stroke-opacity:0;
    style nmea fill-opacity:0, stroke-opacity:0;
    style tf fill-opacity:0, stroke-width:1px,stroke-dasharray:3;
    style odometry fill-opacity:0, stroke-width:1px,stroke-dasharray:3;

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
        ubx
        nmea
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
    ros_humble --> ubx
    ubuntu_focal --> px4
    ros_foxy --> ardupilot
    ubuntu_focal --> qgc

```

### Network isolation

Isolation of groups of directly unrelated services is provided by allocating dedicated Docker bridge networks to groups of related services. Docker bridge networks have in-built DNS which means the container names resolve to their respective IP addresses on the bridge networks. The container name will equal the service name prefixed with `gisnav-` and suffixed with `-1`. For example, deploying the `mapserver` service using the Compose file will thereby start a container that can be found by any other container on the same network by its hostname `gisnav-mapserver-1`.

If a service wants to talk to a service in another network, the requests must go through the `nginx` reverse proxy. Only specific routes are allowed to enable important features - these are shown in the [service topography diagram](#service-topography).

### Shared volumes

Configuration files, orthoimagery and DEM rasters and other data that is intended to be managed by end users are bind-mounted via separate volumes that are exposed to end-user administration tools.

### Service topography

The diagram below depicts the GISNav services topography for SITL simulation that is implemented by Docker Compose.

Some notes on the service topography:

- The Application services, Simulation services, Middleware services, GIS services, and Admin services terms are an attempt to identify and give descriptive names to layers that have naturally emerged in the architecture. They have no further description.
- GISnav uses `gscam` to publish the ROS `sensor_msgs.msg.CameraInfo` and `sensor_msgs.msg.Image` messages. The camera topics are not published over the MAVROS middleware.

::: info Todo
- Serve static documentation.
- Make shared memory transport work. It does not work possibly because of what's described in the note on privileged users [here](https://fast-dds.docs.eprosima.com/en/latest/fastdds/transport/shared_memory/shared_memory.html#segment). A `dds` network is included to allow Fast DDS to communicate via UDP over the network stack instead (`dds` network included in diagram and it includes the `gisnav` service).
- `px4` shares the host network stack (along with `qgc`) because receiving communication from `uxrce_dds_agent` seems to depend on ephemeral ports which makes exposing specific ports difficult (unverified) -> try to isolate the simulator and QGC in a `mavlink` network
- Show `gis` network, which connects `nginx`, `mapserver`, and `gisnav` (not in diagram)
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

    browser["Web browser"] -->|"80/http\n(443/https not currently supported)"| nginx

    subgraph companion["Companion computer (or simulation host)"]

        nginx -.-|admin| admin
        nginx -.-|dds| dds

        application_autoheal[autoheal]

        subgraph dds
            subgraph middleware ["Middleware Services"]
                middleware_mavros[mavros]
                middleware_micro_ros_agent["micro-ros-agent\n(uXRCE-DDS Agent)"]
                middleware_ubx_nmea[ubx or nmea]
                middleware_gscam[gscam]
            end
            subgraph application ["Application Services"]
                application_gisnav[gisnav]
            end
        end
        subgraph data

            subgraph gis_services ["GIS Services"]
                gis_mapserver[mapserver]
                gis_postgres["postgres\n(PostGIS)"]
            end
        end

        subgraph admin
            subgraph admin_services ["Admin services"]
                homepage[homepage]
                openlayers
                monitoring["monitoring\n(Glances)"]
                fileserver["fileserver\n(FileGator)"]
            end
        end

        subgraph volumes[User managed volumes]
            gscam_volume[gscam-volume]
            application_gisnav_volume[gisnav-volume]
            gis_maps_volume[maps-volume]
        end
    end

    mw_qgc -->|14550/udp\nMAVLink| simulation_px4
    simulation_px4 ------>|14540/udp\nMAVLink| middleware_mavros
    simulation_px4 ------>|8888/udp\nDDS-XRCE | middleware_micro_ros_agent
    simulation_px4 ------>|15000/TCP via socat\nUBX or NMEA| middleware_ubx_nmea
    simulation_px4 ------>|5600/udp\nRTP H.264 Video| middleware_gscam
    middleware_micro_ros_agent -->|/dev/shm\nROS 2 Fast DDS| application_gisnav
    middleware_mavros -->|/dev/shm\nROS 2 Fast DDS| application_gisnav
    middleware_ubx_nmea -->|/dev/shm\nROS 2 Fast DDS| application_gisnav
    middleware_gscam -->|/dev/shm\nROS 2 Fast DDS| application_gisnav

    application_gisnav -->|"80/tcp\nHTTP WMS/WFS-T\nvia nginx (local)\ndirectly (containerized)"| gis_mapserver
    application_gisnav_volume ---|/etc/gisnav| application_gisnav

    fileserver ----|"/var/www/filegator/repository\n/mapserver"| gis_maps_volume
    gis_maps_volume---|/etc/mapserver| gis_mapserver

    openlayers -.->|80/tcp\nHTTP WMS/WFS-T\nvia nginx| gis_mapserver
    fileserver ----|"/var/www/filegator/repository\n/gscam"| gscam_volume
    fileserver ----|"/var/www/filegator/repository\n/gisnav"| application_gisnav_volume

    gscam_volume ---|/etc/gscam| middleware_gscam

    gis_mapserver --- gis_postgres

    nginx -->|80/tcp\nHTTP| homepage
    nginx -->|80/tcp\nHTTP| monitoring
    nginx -->|80/tcp\nHTTP| fileserver
    nginx -->|80/tcp\nHTTP| gis_mapserver
    nginx -->|/var/www/html/static\n/openlayers| openlayers

    classDef network fill:transparent,stroke-dasharray:10 5;
    class mavlink,gis,admin,dds,data network

    classDef host fill:transparent,stroke:10;
    class simulation_host,companion host

    classDef volume fill:transparent;
    class volumes,gscam_volume,gis_maps_volume,application_gisnav_volume volume
```

### Service descriptions

The Homepage labels in the `docker/docker-compose.yaml` file have brief descriptions of the purpose of each individual service. These labels are picked up by the `homepage` service and displayed on the [admin portal](/admin-portal).
