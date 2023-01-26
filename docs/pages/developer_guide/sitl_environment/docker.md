# GISNav SITL Simulation Environment

The `docker` folder contains scripts for generating SITL simulation environments for development, testing and 
demonstration. 

The `docker-compose.yaml` file defines the following services:

* `ardupilot`
  * ArduPilot Gazebo SITL simulation
  * Starts `gazebo-iris` model with added static down (FRD frame) facing camera at
    KSQL Airport
* `px4`
  * PX4 Gazebo SITL simulation
  * Starts `typhoon_h480` model at KSQL Airport
* `mavros`
  * Used for ArduPilot SITL
* `micro-ros-agent`
  * Used for PX4 SITL
* `mapserver`
  * WMS server with self-hosted [NAIP][2] and [OSM Buildings][3] data covering KSQL airport  
* `mapproxy`
  * WMS proxy for existing remote tile-based imagery endpoint. Alternative for `mapserver` when imagery layer needs to cover multiple flight regions (over presumedly multiple test scenarios covering too large an area to self-host).
* `gisnav`
  * GISNav ROS 2 package for demo purposes
  * Launches GISNav with PX4 configuration by default. Intended to be used with `px4` service, but 
    Docker command can be changed to launch for `ardupilot`.

[1]: https://github.com/hmakelin/gisnav
[2]: https://en.wikipedia.org/wiki/National_Agriculture_Imagery_Program
[3]: https://osmbuildings.org/

## Prerequisites
You must install [docker][4] and the [docker compose plugin][5] to run these example commands.

[4]: https://docs.docker.com/engine/install/
[5]: https://docs.docker.com/compose/install/linux/

If you have an NVIDIA GPU on your host machine, ensure you have [NVIDIA Container Toolkit installed][6].

[6]: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html


## PX4 SITL (Mock GPS Demo)

Follow these instructions to launch the SITL simulation used in the [mock GPS demo][7].

> **Note** Run the below commands without the `gisnav` service if you want to develop or test a local copy of `gisnav`

[7]: https://github.com/hmakelin/gisnav/blob/master/README.md#mock-gps-example

### Build

To build the `mapserver`, `px4`,  `micro-ros-agent`, and `gisnav` services by running the following command:

```bash
docker compose build mapserver px4 micro-ros-agent qgc gisnav
```

### Run

Run the PX4 SITL simulation with GISNav:

```bash
docker compose up mapserver px4 micro-ros-agent qgc gisnav
```

### Shutdown

```bash
docker-compose down
```

## ArduPilot SITL

Build and run the SITL simulation environment with ArduPilot instead of PX4:

### Build
```bash
docker compose \
  -f docker-compose.yaml \
  -f docker-compose.ardupilot.yaml \
  build mapserver ardupilot mavros qgc
```

### Run
```bash
docker compose \
  -f docker-compose.yaml \
  -f docker-compose.ardupilot.yaml \
  up mapserver ardupilot mavros qgc
```

## Mapproxy

Run the SITL simulation with a WMS proxy instead of locally hosted maps:

> **Note**
> Replace the example `MAPPROXY_TILE_URL` string below with your own tile-based endpoint url (e.g. WMTS). See
> [MapProxy configuration examples][8] for more information on how to format the string.

[8]: https://mapproxy.org/docs/latest/configuration_examples.html

```bash
docker-compose build \
  --build-arg MAPPROXY_TILE_URL="https://<your-map-server-url>/tiles/%(z)s/%(y)s/%(x)s" \
  mapproxy px4 micro-ros-agent gisnav
docker compose up mapproxy px4 micro-ros-agent qgc
```

## Troubleshooting

### Expose `xhost`

If the Gazebo and QGroundControl windows do not appear on your screen soon after running your container you may need to 
expose your ``xhost`` to your Docker container as described in the [ROS GUI Tutorial][9]:

[9]: http://wiki.ros.org/docker/Tutorials/GUI

```bash
export containerId=$(docker ps -l -q)
xhost +local:$(docker inspect --format='{{ .Config.Hostname }}' $containerId)
```

### Headless mode

You may want to run Gazebo in headless mode when doing automated testing ([e.g. with mavsdk][10]):

[10]: https://github.com/hmakelin/gisnav/blob/master/test/sitl/sitl_test_mock_gps_node.py

```bash
GAZEBO_HEADLESS=1 docker compose up px4
```

### Disable SharedMemory for Fast DDS

> [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7412: open_and_lock_file failed -> Function 
> open_port_internal

If you are not able to establish ROS communication between the `mavros` or `micro-ros-agent` container and the host, or 
receive the above error when using `--network host`, try disabling SharedMemory for Fast DDS **on your host**. You can
do so by creating an XML configuration (e.g. `disable_shared_memory.xml`) as described in [this comment][11] or 
discussion [here][12] and restarting ROS 2 daemon with the new configuration:

[11]: https://github.com/eProsima/Fast-DDS/issues/1698#issuecomment-778039676
[12]: https://stackoverflow.com/questions/65900201/troubles-communicating-with-ros2-node-in-docker-container

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=disable_fastrtps.xml
ros2 daemon stop
ros2 daemon start
```

### Disable AppArmor for ArduPilot SITL

> dbus[33]: The last reference on a connection was dropped without closing the connection. This is a bug in an 
> application. See dbus_connection_unref() documentation for details. Most likely, the application was supposed to call 
> dbus_connection_close(), since this is a private connection. D-Bus not built with -rdynamic so unable to print a 
> backtrace

**Caution advised**: Possibly needed if using `--network host`: If QGroundControl or Gazebo do not seem to be starting 
when running the containers, you may need to run them image with `--security-opt apparmor:unconfined` or `--privileged` 
options.

### Run shell inside container

If you need to do debugging on the images with GUI applications enabled (e.g. Gazebo inside `px4`), you can use the 
following command to run bash inside the container:

```bash
docker run -it \
  --env="DISPLAY" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume "/dev/shm:/dev/shm" \
  --volume="/dev/dri:/dev/dri" \
  --gpus all \
  --tty \
  --network host \
  --entrypoint="/bin/bash" \
  gisnav-px4
```
