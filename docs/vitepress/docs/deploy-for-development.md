# Deploy for development

The recommended way of developing GISNav is to deploy a SITL simulation and supporting services using [Docker Compose](/deploy-with-docker-compose) while deploying the GISNav ROS 2 launch configuration directly on the host. With a local non-containerized installation of GISNav it is easier re-test the software without having to rebuild a Docker image and re-deploy a container every time changes are made to the source code.

## Prerequisites

<!--@include: ./shared/require-install-locally.md-->

## Deploy via Makefile <Badge type="tip" text="Recommended"/>

The easy way to deploy for development is via the Makefile. Use the below commands to run a local ROS 2 default launch configuration of GISNav and to deploy a supporting SITL simulation as Docker Compose services.

- The `uorb` target uses the `micro-ros-agent` middleware service and `UORBNode`.
- The `ublox` and `nmea` targets use `socat` running on the Docker host to bridge the serial port output via TCP to the SITL simulation container.

::: code-group

```bash [uORB <Badge type="tip" text="Recommended for PX4"/>]
cd ~/colcon_ws/src/gisnav
make dev PROTOCOL=uorb
```

```bash [u-blox]
cd ~/colcon_ws/src/gisnav
make dev PROTOCOL=ublox
```

```bash [NMEA <Badge type="warning" text="Deprecated"/>]
cd ~/colcon_ws/src/gisnav
make dev PROTOCOL=nmea
```
:::

::: warning NMEA deprecated
The PX4 NMEA driver hard-codes some variables like the `s_variance_m_s` speed variance to 0 which may lead to failsafes triggering (unverified) if only relying on GISNav for velocity estimates (e.g. when [simulating failure of the primary GPS](/README#simulate-gps-failure)).
NMEA also was not originally intended for airborne drone navigation and the message types are not well suited for GISNav's use case.
:::

::: tip Ctrl-C and relaunch
You can use `Ctrl-C` and then `make dev` again to kill and relaunch GISNav without killing the supporting services. This enables iterating with quick changes to the source code without having to restart the simulation. You will still have to `colcon build --packages-select gisnav` in your workspace after making changes to the GISNav source code before relaunching with `make dev`.

:::

## Deploy via ROS launch system

The Makefile uses the ROS launch system under the hood to define and deploy configurations of multiple nodes with preconfigured parameters and optional launch arguments.

### Default configuration

A `local.launch.py` launch configuration is provided that allows choosing the output protocol and port.

To see what launch arguments the launch file accepts, type in the following command:

```bash
cd ~/colcon_ws
ros2 launch gisnav local.launch.py --show-args
```

### Redirecting serial output for SITL simulation <Badge type="info" text="NMEA/u-blox"/>

The `px4` SITL simulation container listens for serial messages (e.g NMEA or u-blox) at TCP port 15000. GISNav `NMEANode` and `UBloxNode` write into a serial port, and this serial port traffic must be bridged to the TCP port of the running `px4` container to make the mock GPS demo work in SITL simulation.

The Makefile `make dev PROTOCOL=nmea` and `make dev PROTOCOL=ublox` recipes create a pseudo-tty (virtual serial port) using `socat` to bridge the GISNav serial port output via TCP to the Docker container running the SITL simulation. The Makefile hardcodes `localhost` as the container host as the targets are intended for local development, but you can use the below example if your simulation is running on a different host:

```bash
cd ~/colcon_ws/src/gisnav
gnc start px4
PX4_IP=$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' gisnav-px4-1)
socat pty,link=/tmp/gisnav-pty-link,raw,echo=0 tcp:$(PX4_IP):15000
```

### Launch

You are now ready to launch GISNav via the ROS launch system.

::: code-group
```bash [uORB <Badge type="tip" text="Recommended for PX4"/>]
cd ~/colcon_ws
ros2 launch gisnav local.launch.py protocol:=uorb
```

```bash [u-blox]
cd ~/colcon_ws
PTY_PORT=$(readlink /tmp/gisnav-pty-link)
ros2 launch gisnav local.launch.py protocol:=u-blox port:=${PTY_PORT} baudrate:=${BAUDRATE:-9600}
```

```bash [NMEA <Badge type="warning" text="Deprecated"/>]
cd ~/colcon_ws
PTY_PORT=$(readlink /tmp/gisnav-pty-link)
ros2 launch gisnav local.launch.py protocol:=nmea port:=${PTY_PORT} baudrate:=${BAUDRATE:-9600}
```

:::
