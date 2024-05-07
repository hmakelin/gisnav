# Deploy for development

The recommended way of developing GISNav is to deploy a SITL simulation and supporting services using [Docker Compose](/deploy-with-docker-compose) while deploying the GISNav ROS 2 launch configuration directly on the host. With a local non-containerized installation of GISNav it is easier re-test the software without having to rebuild a Docker image and re-deploy a container every time changes are made to the source code.

## Prerequisites

<!--@include: ./shared/require-install-locally.md-->

## Deploy via Makefile <Badge type="tip" text="Recommended"/>

The easy way to deploy for development is via the Makefile. Use the below commands to run a local ROS 2 default launch configuration of GISNav and to deploy a supporting SITL simulation as Docker Compose services.

Two Makefile targets are provided for uORB and NMEA output:

- The `dev-uorb` target uses the `micro-ros-agent` middleware service and `UORBNode` and plays more nicely with PX4 than the NMEA target.
- The `dev-nmea` target uses `socat` running on the Docker host to bridge the serial port output from the `NMEANode` via TCP to the `px4` SITL simulation container.

::: code-group

```bash [uORB <Badge type="tip" text="Recommended for PX4"/>]
cd ~/colcon_ws/src/gisnav
make -C docker dev-uorb
```

```bash [NMEA]
cd ~/colcon_ws/src/gisnav
make -C docker dev-nmea
```
:::

::: tip Use uORB if possible
The PX4 NMEA driver and the NMEA 0183 protocol itself does not support sub-second precision for timestamps, while the PX4 (v1.14) GPS driver seems to have hard-coded a 5 Hz / 200 ms frequency requirement for GPS messages even when using the NMEA protocol. This leads to the secondary NMEA mock GPS often being flagged as unhealthy. For context, see the driver code lines [gps.cpp#L985](https://github.com/PX4/PX4-Autopilot/blob/v1.14.2/src/drivers/gps/gps.cpp#L985), [gps.cpp#L994](https://github.com/PX4/PX4-Autopilot/blob/v1.14.2/src/drivers/gps/gps.cpp#L994) and [nmea.cpp#L545-L553](https://github.com/PX4/PX4-GPSDrivers/blob/release/1.14/src/nmea.cpp#L545-L553).

The PX4 NMEA driver also hard-codes some variables like the `s_variance_m_s` speed variance to 0 which may lead to failsafes triggering (unverified) if only relying on GISNav for velocity estimates (e.g. when [simulating failure of the primary GPS](/README#simulate-gps-failure)).

:::

::: info Prompt for `sudo` access
The Docker Compose service containers are hard-coded to use their Docker DNS hostnames when communicating with each other over Docker bridge networks. The Makefile `dev` recipe will add some of these hostnames to the `/etc/hosts` file on your Docker host i.e. the development computer so that your local GISNav installation will be able to any containers it wants to talk to on the host network. You may therefore be prompted for `sudo` access when running the above command.

Read more about the [service architecture](/system-architecture#service-architecture) for more context.

:::

::: tip Ctrl-C and relaunch
You can use `Ctrl-C` and then `make dev` again to kill and relaunch GISNav without killing the supporting services. This enables iterating with quick changes to the source code without having to restart the simulation. You will still have to `colcon build --packages-select gisnav` in your workspace after making changes to the GISNav source code before relaunching with `make dev`.

:::

## Deploy via ROS launch system

The Makefile uses the ROS launch system under the hood to define and deploy configurations of multiple nodes with preconfigured parameters and optional launch arguments.

### Default configuration

The `default.launch.py` file can be used for all launches. A `dev.launch.py` configuration is provided to include additional nodes that help with development.

::: info Todo
Merge `dev.launch.py` into `default.launch.py` by adding more launch arguments to `default.launch.py`.

:::

To see what launch arguments the launch file accepts, type in the following command:

```bash
cd ~/colcon_ws
ros2 launch gisnav default.launch.py --show-args
```

### Edit local hosts file

With a local installation of GISNav we cannot rely on the Docker DNS and must find our containers manually.

The `dev-base-setup` Makefile target automates finding the relevant container IP addresses and adding them to your `/etc/hosts` file:

```bash
cd ~/colcon_ws/src/gisnav/docker
make dev-base-setup
```

Alternatively, following the below steps will do the same thing.

`GISNode` needs to know where `gisnav-mapserver-1` is, and `QGISNode` needs to know where `gisnav-postgres-1` is. You can find them with the following commands if the `mapserver` and `postgres` services are running:

```bash
cd ~/colcon_ws/src/gisnav/docker
docker compose -p gisnav start postgres mapserver
POSTGRES_IP=`docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' gisnav-postgres-1`
MAPSERVER_IP=`docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' gisnav-mapserver-1`
```

If these are not yet in your `/etc/hosts` file, add them:

```bash
echo "$(POSTGRES_IP) gisnav-postgres-1" | sudo tee -a /etc/hosts
echo "$(MAPSERVER_IP) gisnav-mapserver-1" | sudo tee -a /etc/hosts
```

### Redirecting serial output for SITL simulation <Badge type="info" text="NMEA only"/>

The `px4` SITL simulation container listens for NMEA messages at TCP port 15000. GISNav `NMEANode` writes into a serial port, and this serial port traffic must be bridged to the TCP port of the running `px4` container to make the mock GPS demo work in SITL simulation.

The Makefile `dev-nmea` recipe looks up the IP address of the running `px4` container creates a pseudo-tty (virtual serial port) using `socat` to bridge the GISNav serial port output via TCP to the Docker container running the PX4 SITL simulation.

```bash
cd ~/colcon_ws/src/gisnav/docker
docker compose -p gisnav start px4
PX4_IP=$(docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' gisnav-px4-1)
socat pty,link=/tmp/gisnav-pty-link,raw,echo=0 tcp:$(PX4_IP):15000
```

### Launch

You are now ready to launch GISNav via the ROS launch system.

::: code-group
```bash [uORB <Badge type="tip" text="Recommended for PX4"/>]
cd ~/colcon_ws
ros2 launch gisnav default.launch.py protocol:=uorb
```

```bash [NMEA]
cd ~/colcon_ws
PTY_PORT=$(readlink /tmp/gisnav-pty-link)
ros2 launch gisnav default.launch.py protocol:=nmea port:=${PTY_PORT} baudrate:=${BAUDRATE:-9600}
```

:::
