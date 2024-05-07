# Deploy for development

The recommended way of developing GISNav is to deploy a SITL simulation and supporting services using [Docker Compose](/deploy-with-docker-compose) while deploying the GISNav ROS 2 launch configuration directly on the host. With a local non-containerized installation of GISNav it is easier re-test the software without having to rebuild a Docker image and re-deploy a container every time changes are made to the source code.

## Prerequisites

<!--@include: ./shared/require-install-locally.md-->

## Deploy via Makefile <Badge type="tip" text="Recommended"/>

The easy way to deploy for development is via the Makefile. Use the below commands to run a local ROS 2 default launch configuration of GISNav and to deploy a supporting SITL simulation as Docker Compose services.

```bash
cd ~/colcon_ws/src/gisnav
make -C docker dev
```

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
Merge `dev.launch.py` into `default.launch.py` by adding more launch arguments to `default.launch.py` that invoke current `dev.launch.py` functionality

:::

### Edit local hosts file

TODO

### Redirecting serial output for SITL simulation

The `px4` SITL simulation container listens for NMEA messages at TCP port 15000. GISNav `NMEANode` writes into a serial port, and this serial port traffic must be bridged to the TCP port of the running SITl container to make the mock GPS demo work in SITL simulation. This is done using `socat` which creates a new virtual serial port for you (a pseudo-tty):

The Makefile `dev` recipe looks up the IP addresses of the containers of the supporting services and creates a pseudo-tty (virtual serial port) using `socat` to bridge the GISNav serial port output via TCP to the Docker container running the PX4 SITL simulation.

```bash
PX4_IP=`docker inspect -f '{{range .NetworkSettings.Networks}}{{.IPAddress}}{{end}}' gisnav-px4-1`
socat pty,link=/tmp/gisnav-pty-link,raw,echo=0 tcp:$(PX4_IP):15000
```

Assuming you have already installed the `gisnav` colcon package, you can launch with a single command. You need to provide a serial port and an optional baudrate as launch args to the default launch configuration.

::: code-group
```bash [uORB <Badge type="tip" text="Recommended for PX4"/>]
cd ~/colcon_ws
ros2 launch gisnav default.launch.py protocol:=uorb
```

```bash [NMEA]
cd ~/colcon_ws
ros2 launch gisnav default.launch.py protocol:=nmea port:=${PTY_PORT} baudrate:=${BAUDRATE:-9600}
```

:::
