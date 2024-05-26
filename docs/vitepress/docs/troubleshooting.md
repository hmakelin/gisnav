# Troubleshooting

On this page you will find tips for solving some common problems with the software.

## X window or display or GUI not appearing

This might be accompanied by error messages such as the ones below:

```console
px4-1  | [Err] [RenderEngine.cc:749] Can't open display: :1
px4-1  | [Wrn] [RenderEngine.cc:89] Unable to create X window. Rendering will be disabled
px4-1  | [Wrn] [RenderEngine.cc:292] Cannot initialize render engine since render path type is NONE. Ignore this warning if rendering has been turned off on purpose.
px4-1  | Authorization required, but no authorization protocol specified
px4-1  | [Wrn] [GuiIface.cc:114] could not connect to display :1
px4-1  | [Msg] Could not load the Qt platform plugin "xcb" in "" even though it was found.
px4-1  | [Err] [GuiIface.cc:118] This application failed to start because no Qt platform plugin could be initialized. Reinstalling the application may fix this problem.
```

### Expose X server

If the Gazebo, QGroundControl, RViz, or QGIS windows do not appear on your screen soon after [deploying your Docker Compose services](#deploy-with-docker-compose), you may need to expose your X server to your containers.

::: info `gisnav` required in container names
The scripts here look for containers that have the string `gisnav` in their names. It is important that you use the `-p gisnav` option or `COMPOSE_PROJECT_NAME=gisnav` environment variable when building and creating your containers.

:::

<!--@include: ./shared/expose-x-server.md-->

## Simulation is slow

### Headless mode

When developing on a lower performance system or when doing automated testing (e.g., with MAVSDK), you may want to run the Gazebo simulation in headless mode to increase performance:

::: info Building Compose services
See [Deploy with Docker Compose](/deploy-with-docker-compose) for more information on how to build the Docker Compose services used in the below example.

:::

::: code-group

```bash [PX4]
cd ~/colcon_ws/src/gisnav/docker
docker compose -p gisnav -f docker-compose.yaml -f docker-compose.headless.yaml up px4
```

```bash [ArduPilot]
cd ~/colcon_ws/src/gisnav/docker
docker compose -p gisnav -f docker-compose.yaml -f docker-compose.headless.yaml up ardupilot
```
:::

### GPU drivers not available

Your system might not be using the GPU. Check that CUDA is available:

```bash
hmakelin@hmakelin-MS-7D48:~/colcon_ws/src/gisnav$ python3
Python 3.10.12 (main, Nov 20 2023, 15:14:05) [GCC 11.4.0] on linux
>>> import torch
>>> torch.cuda.is_available()
True
```

Sometimes this command will not return `True` and possibly raises an error. Try updating your drivers and/or restarting your computer.

### GPU temperature

Check your GPU temperature to ensure it's not overheating and therefore being throttled. Overheating might be a more common problem on laptops with less efficient heat discharge.

## Keypoint match visualization not appearing

Here we assume the SITL or HIL simulation is working but GISNav itself does not appear to be working since the keypoint match visualization does not appear.

### Disable SharedMemory for Fast DDS

If you are not able to establish ROS communication between the `mavros` container and the host, or receive the above error when using the `--network host` option, try disabling SharedMemory for Fast DDS on your host. You can do so by creating an XML configuration and restarting the ROS daemon with the new configuration. You can do so by creating an XML
configuration (e.g., ``disable_shared_memory.xml``) as described in [this comment](https://github.com/eProsima/Fast-DDS/issues/1698#issuecomment-778039676)
or [discussion here](https://stackoverflow.com/questions/65900201/troubles-communicating-with-ros2-node-in-docker-container) and restarting the ROS daemon with the new configuration:

```bash
export FASTRTPS_DEFAULT_PROFILES_FILE=disable_shared_memory.xml
ros2 daemon stop
ros2 daemon start
```

## ArduPilot simulation not working

::: info Todo
Currently, ArduPilot support is broken, and the simulation is not expected to work. Use PX4 instead.

:::

### Disable AppArmor

::: warning Security implications
Consider the security implications to your system before trying this out.

:::

Possibly needed if using `--network host`: If QGroundControl or Gazebo do not seem to be starting when running the containers, you may need to run them image with `--security-opt apparmor:unconfined` or `--privileged` options.

## Simulator is not receiving NMEA messages

During SITL simulation, the simulator might not receive the NMEA messages via the virtual serial port (pseudo-tty). The examples here assume you are using the `px4` service for simulation.

### Check TCP port

```bash
hmakelin@hmakelin-MS-7D48:~/colcon_ws/src/gisnav/docker$ docker compose -p gisnav exec -it px4 bash
root@669b94309b51:/PX4-Autopilot# tcpdump port 15000 and '(tcp-syn|tcp-ack)!=0'
```

### Check serial port

```bash
hmakelin@hmakelin-MS-7D48:~/colcon_ws/src/gisnav/docker$ docker compose -p gisnav exec -it px4 bash
root@669b94309b51:/PX4-Autopilot# cat /dev/ttyS4
$GPGGA,090425,3731.4157,N,12215.3002,W,1,12,0.00,45.5,M,0.0,M,,*68
```

### Issues reinstalling `gisnav`

Try removing any previous installation and cleaning `apt` cache before reinstalling:

```bash
sudo apt-get remove gisnav
sudo apt-get clean
```

## Cannot build `mavros` or X on Raspberry Pi

### CPU overheating

The CPU might be overheating. You can get a case for your Pi that has a fan to improve heat dissipation.

### Out of memory

4GB of memory may not be enough to build `mavros`. You can try adding another 4 GB using a swapfile:

```bash
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

Monitor your swapfile usage:

```bash
watch swapon --show
```

Monitor your memory usage:

```bash
watch free -m
```

Delete the swapfile after building (optional):

```bash
sudo swapoff /swapfile
sudo rm /swapfile
```

## General debugging

### Run shell inside container

If you need to do debugging on your [Docker Compose images](/deploy-with-docker-compose) with GUI applications (e.g., Gazebo inside the `px4` service), run bash inside your service container using the following command:

```bash
cd ~/colcon_ws/src/gisnav/docker
docker compose -p gisnav run px4 bash
```

If the container is already running, you must use `exec` instead:

```bash
cd ~/colcon_ws/src/gisnav/docker
docker compose -p gisnav exec px4 bash
```

::: tip `docker` vs `docker compose`
You will probably want to use `docker compose` here instead of `docker` for the GUI applications to work properly, but `docker` will also work
for basic debugging that does not require launching GUI apps.

:::
