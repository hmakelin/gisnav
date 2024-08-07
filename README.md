https://user-images.githubusercontent.com/22712178/187902004-480397cc-460f-4d57-8ed7-13f4e9bb3757.mp4

# Introduction

> **Warning** Do not use this software for real drone flights. GISNav is untested and has only been demonstrated
> in a simulation environment.

GISNav is a ROS 2 package that enables map-based visual navigation for airborne drones **in a simulation environment**.

GISNav provides a *precise* global position by visually comparing frames from the drone's nadir-facing camera to a map
of the drone's *approximate* global position retrieved from an onboard GIS system.

# Mock GPS Example

The below steps demonstrate how GISNav enables GNSS-free flight with PX4 Autopilot's [Mission mode][1] in a SITL
simulation.

You will need to have the [Docker Compose plugin][2] and [NVIDIA Container Toolkit][3] installed.

[1]: https://docs.px4.io/main/en/flight_modes/mission.html
[2]: https://docs.docker.com/compose/install/linux/
[3]: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html

## Install GISNav CLI

The GISNav CLI (`gnc`) is a Docker Compose wrapper that significantly simplifies building and deploying GISNav's Docker Compose services.

Create the Debian package yourself and install `gnc` from it using the following commands:

```bash
git clone https://github.com/hmakelin/gisnav.git
cd gisnav
make install
```

## Build and run SITL simulation

Build the Docker images and create and run the containers (downloading and building
everything will take a long time):

> **Note**
> * This script will automatically expose your X server to the created
>   Docker containers to make the GUI applications work.
> * We stop autoheal because current healthchecks are quite naive and often
>   flag healthy services as unhealthy.
> * Gazebo in the `px4` service will on first run download a number of models
>   and will be slow to start up (on first run).
> * `mapserver` service will on first run transfer some files to a shared
>   volume and will be slow to start up (on first run).

```bash
gnc create --build px4 gisnav
gnc start px4 gisnav && gnc stop autoheal
```

[4]: http://wiki.ros.org/docker/Tutorials/GUI

## Upload flight plan via QGroundControl

Once both the Gazebo and QGroundControl windows have appeared (QGroundControl should show the drone location near San
Carlos airport), use QGroundControl to upload the sample `~/ksql_airport_px4.plan` flight plan that is included inside the
Docker container, and then start the mission.

## Simulate GPS failure

Wait until the drone has risen to its final mission altitude. You should see a visualization of the GISNav-estimated
field of view projected on the ground appear. You can then try disabling GPS through your [MAVLink Shell][5]
*(accessible e.g. through QGroundControl > Analyze Tools > MAVLink Console)*:

```
failure gps off
```

The drone should now continue to complete its mission *GNSS-free* with GISNav substituting for GPS.

You can check if PX4 is receiving the mock GPS position estimates by typing the following in the MAVLink shell:

```
listener sensor_gps
```

> **Note**
> The `gisnav` service will by default will send uORB messages to PX4 via the
> micro-ROS agent which bypasses the GPS driver so you will not see the GPS
> status with commands like `gps status`.

[5]: https://docs.px4.io/main/en/debug/mavlink_shell.html#qgroundcontrol

## Build and run SITL simulation

Stop all simulation services with the below command:

```bash
gnc stop
```

# Documentation

See the [latest developer documentation][6] for information on how to setup a local environment for GISNav development,
for code examples and API documentation, and for contribution guidelines.

[6]: https://hmakelin.github.io/gisnav

# License

This software is released under the MIT license. See the `LICENSE.md` file for more information.
