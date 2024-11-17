https://user-images.githubusercontent.com/22712178/187902004-480397cc-460f-4d57-8ed7-13f4e9bb3757.mp4


## Project Overview
GISNav is a ROS 2 package that enables map-based visual navigation for airborne drones **in a simulation environment**.
GISNav provides a *precise* global position by visually comparing frames from the drone's nadir-facing camera to a map
of the drone's *approximate* global position retrieved from an onboard GIS system.

> **Warning** Do not use this software for real drone flights. GISNav is untested and has only been demonstrated
> in a simulation environment.

## Prerequisites
This project is built based on ROS 2. Make sure to have the following installed before proceeding:
- [Docker Compose plugin](https://docs.docker.com/compose/install/linux/)
- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)

## Installation Steps
### Manual Installation GISNav CLI
The GISNav CLI (`gnc`) is a Docker Compose wrapper that simplifies building and deploying GISNav's services. It is packaged as a Debian distributable.

Create the Debian distributable and install `gnc` from it using the following commands:

```bash
git clone https://github.com/hmakelin/gisnav.git
cd gisnav
make install
```

### VBuild and run SITL simulation
To build the Docker images and create and run the containers (and also verifying the success of the previous step):

```bash
gnc build px4 gisnav --with-dependencies
gnc create px4 gisnav
gnc start px4 gisnav && gnc stop autoheal
```

> **Note**:
> - Downloading and building everything will take a long time
> - This script will expose your X server to Docker containers for GUI applications.
> - We stop autoheal because current healthchecks are quite naive and often flag healthy services as unhealthy.
> - The `px4` service will download Gazebo models on the first run and may start slowly.
> - The `mapserver` service will transfer files to a shared volume during its first run and may also start slowly.

## Example Use Case: Mock GPS Example
1. Build and run the SITL simulation as described in the previous step.
2. Upload the sample flight plan (`~/ksql_airport_px4.plan`) via QGroundControl once the Gazebo and QGroundControl windows appear.
3. Start the mission and simulate GPS failure using the MAVLink Shell command:

```bash
failure gps off
```

The drone should continue its mission *GNSS-free*, with GISNav substituting for GPS. Check PX4 receiving the mock GPS position estimates with:

```bash
listener sensor_gps
```

> **Note**: GISNav uses uORB messages to bypass the GPS driver, so commands like `gps status` will not show the GPS status.

4. Stop all simulation services with:

```bash
gnc stop
```

## Documentation
See the [latest developer documentation](https://hmakelin.github.io/gisnav) for on how to setup a local environment for GISNav development, for code examples and API documentation, and integration examples.

## Help Information
Feel free to open issues and pull request if you encounter some problems.

## License
This software is released under the MIT license. See the `LICENSE.md` file for more information.
