# Run mock GPS demo

<!--@include: ./shared/warning-simulation-use-only.md-->

<video width="100%" height="auto" controls>
  <source src="https://user-images.githubusercontent.com/22712178/187902004-480397cc-460f-4d57-8ed7-13f4e9bb3757.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

## Introduction

GISNav is a ROS 2 package that accurately determines UAV global position by aligning real-time video with maps from an onboard GIS server.

GISNav provides a precise global position by visually comparing frames from the vehicle's nadir-facing camera to a map of the UAVs approximate global position retrieved from an onboard GIS server.

## Example walkthrough

The below steps demonstrate how GISNav enables GNSS-free flight with PX4 Autopilot's [Mission mode][1] in a SITL simulation.

[1]: https://docs.px4.io/main/en/flight_modes/mission.html

### Prerequisites

You will need to have the [Docker Compose plugin][2] and [NVIDIA Container Toolkit][3] installed.

[2]: https://docs.docker.com/compose/install/linux/
[3]: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html

### Build and run SITL simulation

Build the Docker images and create and run the containers (downloading and building everything will take a long time):

> [!WARNING] Warning: Exposed X server
> This script will expose your X server to the Docker containers to make GUI applications work.

```bash
git clone https://github.com/hmakelin/gisnav.git
cd gisnav
make -C docker demo
```

### Upload flight plan via QGroundControl

Once both the Gazebo and QGroundControl windows have appeared (QGroundControl should show the vehicle location near San Carlos airport), use QGroundControl to upload the sample `~/ksql_airport_px4.plan` flight plan that is included inside the Docker container, and then start the mission.

### Simulate GPS failure

After a while and once the vehicle has gained some altitude you should see a visualization of the GISNav-estimated field of view projected on the ground appear. You can then try disabling GPS through your [MAVLink or NSH shell][4] (accessible e.g. through QGroundControl > Analyze Tools > MAVLink Console):

```nsh
failure gps off
```

The vehicle should now continue to complete its mission GNSS-free with GISNav substituting for GPS.

You can check if PX4 is receiving the mock GPS position estimates by typing the following in the MAVLink shell:

```nsh
listener sensor_gps
```

[4]: https://docs.px4.io/main/en/debug/mavlink_shell.html#qgroundcontrol
