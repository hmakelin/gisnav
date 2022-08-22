[![Watch the GISNav demo video](https://img.youtube.com/vi/JAK2DPZC33w/0.jpg)](https://youtu.be/JAK2DPZC33w)

# Introduction

> **Warning** Do not use this software for real drone flights. This software is untested and has only been demonstrated
> in a software-in-the-loop (SITL) simulation environment.

GISNav is a ROS 2 package that enables map-based visual navigation for airborne drones **in a simulation environment**.

GISNav provides an *accurate* **global** position for an airborne drone by visually comparing frames from the drone's 
nadir-facing camera to a map of the drone's *approximate* global position retrieved from an underlying 
GIS system.

# Mock GPS Example

The below steps demonstrate how GISNav's example `MockGPSNode` ROS 2 node enables GNSS-free flight with PX4 Autopilot's 
[Mission mode][1] in a SITL simulation.

You will need to have Python 3 and [NVIDIA Container Toolkit installed][2], and a [WMS endpoint][3] for high-resolution 
aerial or satellite imagery with coverage over the flight area (San Carlos KSQL airport in CA, USA) available.

[1]: https://docs.px4.io/v1.12/en/flight_modes/mission.html

[2]: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html

[3]: https://hmakelin.github.io/gisnav/pages/setup.html#wms-endpoint

## Build and run GISNav in dockerized environment

Replace the `<your-map-server-url>` substring below with your WMS endpoint URL and build the Docker images:

```bash
git clone https://github.com/hmakelin/gisnav-docker.git
cd gisnav-docker
docker-compose build --build-arg MAPPROXY_TILE_URL="https://<your-map-server-url>/tiles/%(z)s/%(y)s/%(x)s"
```

> **Note** The build for the `px4-sitl` image takes a long time, especially if you are building it for the first time.

Once the images have been built, run a `mapproxy` container in the background and a `px4-sitl` container in the 
foreground (you will need to type in some commands into the PX4 shell later):

```bash
docker-compose up -d mapproxy
docker-compose up px4-sitl
```

> **Note**
> The `px4-sitl` container should pop up [Gazebo][4] and [QGroundControl][5] automatically once you run it. The Gazebo 
> window may take several minutes to appear, while QGroundControl should pop up in a few seconds after running the 
> container.

[4]: https://gazebosim.org/home

[5]: https://qgroundcontrol.com/

## Upload flight plan via QGroundControl

Once both the Gazebo and QGroundControl windows have appeared (QGroundControl should show the drone location near San 
Carlos airport), use QGroundControl to upload the sample `test/assets/ksql_airport.plan` flight plan that is already 
included inside the Docker container, and start the mission.

## Simulate GPS failure

> **Warning** Do not attempt this on a real flight - simulation use only.

Wait until the drone has risen to its final mission altitude. You should see a visualization of the GISNav-estimated 
field of view projected on the ground pop up. You can then try disabling the primary GPS from your PX4 shell:

```
failure gps off
```

The drone should now continue to complete its mission *GNSS-free* with GISNav substituting for GPS.

You can check if PX4 is receiving the mock GPS position estimates by typing the following in the PX4 shell:

```
listener sensor_gps
```

If the printed GPS message has a `device_id` other than 0, your PX4 is receiving the mock GPS node output as expected.

# Documentation

See the [latest developer documentation][8] for information on how to setup a local environment for GISNav development, 
for code examples and API documentation, and for contribution guidelines.

You can also generate the GISNav documentation with Sphinx:

```bash
cd ~/colcon_ws/src/gisnav
python3 -m pip install -r requirements.txt
```

Then change to `docs/` folder and generate the HTML documentation which will appear in the `_build/` folder:

```bash
cd docs && make html
```

[8]: https://hmakelin.github.io/gisnav

# License

This software is released under the MIT license. See the `LICENSE.md` file for more information.
