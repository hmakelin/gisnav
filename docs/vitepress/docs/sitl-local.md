# Simulate GPS failure

This page describes how to prepare and deploy a local SITL simulation environment. The below video shows what the local SITL simulation should look like.

::: details Note on Docker commands seen on video
Unlike in the video, the simulation environment is no longer provided in a single container and is managed by `gnc` instead. The `gisnav-docker` repository seen in the beginning of the video is no longer maintained, and all the Docker source files have been moved over to the main `gisnav` repository.
:::

<video width="100%" height="auto" controls>
  <source src="https://user-images.githubusercontent.com/22712178/187902004-480397cc-460f-4d57-8ed7-13f4e9bb3757.mp4" type="video/mp4">
  Your browser does not support the video tag.
</video>

## Example walkthrough

The below steps demonstrate how GISNav enables GNSS-free flight with PX4 Autopilot's [Mission mode][1] in a SITL simulation.

[1]: https://docs.px4.io/main/en/flight_modes/mission.html

### Prerequisites

- Install the GISNav CLI via the [Debian package](/install-from-debian-package).

### Build and start SITL simulation

Prepare the containers for the SITL simulation environment:

```bash
gnc create --build px4 gisnav
```

Start your simulation:

::: info Slow startup on first run
Gazebo in the `px4` container will download some models on first startup which may take several minutes with slower internet. You may not see the Gazebo GUI pop up until after the models are downloaded.
:::

```bash
gnc start px4 gisnav
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
