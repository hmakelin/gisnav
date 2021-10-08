# WMS Map Matching

## Introduction

This ROS2 package matches a nadir-facing video stream from an airborne drone's camera to a map of its location. This
package is designed to be simulated software-in-the-loop (SITL) with [PX4](https://docs.px4.io/master/) in
[Gazebo](https://gazebosim.org/).

Current implementation retrieves a map raster from a Web Map Service (WMS) endpoint for the vehicle's approximate
location as determined by GNSS and then matches it to frame from the video stream using a graph neural network based 
algorithm ([SuperGlue](https://github.com/magicleap/SuperGluePretrainedNetwork)).

## Getting started

These instructions provide an example for running this package in a PX4-Gazebo SITL simulation. They assume you are
working on a Linux system. Running this package in another kind of setup is also possible but may require further
integration work.

### 1. Prerequisites

You will need to setup the following items to get started:

1. [Install ROS2](https://docs.ros.org/en/foxy/Installation.html)
2. [Install PX4](https://docs.px4.io/master/en/dev_setup/dev_env.html) for the Gazebo SITL target
3. [Install PX4-ROS2 bridge](https://docs.px4.io/master/en/ros/ros2_comm.html#installation-setup), including FastDDS
4. [Install QGroundControl](https://docs.qgroundcontrol.com/master/en/index.html)
5. [Install MapProxy](https://mapproxy.org/download)
6. [Install gscam2](https://github.com/clydemcqueen/gscam2)

### 2. Configure and launch MapProxy

Make a YAML configuration file for your MapProxy server. You can use the following example of proxying a tiled endpoint
as a WMS endpoint as a starting point. Many web services use a tiled endpoint instead of WMS because tiles can be
cached.

**TODO: provide example .yaml file**

`$ mapproxy-util serve-develop worldimagery.yaml`

### 3. Launch Gazebo simulation

In the `PX4-Autopilot` installation folder, launch a simulation in the `ksql_airport` (San Carlos airport) world. The
`typhoon_h480` option will enable video streaming (see
[PX4 documentation here](https://docs.px4.io/master/en/simulation/gazebo.html#video-streaming)). Note the double
underscore between before the `ksql_airport` world suffix.

`~/PX4-Autopilot$ make px4_sitl_rtps gazebo_typhoon_h480__ksql_airport`

### 4. Launch microRTPS agent

Open a shell and type in the following command to launch the microRTPS agent:

`$ micrortps_agent -t UDP`

### 5. Launch QGroundControl

In a new shell, navigate to the folder where you installed QGroundControl and launch the app:

`$ ./QGroundControl.AppImage`

### 6. Launch gscam2

**TODO: example gscam_params file and camera_calibration file**

`ros2 run gscam2 gscam_main --ros-args --params-file gscam_params.yaml -p camera_info_url:=file://$PWD/camera_calibration.yaml -p preroll:=True -p sync_sinc:=False`

### 7. Launch the matching node

`~/px4_ros_com_ros2$ ros2 run wms_map_matching matching_node --ros-args --log-level debug`


## Published Topics

- `~essential_matrix` (`float64[9]`)
- `~fundamental_matrix` (`float64[9]`)
- `~homography_matrix` (`float64[9]`)
- `~pose` (`float64[12]`)

## Subscribed Topics

The `camera_info` and `image_raw` topics are assumed to be provided by [gscam2](https://github.com/clydemcqueen/gscam2).
The `VehicleLocalPosition_PubSubTopic` is assumed to be provided by the
[PX4 microRTPS bridge](https://docs.px4.io/master/en/middleware/micrortps.html) and is therefore appended with the
`_PubSubTopic` suffix.

- `camera_info` (`sensor_msgs/CameraInfo`)
- `image_raw` (`sensor_msgs/Image`)
- `VehicleLocalPosition_PubSubTopic` (`px4_msgs/VehicleLocalPosition`)

## Parameters

The default parameters are defined in `config/config.yaml`. Some of the parameters are defined as read-only and cannot
be changed at runtime. You can use e.g. [MapProxy](https://mapproxy.org/) to provide your own WMS endpoint serving
either your own maps or proxying a 3rd party WMS/WMTS service.

- `~url` (`string`, default: `http://localhost:8080/wms`, **read-only**)
- `~version` (`string`, default: `1.1.1`, **read-only**)
- `~layer` (`string`, default: `WorldImagery`)
- `~srs` (`string`, default: `EPSG:4326`)\*

*\*While the `~srs` parameter can technically be changed, only EPSG:4326 is supported. 
The program will currently not work with any other SRS.*

## License

This package is released under the MIT license. See the `LICENSE.md` file included in this repository for more
information. Also see the
[SuperGlue license file](https://github.com/magicleap/SuperGluePretrainedNetwork/blob/master/LICENSE) for SuperGlue
licensing information.