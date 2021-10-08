# WMS Map Matching

## Introduction

This ROS2 package matches a nadir-facing video stream from an airborne drone's camera to a map of its location. This
package is designed to be simulated software-in-the-loop (SITL) with [PX4](https://docs.px4.io/master/) in
[Gazebo](https://gazebosim.org/).

Current implementation retrieves a map raster from a Web Map Service (WMS) endpoint for the vehicle's approximate
location as determined by GNSS and then matches it to frame from the video stream using a graph neural network based 
algorithm ([SuperGlue](https://github.com/magicleap/SuperGluePretrainedNetwork)).

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
be changed at runtime.

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