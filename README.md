[![Watch the GISNav demo video](https://img.youtube.com/vi/JAK2DPZC33w/0.jpg)](https://youtu.be/JAK2DPZC33w)

# Introduction
> **WARNING:** Do not use this software for real drone flights. This software is untested and has only been demonstrated
> in a software-in-the-loop (SITL) simulation environment.

GISNav is a ROS 2 package that enables map-based visual navigation for airborne drones **in a simulation environment**.

GISNav provides an *accurate* **global** position for an airborne drone by visually comparing frames from the drone's 
nadir-facing camera to a map of the drone's *approximate* global position retrieved from an underlying 
GIS system.

# Mock GPS Example
The below steps demonstrate how GISNav's example `MockGPSNode` ROS 2 node extension enables GNSS-free flight with PX4 
Autopilot's [Mission mode](https://docs.px4.io/v1.12/en/flight_modes/mission.html) in a Gazebo SITL simulation.

## Prerequisites
This example requires the following simulation setup:
* [PX4 Ubuntu Development Environment](https://docs.px4.io/v1.12/en/dev_setup/dev_env_linux_ubuntu.html) with 
[Fast DDS](https://docs.px4.io/v1.12/en/dev_setup/fast-dds-installation.html>) and 
[PX4-ROS 2 bridge](https://docs.px4.io/v1.12/en/ros/ros2_comm.html) 
[configured](https://docs.px4.io/v1.12/en/middleware/micrortps.html#supported-uorb-messages) for the following topics:
  * Send
    * ``VehicleGlobalPosition``
    * ``VehicleLocalPosition``
    * ``VehicleAttitude``
    * ``GimbalDeviceSetAttitude``
  * Receive
    * ``VehicleGpsPosition``
* A [WMS](https://en.wikipedia.org/wiki/Web_Map_Service) endpoint for high-resolution aerial or satellite imagery with 
coverage over the flight area (San Carlos KSQL airport in CA, USA)
* ROS 2 [gscam](https://index.ros.org/r/gscam/) or [gscam2](https://github.com/clydemcqueen/gscam2) 
package installed
* *Strongly recommended:* NVIDIA GPU and CUDA


## Setup GISNav
Clone the GISNav repository into your 
[PX4-ROS 2 workspace](https://docs.px4.io/main/en/ros/ros2_comm.html#build-ros-2-workspace) and install Python 
dependencies:
```bash
cd ~/px4_ros_com_ros2
mkdir src && cd "$_"
git clone https://github.com/hmakelin/gisnav.git
cd gisnav
python3 -m pip install -r requirements.txt
```

Install ROS dependencies and build the GISNav package:
```bash
cd ~/px4_ros_com_ros2
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select gisnav
```

## Setup LoFTR network with weights
Download the LoFTR submodule:
```bash
cd ~/px4_ros_com_ros2/src/gisnav && git submodule update LoFTR
```

Download the **dual-softmax** (_ds suffix) outdoor weights as described in the 
[LoFTR repo](https://github.com/zju3dv/LoFTR). Extract the `outdoor_ds.ckpt` from the .zip package and copy it into a 
new `weights` folder:
```bash
mkdir weights && cp ~/Downloads/outdoor_ds.ckpt ./weights
```

## Configure WMS endpoint
Configure the WMS ROS parameters in the `config/typhoon_h480__ksql_airport.yml` file. You must have a WMS endpoint for 
high resolution aerial or satellite imagery:
```yaml
mock_gps_node:
  ros__parameters:
    wms:
      url: 'http://localhost:8080/wms'
      version: '1.1.1'
      layers: ['Imagery']
      styles: ['']
      srs: 'EPSG:4326'
      image_format: 'image/jpeg'
      request_timeout: 10
```

## Run Gazebo simulation
You should run the `px4_sitl_rtps gazebo_typhoon_h480__ksql_airport` 
[build target](https://docs.px4.io/v1.12/en/dev_setup/building_px4.html#px4-make-build-targets):
```bash
cd ~/PX4-Autopilot
make px4_sitl_rtps gazebo_typhoon_h480__ksql_airport
```

You should then adjust the following parameters to make PX4 work with GISNav, either via the PX4 shell, via 
QGroundControl, or in the `~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/6011_typhoon_h480` file before 
making the build target:

> **THIS CONFIGURATION IS INTENDED FOR SIMULATION USE ONLY**
```
param set-default NAV_ACC_RAD 20.0
param set-default MPC_YAWRAUTO_MAX 10.0

param set-default COM_POS_FS_DELAY 5

param set-default EKF2_GPS_P_NOISE 10
param set-default EKF2_GPS_V_NOISE 3

param set-default SENS_GPS_MASK 2
```

> **NOTE:**  This is a sample configuration that seems to work, but you may experiment with the parameters. 
> 
> It is important to make the waypoint turns softer and/or to reduce the yaw rate especially if the camera has some 
> pitch (is not completely nadir-facing) to ensure the field of view does not move or rotate* too quickly for GISNav. 
> Otherwise GISNav may lose track of position for long enough for the position delay failsafe to trigger before GISNav 
> can find the drone again. Increasing the position failsafe delay helps if your GPU is a bit slower or GISNav for some 
> reason cannot produce a position estimate for a number of subsequent frames for one reason or another. However as a 
> failsafe parameter it should not be made unreasonably large.
> 
> The other parameters are mainly to increase tolerance for variation in the GPS position estimate. GISNav in its 
> default configuration seems to be more accurate in estimating vertical position than horizontal position, so the 
> example has lower tolerance for vertical position error.
> 
> *_camera yaw rotation speed may be less of an issue if a rotation agnostic neural network is used (not the case by 
> default)_


## Run microRTPS agent and gscam
Let GISNav receive telemetry from PX4:
```bash
micrortps_agent -t UDP
```

Launch `gscam` or `gscam2` to pipe the drone's video stream from Gazebo to ROS.

gscam:
```bash
cd ~/px4_ros_com_ros2
ros2 run gscam gscam_node --ros-args --params-file src/gisnav/test/assets/gscam_params.yaml \
    -p camera_info_url:=file://$PWD/src/gisnav/test/assets/camera_calibration.yaml
```

gscam2:
```bash
cd ~/px4_ros_com_ros2
ros2 run gscam2 gscam_main --ros-args --params-file src/gisnav/test/assets/gscam_params.yaml \
    -p camera_info_url:=file://$PWD/src/gisnav/test/assets/camera_calibration.yaml
```


## Upload mission file via QGroundControl
Start QGroundControl (installed into `~/Applications` in this example):
```bash
~/Applications/QGroundControl.AppImage
```

Use QGroundControl to upload the sample `test/assets/ksql_airport.plan` flight plan or make your own. Ensure that your 
your flight plan sets the gimbal to face nadir (90 degree pitch). Start the mission and proceed to the next step.


## Run GISNav mock GPS node
Run the example `mock_gps_node` ROS 2 node with your modified example configuration:
```bash
cd ~/px4_ros_com_ros2
ros2 run gisnav mock_gps_node --ros-args --log-level info \
    --params-file src/gisnav/config/typhoon_h480__ksql_airport.yaml
```
You should see GISNav start printing log messages into your console.


## Disable primary GPS
> **WARNING:** Do not attempt this on a real flight - simulation use only.

Wait until the drone has risen above the minimum altitude defined in the ``config/typhoon_h480__ksql_airport.yml`` 
file. You will see that GISNav will stop logging warning messages related to the altitude and start estimating the 
position of the drone instead. You should also see a visualization of the estimated field of view pop up.

By default PX4 is blending the primary GPS and the (secondary) mock GPS position estimate. You can then try disabling 
the primary GPS from your PX4 shell:
```
param set SENS_GPS_PRIME 1
```

The Typhoon H480 drone should now continue to complete its mission *GNSS-free* with GISNav substituting for GPS.

You can check if PX4 is receiving the mock GPS position estimates by typing the following in the PX4 shell:
```
listener vehicle_gps_position
```

If the printed GPS message has `selection: 1`, your PX4 is receiving the mock GPS node output as expected.

# Generating Documentation
You can use Sphinx to generate the GISNav documentation which will appear in the `docs/_build` folder:

```bash
cd ~/px4_ros_com_ros2/src/gisnav
python3 -m pip install -r requirements.txt
```

Then change to `docs/` folder and generate the HTML documentation:
```bash
cd docs && make html
```

# License
This software is released under the MIT license. See the `LICENSE.md` file for more information.
