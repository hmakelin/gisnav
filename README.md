# Introduction
> **WARNING:** Do not use this software for real drone flights. This software is untested and has only been demonstrated
> with [PX4](https://px4.io/) in a software-in-the-loop (SITL) simulation environment.

GISNav is a [ROS 2](https://docs.ros.org/) package that enables map-based visual navigation for airborne drones.

GISNav provides an *accurate* **global** position for an airborne drone by visually comparing frames from the drone's 
nadir-facing camera to a map of the drone's *approximate* global position retrieved from an underlying 
[GIS](https://en.wikipedia.org/wiki/Geographic_information_system) system.

# Mock GPS Example
The below steps demonstrate how GISNav's example `MockGPSNode` ROS 2 node extension enables GNSS-free flight with PX4 
Autopilot's [Mission mode](https://docs.px4.io/v1.12/en/flight_modes/mission.html).

## Prerequisites
This example requires the following simulation setup:
* [PX4 Ubuntu Development Environment](https://docs.px4.io/v1.12/en/dev_setup/dev_env_linux_ubuntu.html) with
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
* ROS 2 [``gscam`` package](https://index.ros.org/r/gscam/) installed
* *Strongly recommended:* NVIDIA GPU and [CUDA](https://developer.nvidia.com/cuda-toolkit).

## 1. Source ROS 2 workspace
Build your ROS 2 workspace if you have not yet done so:
```bash
cd ~/px4_ros_com_ros2
./src/px4_ros_com/scripts/build_ros2_workspace.bash
```

Otherwise, you can source your workspace instead:
```bash
cd ~/px4_ros_com_ros2
source /opt/ros/foxy/setup.bash
source install/setup.bash
```

If you work with the ROS 2 workspace often, consider also sourcing it in your `~/.bashrc`:
```bash
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
echo "source ~/px4_ros_com_ros2/install/setup.bash" >> ~/.bashrc
```

## 2. Setup GISNav
Clone the GISNav repository into your ROS 2 workspace:
```bash
mkdir -p ~/px4_ros_com_ros2/src && cd "$_"
git clone https://github.com/hmakelin/gisnav.git
```

## 3. Setup LoFTR with weights
Download the LoFTR submodule and the **dual-softmax** (_ds suffix) outdoor weights as described in the 
[LoFTR repo](https://github.com/zju3dv/LoFTR). Extract the `outdoor_ds.ckpt` from the .zip package and copy it into a 
new `weights` folder:
```bash
cd ~/px4_ros_com_ros2/src/gisnav && \
    git submodule update LoFTR

# Download outdoor_ds.ckpt as described above
# ...

mkdir weights && cp ~/Downloads/outdoor_ds.ckpt ./weights
```

## 4. Configure WMS endpoint
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

## 5. Run GISNav
Run the included ``mock_gps_node`` ROS 2 node with the example configuration:
```bash
cd ~/px4_ros_com_ros2
ros2 run gisnav mock_gps_node --ros-args --log-level info \
    --params-file src/gisnav/config/typhoon_h480__ksql_airport.yaml
```

You should see GISNav start printing log messages into your console. It will not yet estimate your position because the 
Gazebo simulation is not running (see next step).

## 6. Run Gazebo simulation
Depending on how your PX4 is setup, you would run commands such as these to start the Gazebo simulation:
```bash
cd PX4-Autopilot
make px4_sitl_rtps gazebo_typhoon_h480__ksql_airport
```

## 7. Launch microRTPS agent and gscam
Let GISNav receive telemetry from PX4:
```bash
micrortps_agent -t UDP
```

Launch gscam to pipe the drone's video stream from Gazebo to ROS:
```bash
ros2 run gscam gscam_node --ros-args --params-file src/gisnav/test/assets/gscam_params.yaml \
    -p camera_info_url:=file://$PWD/src/gisnav//assets/camera_calibration.yaml
```

## 8. Upload mission file via QGroundControl
Start QGroundControl (installed into `~/Applications` in this example):
```bash
~/Applications/QGroundControl.AppImage
```

Then upload the example mission plan ``test/assets/ksql_airport.plan`` via QGroundControl and start the mission.

## 9. Disable primary GPS
Wait until the drone has risen above the minimum altitude defined in the ``config/typhoon_h480__ksql_airport.yml`` 
file. You will see that GISNav will stop logging warning messages related the altitude and start estimating the 
position of the drone instead.

After GISNav has started outputting info messages with the drone's position, try disabling the primary GPS from your 
PX4 console:
```commandline
pxh> param set SENS_GPS_PRIME 1
```

The Typhoon H480 drone should now continue to complete its mission *GNSS-free* with GISNav substituting for GPS.


# Generating Documentation
You can use Sphinx to generate the GISNav documentation which will appear in the `docs/_build` folder.

Source the ROS workspace in your shell (ROS `foxy` in this example):
```bash
source /opt/ros/foxy/setup.bash
source install/setup.bash
```

Change to `gisnav` parent directory and install Sphinx:
```bash
cd ~/px4_ros_com_ros2/src/gisnav
pip3 install -r requirements-dev.txt
```

Then change to `docs/` folder and generate the HTML documentation:
```bash
cd docs && make html
```

# License
This software is released under the MIT license. See the `LICENSE.md` file for more information.