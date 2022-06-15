# Introduction
> **WARNING:** Do not use this software for real drone flights. This software is untested and has only been demonstrated
> with [PX4](https://px4.io/) in a software-in-the-loop (SITL) simulation environment.

`gisnav` is a [ROS 2](https://docs.ros.org/) package that enables map-based visual navigation for airborne drones.

A GISNav node provides an *accurate* **global** position for an airborne drone by visually comparing frames from the 
drone's nadir-facing camera to a map of the drone's *approximate* global position retrieved from an underlying 
[GIS](https://en.wikipedia.org/wiki/Geographic_information_system) system.

# Quick Start
## Prerequisites
You will need a [tile based](https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames) endpoint for high-resolution aerial or 
satellite imagery with coverage over the demo area (San Carlos or KSQL airport in CA, USA) for the following example 
to work.

## 1. Run the simulation environment
See [README.md](https://gitlab.com/px4-ros2-map-nav/px4-ros2-map-nav-sim.git) at the `px4-ros2-map-nav-sim` repository
for more instruction on what to provide for build arguments - the strings below are examples.
```
xhost +

cd $HOME
git clone https://gitlab.com/px4-ros2-map-nav/px4-ros2-map-nav-sim.git
cd px4-ros2-map-nav-sim
docker-compose build \
    --build-arg MAPPROXY_TILE_URL="https://example.server.com/tiles/%(z)s/%(y)s/%(x)s" \
    --build-arg NVIDIA_DRIVER_MAJOR_VERSION=470 \
    .
docker-compose up -d
```
## 2. Clone this repository and dependencies
```
mkdir -p $HOME/px4_ros_com_ros2/src && cd "$_"
git clone https://github.com/PX4/px4_ros_com.git
git clone https://github.com/PX4/px4_msgs.git
git clone https://gitlab.com/px4-ros2-map-nav/python_px4_ros2_map_nav.git
```

## 3. Build your ROS 2 workspace
```
cd $HOME/px4_ros_com_ros2/src/px4_ros_com/scripts
./build_ros2_workspace.bash
```

## 4. Run the example node
```
cd $HOME/px4_ros_com_ros2
ros2 run python_px4_ros2_map_nav map_nav_node --ros-args --log-level info --params-file \
    src/python_px4_ros2_map_nav/params/typhoon_h480__ksql_airport.yml
```

# Generating Documentation
You can use Sphinx to generate the API documentation which will appear in the `docs/_build` folder.

Load the ROS workspace in your shell if you have not yet done so (ROS `foxy` in this example):
```
source /opt/ros/foxy/setup.bash
source install/setup.bash
```

Go to the `python_px4_ros2_map_nav` parent directory and install Sphinx:
```
cd $HOME/px4_ros_com_ros2/src/python_px4_ros2_map_nav/ && \
    pip3 install -r requirements-dev.txt
```

Then go to `docs/` folder and generate the HTML documentation:
```
cd docs/ && make html
```

# License
This software is released under the MIT license. See the `LICENSE.md` in this repository for more information. Also see
the [SuperGlue license file](https://github.com/magicleap/SuperGluePretrainedNetwork/blob/master/LICENSE) for SuperGlue
licensing information.