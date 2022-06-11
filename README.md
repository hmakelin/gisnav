# Introduction
> **WARNING:** Do not use this software for real drone flights. This software is untested and has only been demonstrated
> with [PX4](https://px4.io/) in a software-in-the-loop (SITL) simulation environment.

`gisnav` is a [ROS 2](https://docs.ros.org/) package that enables map-based visual navigation for airborne drones.

A GISNav node provides an *accurate* **global** position for an airborne drone by visually comparing frames from the 
drone's nadir-facing camera to a map of the drone's *approximate* global position retrieved from an underlying 
[GIS](https://en.wikipedia.org/wiki/Geographic_information_system) system.

# Development Objectives
`gisnav` demonstrates a map-based visual global positioning for airborne drones that complements and improves on 
existing sensor fusion systems. It improves both local and global position and attitude estimate accuracy, and provides 
backup global positioning for [GNSS](https://en.wikipedia.org/wiki/Satellite_navigation)-denied flight.

## Guiding Principles
The following principles are used as design guidance in `gisnav` development:
* Complement and improve - but do not replace - existing local and global position and attitude estimation systems
* The natural or primary application is complementing GNSS in global positioning, while local position and attitude estimation or replacing GNSS (GNSS-denied flight) are secondary applications
* Prioritize maintainability and well-defined interfaces over premature optimization
* Support proven commercial off-the-shelf hardware platforms

## Constraints 
The [Guiding Principles](#guiding-principles) impose constraints on `gisnav`, namely:
* Currently `gisnav` is intended for simulation only
* Favorable operating terrain is strongly featured urban and semi-urban areas and traffic corridors (roads), not featureless natural terrain
* Monocular stabilized camera required
* Drone or UAV size, flight altitude or velocity constrained only to such degree that allows commercial GNSS receivers to work 
* Focus on good flight conditions - reasonable assumption for most commercial use cases which is where most develoment effort should be, niche applications will follow

## Development Focus
Taking the [Constraints](#constraints) into account, development focus should for example be in:
* ROS is baked in, but PX4 could be complemented by other flight control software options such as Ardupilot through Mavlink compatible interface
* Newer algorithms to improve accuracy, reliability or performance
* Making adoption easier for different kinds of hardware platforms or configurations

# Quick Start
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
You can use Sphinx to generate the API documentation which will appear in the `docs/_build` folder:
```
# Load the workspace in your shell if you have not yet done so
source /opt/ros/foxy/setup.bash
source install/setup.bash

# Go to docs/ folder, install Sphinx and generate html docs
cd docs/
pip3 install -r requirements-dev.txt
make html
```

# License
This software is released under the MIT license. See the `LICENSE.md` in this repository for more information. Also see
the [SuperGlue license file](https://github.com/magicleap/SuperGluePretrainedNetwork/blob/master/LICENSE) for SuperGlue
licensing information.