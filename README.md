## Introduction
> **WARNING:** Do not use this software for a real use case. This software is untested and has only been demonstrated
> with PX4 in a software-in-the-loop (SITL) simulation environment.

`gisnav` contains a ROS 2 node which matches a nadir-facing video stream from an airborne drone's
camera to a map from the same location.

The node works by retrieving a map raster from a Web Map Service (WMS) endpoint for the vehicle's approximate
location as determined by existing sensors such as GPS, and then matches it to a frame from the video stream using a
graph neural network (GNN) based estimator ([SuperGlue](https://github.com/magicleap/SuperGluePretrainedNetwork)).

## Project Goals
The `gisnav` project aims to discover a reliable map-based navigation system for airborne drones that can be used 
to complement and possibly improve GPS accuracy of drones when navigating in urban or semi-urban environments. While 
map-based matching can be used to operate a drone in completely GNSS-denied environments, it is not seen as the primary
purpoes of the software, and map matching is seen as a complementary and improvmenet, but not a replacement for GNSS as
navigation solution.

* Complement and improve existing systems and GNSS (GPS) specifically, do not aim to replace them
* Favorable operating terrain is strongly featured urban and semi-urban areas and traffic corridors (roads), not featureless natural terrain
* Prioritize maintainability, scalability and future-proofing over premature optimization
* Discover a future-proof architecture that can later be optimized
* Monocular stabilized camera (gimbal required)
* Drone or UAV size, flight altitude is not constrained, assume velocity low enough for commercial GNSS receivers to work

As of now, `gisnav` is intended for simulation only, and tries to make it easy to swap in newer algorithms.

## Getting Started
### 1. Run the simulation environment
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
### 2. Clone this repository and dependencies
```
mkdir -p $HOME/px4_ros_com_ros2/src && cd "$_"
git clone https://github.com/PX4/px4_ros_com.git
git clone https://github.com/PX4/px4_msgs.git
git clone https://gitlab.com/px4-ros2-map-nav/python_px4_ros2_map_nav.git
```

### 3. Build your ROS 2 workspace
```
cd $HOME/px4_ros_com_ros2/src/px4_ros_com/scripts
./build_ros2_workspace.bash
```

### 4. Run the example node
```
cd $HOME/px4_ros_com_ros2
ros2 run python_px4_ros2_map_nav map_nav_node --ros-args --log-level info --params-file \
    src/python_px4_ros2_map_nav/params/typhoon_h480__ksql_airport.yml
```

## Advanced Configuration
TODO

## Generating API Documentation
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

## License
This software is released under the MIT license. See the `LICENSE.md` in this repository for more information. Also see
the [SuperGlue license file](https://github.com/magicleap/SuperGluePretrainedNetwork/blob/master/LICENSE) for SuperGlue
licensing information.