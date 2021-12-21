# Map Based Navigation for PX4-ROS 2
## Introduction
> **WARNING:** Do not use this software for a real use case. This software is untested and has only been demonstrated
> with PX4 in a software-in-the-loop (SITL) environment.

This repository contains a ROS 2 package and node which matches a nadir-facing video stream from an airborne drone's
camera to a map from the same location.

The ROS 2 node works by retrieving a map raster from a Web Map Service (WMS) endpoint for the vehicle's approximate
location as determined by existing sensors such as GPS, and then matches it to a frame from the video stream using a
graph neural network (GNN) based estimator ([SuperGlue](https://github.com/magicleap/SuperGluePretrainedNetwork)).

## Getting started
### 1. Clone, build, and run the simulation environment at $HOME
See [README.md](https://gitlab.com/px4-ros2-map-nav/px4-ros2-map-nav-sim.git) at the `px4-ros2-map-nav-sim` repository
for more instruction on what to provide for build arguments - the strings below are examples.
```
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
mkdir -p $HOME/px4_ros_com_ros2/src
cd $HOME/px4_ros_com_ros2/src
git clone https://github.com/PX4/px4_ros_com.git
git clone https://github.com/PX4/px4_msgs.git
git clone https://gitlab.com/px4-ros2-map-nav/python-px4-ros2-map-nav.git
```

### 3. Build your ROS 2 workspace
```
cd $HOME/px4_ros_com_ros2/src/px4_ros_com/scripts
./build_ros2_workspace.bash
```

### 4. Run the node
```
ros2 run px4-ros2-map-nav matching_node --ros-args --log-level info
```

## Advanced Configuration
TODO

## Repository Structure
This repository is structured as a `colcon` package:
```
.
├── config
│       └── params.yml                  # Configurable ROS 2 parameters
├── LICENSE.md
├── package.xml                         # Package metadata, also used by setup.py
├── README.md
├── requirements.txt                    # Python dependencies, used by setup.py
├── resource
│        └── wms_map_matching
├── setup.cfg
├── setup.py
├── test
│        ├── test_copyright.py
│        ├── test_flake8.py
│        └── test_pep257.py
└── wms_map_matching
    ├── __init__.py
    ├── matching_node.py                # Code for the ROS 2 node
    ├── superglue.py                    # SuperGlue adapter code
    └── util.py                         # Static functions and other utilities
```
## License
This software is released under the MIT license. See the `LICENSE.md` in this repository for more information. Also see
the [SuperGlue license file](https://github.com/magicleap/SuperGluePretrainedNetwork/blob/master/LICENSE) for SuperGlue
licensing information.