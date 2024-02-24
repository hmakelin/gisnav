#!/bin/bash
set -e

source "/opt/ros/$ROS_VERSION/setup.bash"
source "/opt/colcon_ws/install/setup.bash" --

exec "$@"
