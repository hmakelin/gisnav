#!/bin/bash
set -e

source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/opt/colcon_ws/install/setup.bash" --

exec "$@"
