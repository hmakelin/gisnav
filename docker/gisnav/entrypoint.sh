#!/bin/bash
set -e

source "/opt/ros/foxy/setup.bash"
source "/opt/colcon_ws/install/setup.bash" --

exec "$@"
