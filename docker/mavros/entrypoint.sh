#!/bin/bash
set -e

source "/opt/ros/foxy/setup.bash"
source "/colcon_ws/install/setup.bash" --

exec "$@"