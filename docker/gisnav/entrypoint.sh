#!/bin/bash
set -e

source "/opt/ros/foxy/setup.bash"
source "$HOME/colcon_ws/install/setup.bash" --

exec "$@"
