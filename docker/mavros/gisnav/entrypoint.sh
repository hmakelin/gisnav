#!/bin/bash
set -e

source "/opt/ros/$ROS_VERSION/setup.bash"
source "/opt/colcon_ws/install/setup.bash" --

# Needed for pip installed dev tools like pre-commit and sphinx-build
export PATH=/usr/lib/gisnav:$PATH

exec "$@"
