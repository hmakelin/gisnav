#!/bin/bash
set -e

source "/opt/ros/$ROS_VERSION/setup.bash"

exec "$@"
