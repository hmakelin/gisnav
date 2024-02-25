#!/bin/bash
set -e

source "/opt/ros/$ROS_VERSION/setup.bash"

# Move config files to shared volume if not yet done
mv /gscam_params.yaml /etc/gscam | echo "INFO: gscam_params.yaml not found on container - likely already moved to shared volume"
mv /camera_calibration.yaml /etc/gscam | echo "INFO: camera_calibration.yaml on container - likely already moved to shared volume"

exec "$@"
