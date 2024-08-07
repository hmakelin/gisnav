#!/bin/bash
set -e

source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/opt/colcon_ws/install/setup.bash"

echo "Setting up socat bridge from serial output to simulator container over TCP port 15000..."
socat pty,link=/tmp/gisnav-pty-link,raw,echo=0 tcp:${GISNAV_FCU_URL:?empty or not set}:${SOCAT_BRIDGE_PORT:?empty or not set} || (echo "Could not establish serial-to-TCP bridge. Is the SITL simulation container running?"; exit 1) &
sleep 3  # Give socat time to create the pty
echo PTS device created at: `readlink /tmp/gisnav-pty-link`
echo "Launching GISNav locally..."

#socat pty,link=/tmp/gisnav-pty-link,raw,echo=0 tcp:${GISNAV_FCU_URL:?empty or not set}:${SOCAT_BRIDGE_PORT:?empty or not set} || (echo "Could not establish serial-to-TCP bridge. Is the SITL simulation container running?"; exit 1) &
#sleep 3  # Give socat time to create the pty
#echo PTS device created at: `readlink /tmp/gisnav-pty-link`

exec "$@"
