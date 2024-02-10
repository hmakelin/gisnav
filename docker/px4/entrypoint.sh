#!/bin/bash
set -e

# Use MAVLink router to forward packets to remote QGC and MAVROS
# https://docs.px4.io/main/en/simulation/#use-mavlink-router
# See description of how PX4 uses ports for simulation here:
# https://docs.px4.io/main/en/simulation/#default-px4-mavlink-udp-ports
export QGC_IP=$(getent hosts gisnav-qgc-1 | awk '{ print $1 }')
export MAVROS_IP=$(getent hosts gisnav-mavros-1 | awk '{ print $1 }')
echo "Setting up MAVLink router to QGC host ${QGC_IP:-127.0.0.1}"
mavlink-routerd -e ${QGC_IP:-127.0.0.1}:14550 127.0.0.1:14550 &
echo "Setting up MAVLink router to MAVROS host ${MAVROS_IP:-127.0.0.1}"
mavlink-routerd -e ${MAVROS_IP:-127.0.0.1}:14540 127.0.0.1:14540 &

# Setup uXRCE agent IP
# PX4 needs the IP as int32 - convert_ip.py script does the conversion
# https://docs.px4.io/main/en/middleware/uxrce_dds.html#starting-the-client
# UDP port 8888 used by default for SITL simulation
export UXRCE_AGENT_IP=$(getent hosts gisnav-micro-ros-agent-1 | awk '{ print $1 }')
export UXRCE_DDS_AG_IP=$(python3 Tools/convert_ip.py $UXRCE_AGENT_IP)
echo "Connecting uXRCE client with agent at ${UXRCE_AGENT_IP:-127.0.0.1} (${UXRCE_DDS_AG_IP})."
#echo "uxrce_dds_client start -h ${UXRCE_AGENT_IP}" >> ROMFS/px4fmu_common/init.d-posix/airframes/6011_gazebo-classic_typhoon_h480

# Restart the uXRCE client in the main SITL startup script (rcS) with
# the correct DDS agent IP address (rcS hard codes 127.0.0.1)
echo "uxrce_dds_client stop" >> ROMFS/px4fmu_common/init.d-posix/rcS
echo "uxrce_dds_client start -h ${UXRCE_AGENT_IP}" >> ROMFS/px4fmu_common/init.d-posix/rcS

exec "$@"
