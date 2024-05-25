#!/bin/bash
set -e

# Use MAVLink router to forward packets to remote QGC and MAVROS
# https://docs.px4.io/main/en/simulation/#use-mavlink-router
# See description of how PX4 uses ports for simulation here:
# https://docs.px4.io/main/en/simulation/#default-px4-mavlink-udp-ports

# TODO: assumes that QGC is running in the same Docker bridge network as px4 -
#   not necessarily true
#export QGC_IP=$(getent hosts gisnav-qgc-1 | awk '{ print $1 }')
#echo "Setting up MAVLink router to QGC at ${QGC_IP:?empty or not set}"
#mavlink-routerd -e ${QGC_IP:?empty or not set}:14550 127.0.0.1:14550 &


export GISNAV_COMPANION_IP=$(getent hosts ${GISNAV_COMPANION_HOST:?empty or not set} | awk '{ print $1 }')
# Resolve IPv6 loopback address to IPv4 127.0.0.1
if [ "$GISNAV_COMPANION_IP" = "::1" ]; then
    GISNAV_COMPANION_IP="127.0.0.1"
fi
echo "Setting up MAVLink router to MAVROS at ${GISNAV_COMPANION_IP:?empty or not set}:${GISNAV_CMP_MAVROS_BIND_PORT:?empty or not set}"
mavlink-routerd -e ${GISNAV_COMPANION_IP:?empty or not set}:${GISNAV_CMP_MAVROS_BIND_PORT:?empty or not set} 127.0.0.1:14540 &

# TODO update NMEA support / socat - px4 now in host network to make uXRCE-DDS
# bridge work
# Listen to GISNav mock GPS messages on TCP port and bridge to serial port on
# px4 container (simulation host). Bridging serial ports over TCP is easier with
# Docker than e.g. bridging via virtual serial ports (pseudo-ttys) on Docker
# host
#socat tcp-listen:15000 pty,link=/dev/ttyS4 &
#socat tcp-listen:15000,reuseaddr,fork pty,raw,echo=0,link=/dev/ttyS4 &

# Setup uXRCE agent IP
# PX4 needs the IP as int32 - convert_ip.py script does the conversion
# https://docs.px4.io/main/en/middleware/uxrce_dds.html#starting-the-client
export UXRCE_DDS_AG_IP=$(python3 Tools/convert_ip.py ${GISNAV_COMPANION_IP:?empty or not set})
echo "Connecting uXRCE-DDS client with agent at ${GISNAV_COMPANION_IP:?empty or not set}:${UXRCE_DDS_PRT:?empty or not set} (integer IP: ${UXRCE_DDS_AG_IP:?empty or not set})."

# The rcS file should have been modified to use the GISNAV_COMPANION_IP env variable
# isntead of the hard-coded 127.0.0.1 for the uXRCE-DDS bridge
# Alternative approach below that restarts the client with the right host IP:
# Restart the uXRCE client in the main SITL startup script (rcS) with
# the correct DDS agent IP address (rcS hard codes 127.0.0.1)
#echo "uxrce_dds_client stop" >> ROMFS/px4fmu_common/init.d-posix/rcS
#echo "uxrce_dds_client start -h ${GISNAV_COMPANION_IP:?empty or not set} -p ${UXRCE_DDS_PRT:?empty or not set}" >> ROMFS/px4fmu_common/init.d-posix/rcS

exec "$@"
