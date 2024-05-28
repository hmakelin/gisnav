#!/bin/bash
set -e

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

exec "$@"
