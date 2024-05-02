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

# Listen to GISNav mock GPS messages on TCP port and bridge to serial port on
# px4 container (simulation host). Bridging serial ports over TCP is easier with
# Docker than e.g. bridging via virtual serial ports (pseudo-ttys) on Docker
# host
#socat tcp-listen:15000 pty,link=/dev/ttyS4 &
socat tcp-listen:15000,reuseaddr,fork pty,raw,echo=0,link=/dev/ttyS4 &

exec "$@"
