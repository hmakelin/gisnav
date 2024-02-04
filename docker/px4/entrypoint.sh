# Use MAVLink router to forward packets to remote QGC and MAVROS
# https://docs.px4.io/main/en/simulation/#use-mavlink-router
export QGC_HOSTNAME=$(getent hosts gisnav-qgc-1 | awk '{ print $1 }')
export MAVROS_HOSTNAME=$(getent hosts gisnav-mavros-1 | awk '{ print $1 }')
echo "Setting up MAVLink router to QGC host ${QGC_HOSTNAME:-127.0.0.1} and MAVROS host ${MAVROS_HOSTNAME:-127.0.0.1}"
mavlink-routerd -e ${QGC_HOSTNAME:-127.0.0.1}:14550 -e ${MAVROS_HOSTNAME:-127.0.0.1}:14550 127.0.0.1:14550