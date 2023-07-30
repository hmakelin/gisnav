#!/bin/bash

# Define the plugins, extras, messages and services we want to enable
ENABLED_PLUGINS=("command" "param" "sys_status" "sys_time" "manual_control" "local_position" "global_position" "home_position" "altitude")
ENABLED_EXTRAS=("gps_input" "hil" "gimbal_control")
ENABLED_MSGS=("State" "FileEntry" "Waypoint" "EstimatorStatus" "ExtendedState" "StatusText" "VehicleInfo" "TimesyncStatus" "Param" "ParamEvent" "ParamValue" "ManualControl" "Mavlink" "RCOut" "GimbalDeviceInformation" "GimbalManagerSetPitchyaw" "GimbalManagerStatus" "GimbalManagerInformation" "HilControls" "HilActuatorControls" "HilStateQuaternion" "HilGPS" "HilSensor" "OpticalFlowRad" "RCIn" "GimbalDeviceAttitudeStatus" "GimbalManagerSetAttitude" "GimbalDeviceSetAttitude" "Altitude" "HomePosition" "GPSINPUT")
ENABLED_SRVS=("ParamGet" "ParamPull" "ParamPush" "ParamSet" "ParamSetV2" "StreamRate" "VehicleInfoGet" "MessageInterval" "SetMode" "CommandLong" "CommandAck" "CommandInt" "CommandBool" "CommandHome" "CommandTOL" "CommandTriggerControl" "CommandTriggerInterval" "CommandVtolTransition" "EndpointAdd" "EndpointDel" "GimbalGetInformation" "GimbalManagerCameraTrack" "GimbalManagerConfigure" "GimbalManagerPitchyaw" "GimbalManagerSetRoi")

# Path to mavros source in colcon workspace
MAVROS_PATH="src/mavros"

# Function to build new list from enabled elements
function build_new_list() {
  local -n list=$1
  local new_list=""
  for element in ${list[@]}; do
    new_list+="  $2/${element}.$3\n"
  done
  echo "$new_list"
}

# Use awk to replace the original plugin/message/service lists with our new ones
awk -v r="$(build_new_list ENABLED_PLUGINS src/plugins cpp)" '/add_library\(mavros_plugins SHARED/ {print; print r; f=1} /# \[\[\[end\]\]\] \(checksum: ccf56c1a56e9dccf8464483f7b1eab99\)/ {f=0} !f' $MAVROS_PATH/mavros/CMakeLists.txt > tmp && mv tmp $MAVROS_PATH/mavros/CMakeLists.txt
awk -v r="$(build_new_list ENABLED_EXTRAS src/plugins cpp)" '/add_library\(mavros_extras_plugins SHARED/ {print; print r; f=1} /# \[\[\[end\]\]\] \(checksum: a6dc36e6871deb96f56d61e2a1a38f37\)/ {f=0} !f' $MAVROS_PATH/mavros_extras/CMakeLists.txt > tmp && mv tmp $MAVROS_PATH/mavros_extras/CMakeLists.txt
awk -v r="$(build_new_list ENABLED_MSGS msg msg)" '/set\(msg_files/ {print; print r; f=1} /# \[\[\[end\]\]\] \(checksum: cf55cc727ab8f7a2823267cab0dd1bf3\)/ {f=0} !f' $MAVROS_PATH/mavros_msgs/CMakeLists.txt > tmp && mv tmp $MAVROS_PATH/mavros_msgs/CMakeLists.txt
awk -v r="$(build_new_list ENABLED_SRVS srv srv)" '/set\(srv_files/ {print; print r; f=1} /# \[\[\[end\]\]\] \(checksum: a011691dec330a2a89d35288ebf05c55\)/ {f=0} !f' $MAVROS_PATH/mavros_msgs/CMakeLists.txt > tmp && mv tmp $MAVROS_PATH/mavros_msgs/CMakeLists.txt
