#!/bin/bash
# This script removes unneeded plugins from the mavros and mavros_extras CMakeLists.txt
# files to speed up the build process. Run this script from the mavros repo root folder.

# 1. /src\/plugins\//: This matches lines containing src/plugins/.
# 2. {; /src\/plugins\/gimbal_control.cpp/!d; };: For these matched lines, delete all lines that do not contain src/plugins/gimbal_control.cpp.
cd mavros_extras
sed -i '/src\/plugins\//{; /src\/plugins\/gimbal_control.cpp/!d; };' CMakeLists.txt

# Same for core mavros package (we can remove a lot of packages but not all)
cd ../mavros
sed -i '/src\/plugins\//{; /src\/plugins\/command.cpp\|src\/plugins\/global_position.cpp\|src\/plugins\/home_position.cpp\|src\/plugins\/imu.cpp\|src\/plugins\/local_position.cpp\|src\/plugins\/mission_protocol_base.cpp\|src\/plugins\/nav_controller_output.cpp\|src\/plugins\/param.cpp\|src\/plugins\/sys_status.cpp\|src\/plugins\/sys_time.cpp/!d; };' CMakeLists.txt
