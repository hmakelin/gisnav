#!/bin/bash
# This script removes unneeded plugins from the mavros_extras CMakeLists.txt
# file to speed up the build process.

# Remove all plugins from the add_library(mavros_extras_plugins SHARED ... ) block except gimbal_control.cpp
# 1. /src\/plugins\//: This matches lines containing src/plugins/.
# 2. {; /src\/plugins\/gimbal_control.cpp/!d; };: For these matched lines, delete all lines that do not contain src/plugins/gimbal_control.cpp.
sed -i '/src\/plugins\//{; /src\/plugins\/gimbal_control.cpp/!d; };' CMakeLists.txt
