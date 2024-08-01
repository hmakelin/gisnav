ARG ROS_DISTRO
FROM ros:${ROS_DISTRO} as mavros-msgs

ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}

# Combine apt update, upgrade and cleaning up in one layer
RUN apt update --fix-missing && \
    apt -y upgrade

SHELL ["/bin/bash", "-c"]

WORKDIR /opt/colcon_ws/src/

# Install GeographicLib related dependencies
# Also add universe repository for ROS packages
# tcpdump for health checks
RUN apt-get -y install software-properties-common wget curl tcpdump && \
    add-apt-repository ppa:ubuntugis/ppa && \
    add-apt-repository universe && \
    apt-get update && \
    apt-get -y install gdal-bin libgdal-dev geographiclib-tools && \
    geographiclib-get-geoids egm96-5

# Get mavros with gimbal protocol v2 plugin and build mavros_msgs only
# mavros_msgs is needed by both downstream images so it is already built here
# adds an additional custom patch to gps_input.cpp
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    git clone --branch 2.8.0 https://github.com/mavlink/mavros.git && \
    rosdep update && \
    apt-get update && \
    rosdep install --from-paths . --ignore-src -y && \
    cd .. && \
    colcon build --packages-ignore mavros mavros_extras libmavconn && \
    sed -i 's/rclcpp::Time last_pos_time;/rclcpp::Time last_pos_time{0, 0, get_clock()->get_clock_type()};/' \
      src/mavros/mavros_extras/src/plugins/gps_input.cpp


FROM mavros-msgs AS mavros
ARG ROS_DISTRO

WORKDIR /opt/colcon_ws/

COPY mavros/filter_plugins.sh /opt/colcon_ws/src/mavros

# Build mavros and mavros_extras gimbal protocol v2 (gimbal_control) plugin
# We use the filter_plugins.sh script to disable unneeded plugins to significantly
# speed up build time and use of memory.
# mavros_msgs was built earlier so we ignore it here
RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && source install/setup.bash \
    && cd src/mavros \
    && ./filter_plugins.sh \
    && cd /opt/colcon_ws/ \
    && colcon build --packages-ignore mavros_msgs

RUN rm -rf /var/lib/apt/lists/* && \
    apt clean

COPY mavros/entrypoint.sh /

RUN chmod 755 /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

FROM mavros-msgs AS gisnav

ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}

WORKDIR /opt/colcon_ws/src/

# Copy package.xml only to install system dependencies
COPY ros/gisnav/package.xml gisnav/ros/gisnav/package.xml

# Install GISNav system dependencies - we will install
# the setup.py python dependencies later.
RUN cd gisnav/ros/gisnav && \
    rosdep update && \
    rosdep install --from-paths . -y -r --ignore-src && \
    rm -rf /var/lib/apt/lists/* && \
    apt clean

# Build px4_msgs, add custom patch to only get SensorGps message
RUN git clone --branch release/1.14 https://github.com/px4/px4_msgs.git && \
    sed -i 's/\*\.msg/SensorGps.msg/g' px4_msgs/CMakeLists.txt && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd .. && \
    colcon build --packages-select px4_msgs

# Copy rest of gisnav ros2 package
COPY ros/ gisnav/ros

WORKDIR /opt/colcon_ws/src/gisnav/

# Install python dependencies, mainly torch and related CUDA deps
# Finally, build gisnav
RUN cd ros/gisnav && \
    pip install . .[nmea_node,uorb_node,dev] && \
    cd /opt/colcon_ws && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source install/setup.bash && \
    colcon build --packages-select gisnav gisnav_msgs

COPY docker/mavros/gisnav/entrypoint.sh /

RUN chmod +x /entrypoint.sh

# Download LightGlue pretrained weights
RUN python3 -c  \
    "from kornia.feature import LightGlueMatcher, DISK; LightGlueMatcher('disk'); DISK.from_pretrained('depth')"

# Socat for bridging serial port to PX4 container when simulating
RUN apt-get update \
    && apt-get -y install socat \
    && rm -rf /var/lib/apt/lists/* \
    && apt clean

COPY docs/ docs/

# Node 18 for building the docs (Sphinx dependencies already in setup.py)
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash \
    && source ~/.nvm/nvm.sh \
    && nvm install 20 \
    && nvm use 20 \
    && cd docs/vitepress \
    && npm install

ENTRYPOINT ["/entrypoint.sh"]
