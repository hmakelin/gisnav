ARG ROS_VERSION
FROM ros:${ROS_VERSION}

ARG ROS_VERSION
ENV ROS_VERSION=${ROS_VERSION}

LABEL maintainer="Harri Makelin <hmakelin@protonmail.com>"

COPY * /
RUN chmod 755 entrypoint.sh edit_mavros.sh

# Combine apt update, dist-upgrade and cleaning up in one layer
RUN apt update --fix-missing && \
    apt -y dist-upgrade && \
    apt -y install geographiclib-tools && \
    geographiclib-get-geoids egm96-5

WORKDIR /opt/colcon_ws

# Get mavros and egm96-5 geoid
RUN mkdir src && \
    cd src && \
    git clone --branch gimbal-protocol-v2-plugin \
      https://github.com/adinkra-labs/mavros_feature_gimbal-protocol-v2-plugin.git mavros && \
    rosdep update && \
    rosdep install --from-paths . --ignore-src -y

SHELL ["/bin/bash", "-c"]

# Custom MAVROS patches to make this work
# The clock type one is needed for ArduPilot SITL
# TODO: fix edit_mavros.sh so that mavros/local_position/pose topic
# orientation works (does not work with current script -> commented out)
RUN source /opt/ros/${ROS_VERSION}/setup.bash && \
    sed -i 's/rclcpp::Time last_pos_time;/rclcpp::Time last_pos_time{0, 0, get_clock()->get_clock_type()};/' \
      src/mavros/mavros_extras/src/plugins/gps_input.cpp && \
    #/edit_mavros.sh && \
    colcon build

RUN rm -rf /var/lib/apt/lists/* && \
    apt clean

ENTRYPOINT ["/entrypoint.sh"]
