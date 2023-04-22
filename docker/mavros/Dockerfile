FROM ros:foxy

LABEL maintainer="Harri Makelin <hmakelin@protonmail.com>"

RUN apt update --fix-missing && \
    apt -y dist-upgrade #&& \apt -y install ros-foxy-mavlink

RUN mkdir -p colcon_ws/src && \
    cd colcon_ws/src && \
    git clone --branch gimbal-protocol-v2-plugin \
      https://github.com/adinkra-labs/mavros_feature_gimbal-protocol-v2-plugin.git mavros

SHELL ["/bin/bash", "-c"]

RUN cd colcon_ws && \
    rosdep install --from-paths src --ignore-src -y && \
    src/mavros/mavros/scripts/install_geographiclib_datasets.sh

# Custom MAVROS patches to make this work
# The clock type one is needed for ArduPilot SITL
RUN cd colcon_ws && \
    source /opt/ros/foxy/setup.bash && \
    sed -i 's/tf2_eigen.hpp/tf2_eigen.h/g' \
      src/mavros/mavros_extras/src/plugins/gimbal_control.cpp && \
    sed -i 's/rclcpp::Time last_pos_time;/rclcpp::Time last_pos_time{0, 0, node->get_clock()->get_clock_type()};/' \
      src/mavros/mavros_extras/src/plugins/gps_input.cpp && \
    colcon build

RUN rm -rf /var/lib/apt/lists/* && \
    apt clean

COPY * /
RUN chmod 755 disable_shared_memory.xml && \
    chmod +x entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
