FROM microros/micro-ros-agent:foxy

LABEL maintainer="Harri Makelin <hmakelin@protonmail.com>"

RUN cd /uros_ws && \
    mkdir src && \
    cd src && \
    git clone https://github.com/px4/px4_msgs.git && \
    cd .. && \
    . /opt/ros/foxy/setup.sh && \
    colcon build --packages-select px4_msgs