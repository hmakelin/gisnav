FROM microros/micro-ros-agent:foxy

# tcpdump for healthcheck
RUN apt-get update && \
    apt-get -y install tcpdump && \
    rm -rf /var/lib/apt/lists/* && \
    apt clean

# sed edits px4_msgs CMakeLists.txt so that only SensorGps message is built
RUN cd /uros_ws && \
    mkdir src && \
    cd src && \
    git clone -b release/1.14 https://github.com/px4/px4_msgs.git && \
    sed -i 's/\*\.msg/SensorGps.msg/g' px4_msgs/CMakeLists.txt && \
    cd .. && \
    . /opt/ros/foxy/setup.sh && \
    colcon build --packages-select px4_msgs
