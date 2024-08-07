ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}

ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}

SHELL ["/bin/bash", "-c"]

# tcpdump for health checks
RUN apt-get update && \
    apt-get -y dist-upgrade && \
    apt-get -y install tcpdump socat python3-pip && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

COPY opt/ /opt/

WORKDIR /opt/colcon_ws

RUN rosdep update && \
    apt-get update && \
    rosdep install --from-paths ./src -y -r --ignore-src && \
    rm -rf /var/lib/apt/lists/* && \
    apt clean

RUN pip3 install ./src/ubx_publisher && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build

COPY --chmod=755 entrypoint.sh /

SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/entrypoint.sh"]
