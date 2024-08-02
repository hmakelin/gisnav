ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}

ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}

# tcpdump for health checks
RUN apt-get update && \
    apt-get -y dist-upgrade && \
    apt-get -y install ros-humble-ublox tcpdump socat && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

COPY etc/ etc/
COPY --chmod=755 entrypoint.sh /

SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/entrypoint.sh"]
