ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}

ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}

RUN apt-get update && \
    apt-get -y dist-upgrade && \
    apt-get -y install ros-${ROS_DISTRO}-rviz2 && \
    rm -rf /var/lib/apt/lists/* && \
    apt clean

COPY --chmod=755 * /

ENTRYPOINT ["/entrypoint.sh"]
