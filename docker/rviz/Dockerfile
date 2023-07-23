ARG ROS_VERSION
FROM ros:${ROS_VERSION}

ARG ROS_VERSION
ENV ROS_VERSION=${ROS_VERSION}

RUN apt-get update && \
    apt-get -y dist-upgrade && \
    apt-get -y install ros-${ROS_VERSION}-rviz2 && \
    rm -rf /var/lib/apt/lists/* && \
    apt clean

COPY --chmod=755 * /

ENTRYPOINT ["/entrypoint.sh"]
