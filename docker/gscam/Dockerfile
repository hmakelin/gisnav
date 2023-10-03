ARG ROS_VERSION
FROM ros:${ROS_VERSION}

ARG ROS_VERSION
ENV ROS_VERSION=${ROS_VERSION}

COPY camera_calibration.yaml gscam_params.yaml /
COPY --chmod=755 entrypoint.sh /

RUN apt-get update && \
    apt-get -y dist-upgrade && \
    apt-get -y install gstreamer1.0-tools libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
      libgstreamer-plugins-good1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl && \
    apt-get -y install ros-${ROS_VERSION}-gscam && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

HEALTHCHECK --interval=10s --timeout=3s \
  CMD /bin/bash -c "source /opt/ros/${ROS_VERSION}/setup.bash && \
    ros2 node list | grep -q gscam_node" || exit 1

SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/entrypoint.sh"]
