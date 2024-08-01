ARG ROS_DISTRO
FROM ros:${ROS_DISTRO}

ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}

# tcpdump for health checks
RUN apt-get update && \
    apt-get -y dist-upgrade && \
    apt-get -y install gstreamer1.0-tools libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
      libgstreamer-plugins-good1.0-dev gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl && \
    apt-get -y install ros-${ROS_DISTRO}-gscam && \
    apt-get -y install tcpdump && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

HEALTHCHECK --interval=10s --timeout=3s \
  CMD /bin/bash -c "source /opt/ros/${ROS_DISTRO}/setup.bash && \
    ros2 node list | grep -q gscam_publisher" || exit 1

COPY camera_calibration.yaml gscam_params.yaml /
COPY --chmod=755 entrypoint.sh /

SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/entrypoint.sh"]
