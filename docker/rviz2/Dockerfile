FROM ros:foxy

RUN apt-get update && \
    apt-get -y dist-upgrade && \
    apt-get -y install ros-foxy-rviz2

COPY --chmod=755 * /

ENTRYPOINT ["/entrypoint.sh"]
