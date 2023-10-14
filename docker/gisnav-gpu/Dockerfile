ARG BASE_IMAGE
FROM ${BASE_IMAGE}

LABEL maintainer="Harri Makelin <hmakelin@protonmail.com>"

ARG ROS_VERSION
ENV ROS_VERSION=${ROS_VERSION}

COPY --chmod=755 entrypoint.sh /

# Install ROS and gisnav-gpu
RUN apt update && \
    apt -y upgrade && \
    apt -y install ros-humble-ros-base ros-${ROS_VERSION}-gisnav-gpu

ENTRYPOINT ["/entrypoint.sh"]
