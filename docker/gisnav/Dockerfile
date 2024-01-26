ARG ROS_VERSION
FROM ros:${ROS_VERSION}

ARG ROS_VERSION
ENV ROS_VERSION=${ROS_VERSION}

WORKDIR /opt/colcon_ws/src/

COPY . gisnav/

RUN apt-get update --fix-missing && \
    apt-get -y upgrade && \
    mv gisnav/docker/gisnav/entrypoint.sh / && \
    chmod +x /entrypoint.sh

# Install GISNav dependencies
# We install all extended dependencies so that we can do development work like
# building the full documentation inside the container
RUN cd gisnav/gisnav && \
    rosdep update && \
    rosdep install --from-paths . -y -r --ignore-src && \
    apt-get -y install python3-pip libpq-dev && \
    pip install . .[mock_gps_node] .[qgis_node] .[dev]

# TODO: no need to install egm96-5 separately after pygeodesy was refactored out in gisnav-gpu branch?
# Install GIS related dependencies
RUN apt-get -y install software-properties-common python3-pip wget && \
    add-apt-repository ppa:ubuntugis/ppa && \
    apt-get update && \
    apt-get -y install gdal-bin libgdal-dev geographiclib-tools && \
    geographiclib-get-geoids egm96-5 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

RUN git clone --branch release/1.14 https://github.com/px4/px4_msgs.git && \
    sed -i 's/\*\.msg/SensorGps.msg/g' px4_msgs/CMakeLists.txt && \
    git clone --branch gimbal-protocol-v2-plugin \
      https://github.com/adinkra-labs/mavros_feature_gimbal-protocol-v2-plugin.git mavros

# Build gisnav, px4_msgs and mavros_msgs
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/${ROS_VERSION}/setup.bash && \
    apt-get -y update && \
    rosdep update && \
    rosdep install --from-paths . -y -r --ignore-src && \
    cd .. && \
    colcon build --packages-ignore mavros mavros_extras libmavconn

# TODO: proper health check - check that public facing topics are publishing
HEALTHCHECK --interval=10s --timeout=3s \
  CMD /bin/bash -c "source /opt/ros/${ROS_VERSION}/setup.bash && \
  ros2 node list | grep -q pose_node && \
  ros2 node list | grep -q gis_node" || exit 1


WORKDIR /opt/colcon_ws/src/gisnav
ENTRYPOINT ["/entrypoint.sh"]
