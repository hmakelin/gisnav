ARG ROS_VERSION
FROM ros:${ROS_VERSION}

ARG ROS_VERSION

RUN apt-get update && \
    apt-get -y dist-upgrade

COPY . /opt/colcon_ws/src/gisnav/

RUN chmod +x /opt/colcon_ws/src/gisnav/docker/gisnav/entrypoint.sh

ENV FASTRTPS_DEFAULT_PROFILES_FILE \
    /opt/colcon_ws/src/gisnav/docker/gisnav/disable_shared_memory.xml

# Install GISNav dependencies
RUN cd /opt/colcon_ws/src/gisnav/gisnav && \
    rosdep update && \
    rosdep install --from-paths . -y -r --ignore-src && \
    apt-get -y install python3-pip && \
    pip install . && \
    pip install .[mock_gps_node] && \
    pip install .[dev]

# Install GDAL
RUN apt-get -y install software-properties-common && \
    add-apt-repository ppa:ubuntugis/ppa && \
    apt-get -y upgrade && \
    apt-get update && \
    apt-get -y install gdal-bin libgdal-dev

# Install GeographicLib
RUN apt-get -y install ros-${ROS_VERSION}-mavros ros-${ROS_VERSION}-mavros-extras && \
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    bash install_geographiclib_datasets.sh

# Custom ROS messages (px4_msgs and mavros mavlink gimbal protocl v2 support)
# sed edits px4_msgs CMakeLists.txt so that only SensorGps message is built
RUN cd /opt/colcon_ws/src/ && \
    git clone --branch release/1.14  \
      https://github.com/px4/px4_msgs.git && \
    sed -i 's/\*\.msg/SensorGps.msg/g' px4_msgs/CMakeLists.txt && \
    git clone --branch gimbal-protocol-v2-plugin \
      https://github.com/adinkra-labs/mavros_feature_gimbal-protocol-v2-plugin.git mavros && \
    sed -i 's/tf2_eigen.hpp/tf2_eigen.h/g'  \
      mavros/mavros_extras/src/plugins/gimbal_control.cpp

# Build
SHELL ["/bin/bash", "-c"]
RUN cd /opt/colcon_ws/ && \
    source /opt/ros/${ROS_VERSION}/setup.bash && \
    colcon build

# Upgrade & clean
RUN apt-get update && \
    apt-get -y upgrade && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# TODO: add better health check (are the core nodes publishing?)
HEALTHCHECK --interval=10s --timeout=3s \
  CMD /bin/bash -c "source /opt/ros/${ROS_VERSION}/setup.bash && \
  ros2 node list | grep -q cv_node && \
  ros2 node list | grep -q gis_node" || exit 1

WORKDIR /opt/colcon_ws/src/gisnav/gisnav
ENTRYPOINT ["../docker/gisnav/entrypoint.sh"]
