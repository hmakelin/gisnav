ARG ROS_VERSION
FROM ros:${ROS_VERSION} as mavros-msgs

ARG ROS_VERSION
ENV ROS_VERSION=${ROS_VERSION}

LABEL maintainer="Harri Makelin <hmakelin@protonmail.com>"

# Combine apt update, upgrade and cleaning up in one layer
RUN apt update --fix-missing && \
    apt -y upgrade

SHELL ["/bin/bash", "-c"]

WORKDIR /opt/colcon_ws/src/

# Get mavros with gimbal protocol v2 plugin and build mavros_msgs only
# mavros_msgs is needed by both downstream images so it is already built here
# adds an additional custom patch to gps_input.cpp
RUN source /opt/ros/${ROS_VERSION}/setup.bash && \
    git clone --branch gimbal-protocol-v2-plugin \
      https://github.com/adinkra-labs/mavros_feature_gimbal-protocol-v2-plugin.git mavros && \
    rosdep update && \
    rosdep install --from-paths . --ignore-src -y && \
    cd .. && \
    colcon build --packages-ignore mavros mavros_extras libmavconn && \
    sed -i 's/rclcpp::Time last_pos_time;/rclcpp::Time last_pos_time{0, 0, get_clock()->get_clock_type()};/' \
      src/mavros/mavros_extras/src/plugins/gps_input.cpp


FROM mavros-msgs AS mavros
ARG ROS_VERSION

WORKDIR /opt/colcon_ws/

# mavros_msgs was built earlier so we ignore it here
RUN source /opt/ros/${ROS_VERSION}/setup.bash && \
    source install/setup.bash && \
    colcon build --packages-ignore mavros_msgs

RUN rm -rf /var/lib/apt/lists/* && \
    apt clean

COPY mavros/entrypoint.sh /

RUN chmod 755 /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

FROM mavros-msgs AS gisnav

ARG ROS_VERSION
ENV ROS_VERSION=${ROS_VERSION}

WORKDIR /opt/colcon_ws/src/

# Copy package.xml only to install system dependencies
COPY gisnav/package.xml gisnav/gisnav/package.xml

# Install GISNav system dependencies
# We install all extended dependencies so that we can do development work like
# building the full documentation inside the container. We will not install
# the setup.py python dependencies.
RUN cd gisnav/gisnav && \
    rosdep update && \
    rosdep install --from-paths . -y -r --ignore-src

# TODO: no need to install egm96-5 separately after pygeodesy was refactored out in gisnav-gpu branch?
# Install GIS related dependencies
RUN apt-get -y install software-properties-common wget curl && \
    add-apt-repository ppa:ubuntugis/ppa && \
    apt-get update && \
    apt-get -y install gdal-bin libgdal-dev geographiclib-tools && \
    geographiclib-get-geoids egm96-5 && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean

# Build px4_msgs, add custom patch to only get SensorGps message
RUN git clone --branch release/1.14 https://github.com/px4/px4_msgs.git && \
    sed -i 's/\*\.msg/SensorGps.msg/g' px4_msgs/CMakeLists.txt && \
    source /opt/ros/${ROS_VERSION}/setup.bash && \
    cd .. && \
    colcon build --packages-select px4_msgs

# Copy rest of gisnav ros2 package
COPY gisnav/ gisnav/

WORKDIR /opt/colcon_ws/src/gisnav

# Install python dependencies, mainly torch and related CUDA deps
# Finally, build gisnav
RUN pip install . .[mock_gps_node] .[qgis_node] .[dev] && \
    cd ../.. && \
    source /opt/ros/${ROS_VERSION}/setup.bash && \
    source install/setup.bash && \
    colcon build --packages-select gisnav

COPY docker/mavros/gisnav/entrypoint.sh /

RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

# TODO: proper health check - check that public facing topics are publishing
HEALTHCHECK --interval=10s --timeout=3s \
  CMD /bin/bash -c "source /opt/ros/${ROS_VERSION}/setup.bash && \
  ros2 node list | grep -q pose_node && \
  ros2 node list | grep -q gis_node" || exit 1
