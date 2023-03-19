FROM ros:foxy

RUN apt-get update --fix-missing && \
    apt-get -y dist-upgrade

COPY . /opt/colcon_ws/src/gisnav/

RUN chmod +x /opt/colcon_ws/src/gisnav/docker/gisnav/entrypoint.sh

ENV FASTRTPS_DEFAULT_PROFILES_FILE \
    /opt/colcon_ws/src/gisnav/docker/gisnav/disable_shared_memory.xml

# Install GISNav dependencies
RUN cd /opt/colcon_ws/src/gisnav/ && \
    rosdep update && \
    rosdep install --from-paths . -y -r --ignore-src && \
    apt-get -y install python3-pip && \
    pip install -r requirements.txt && \
    pip install -r requirements-dev.txt

# Install GDAL
RUN cd /opt/colcon_ws/src/gisnav/ && \
    apt-get -y install software-properties-common && \
    add-apt-repository ppa:ubuntugis/ppa && \
    apt-get -y upgrade && \
    apt-get update && \
    apt-get -y install gdal-bin libgdal-dev

# Install GeographicLib
RUN cd /opt/colcon_ws/src/gisnav/ && \
    apt-get -y install ros-foxy-mavros ros-foxy-mavros-extras && \
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    bash install_geographiclib_datasets.sh

# Custom ROS messages
RUN cd /opt/colcon_ws/src/ && \
    git clone https://github.com/hmakelin/gisnav_msgs.git && \
    git clone https://github.com/px4/px4_msgs.git

# Build
SHELL ["/bin/bash", "-c"]
RUN cd /opt/colcon_ws/ && \
    source /opt/ros/foxy/setup.bash && \
    colcon build

# Upgrade & clean
RUN apt-get update && \
    apt-get -y upgrade && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/opt/colcon_ws/src/gisnav/docker/gisnav/entrypoint.sh"]
