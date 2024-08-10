ARG ROS_DISTRO
FROM ros:${ROS_DISTRO} as mavros-msgs

ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}

SHELL ["/bin/bash", "-c"]

# Install GeographicLib related dependencies
# Also add universe repository for ROS packages
# tcpdump for health checks
# Need mavros-extras for the gimbal_control plugin (mavros-extras v2.8+)
RUN apt-get update && \
    apt-get -y install software-properties-common wget curl tcpdump && \
    add-apt-repository ppa:ubuntugis/ppa && \
    add-apt-repository universe && \
    apt-get update && \
    apt-get -y install gdal-bin libgdal-dev geographiclib-tools ros-${ROS_DISTRO}-mavros ros-${ROS_DISTRO}-mavros-extras && \
    geographiclib-get-geoids egm96-5 && \
    rm -rf /var/lib/apt/lists/* && \
    apt-get clean

RUN mkdir -p /opt/colcon_ws/src

FROM mavros-msgs AS mavros
ARG ROS_DISTRO

WORKDIR /opt/colcon_ws/

COPY mavros/entrypoint.sh /

RUN chmod 755 /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

FROM mavros-msgs AS gisnav

ARG ROS_DISTRO
ENV ROS_DISTRO=${ROS_DISTRO}

WORKDIR /opt/colcon_ws/src/

# Copy package.xml only to install system dependencies
COPY ros/gisnav/package.xml gisnav/ros/gisnav/package.xml

# Install GISNav system dependencies - we will install
# the setup.py python dependencies later.
RUN cd gisnav/ros/gisnav && \
    rosdep update && \
    apt-get update && \
    rosdep install --from-paths . -y -r --ignore-src && \
    rm -rf /var/lib/apt/lists/* && \
    apt clean

# Build px4_msgs, add custom patch to only get SensorGps message
RUN git clone --branch release/1.14 https://github.com/px4/px4_msgs.git && \
    sed -i 's/\*\.msg/SensorGps.msg/g' px4_msgs/CMakeLists.txt && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd .. && \
    colcon build --packages-select px4_msgs

# Copy rest of gisnav ros2 package
COPY ros/ gisnav/ros

WORKDIR /opt/colcon_ws/src/gisnav/

# Install python dependencies, mainly torch and related CUDA deps
# Finally, build gisnav
RUN cd ros/gisnav && \
    pip install . .[nmea_node,uorb_node,dev] && \
    cd /opt/colcon_ws && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    source install/setup.bash && \
    colcon build --packages-select gisnav gisnav_msgs

COPY docker/mavros/gisnav/entrypoint.sh /

RUN chmod +x /entrypoint.sh

# Download LightGlue pretrained weights
RUN python3 -c  \
    "from kornia.feature import LightGlueMatcher, DISK; LightGlueMatcher('disk'); DISK.from_pretrained('depth')"

# Socat for bridging serial port to PX4 container when simulating
RUN apt-get update \
    && apt-get -y install socat \
    && rm -rf /var/lib/apt/lists/* \
    && apt clean

COPY docs/ docs/

# Node 18 for building the docs (Sphinx dependencies already in setup.py)
RUN curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.39.7/install.sh | bash \
    && source ~/.nvm/nvm.sh \
    && nvm install 20 \
    && nvm use 20 \
    && cd docs/vitepress \
    && npm install

COPY Makefile .pre-commit-config.yaml LICENSE.md README.md pyproject.toml /opt/colcon_ws/src/gisnav/

# .git needed to make pre-commit and git describe work. We create a shallow
# copy of the repo to avoid carrying over all of commit history.
COPY .git /tmp/gisnav/.git
RUN git clone --depth 1 /tmp/gisnav/.git /opt/colcon_ws/src/gisnav/.git && \
    rm -rf /tmp/gisnav/.git

ENTRYPOINT ["/entrypoint.sh"]
