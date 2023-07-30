ARG ROS_VERSION

FROM px4io/px4-dev-ros2-${ROS_VERSION}
ARG ROS_VERSION

LABEL maintainer="Harri Makelin <hmakelin@protonmail.com>"

# Update and upgrade packages, then install required packages
RUN apt-get update && \
    apt-get -y dist-upgrade && \
    apt-get -y install ros-${ROS_VERSION}-gazebo-ros-pkgs gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl && \
    rm -rf /var/lib/apt/lists/* && \
    apt clean

# Clone and setup PX4-Autopilot
RUN git clone --branch v1.14.0-beta2 --single-branch \
        https://github.com/PX4/PX4-Autopilot.git && \
    cd PX4-Autopilot && \
    git -c submodule."Tools/simulation/jmavsim/jMAVSim".update=none \
        -c submodule."Tools/flightgear_bridge".update=none \
        -c submodule."Tools/simulation/jsbsim/jsbsim_bridge".update=none \
        submodule update --init --recursive && \
    bash Tools/setup/ubuntu.sh

# Explicitly add required \
WORKDIR PX4-Autopilot

COPY merge_xml.py merge_yaml.py dds_topics.yaml typhoon_h480_camera.xml iris_hitl_camera.xml iris_hitl_ksql_airport.world .

# Add ROS camera plugin for SITL and HIL simulation, setup ksql_airport.world for HIL
RUN pip install --no-cache-dir lxml && \
    python3 merge_xml.py typhoon_h480_camera.xml Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/typhoon_h480/typhoon_h480.sdf.jinja && \
    python3 merge_xml.py iris_hitl_camera.xml Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_hitl/iris_hitl.sdf && \
    python3 merge_xml.py iris_hitl_ksql_airport.world Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/hitl_iris.world

# Configure XRCE-DDS bridge
RUN python3 merge_yaml.py dds_topics.yaml src/modules/microdds_client/dds_topics.yaml

# Apply configuration files
#RUN cat 6011_typhoon_h480 >> ROMFS/px4fmu_common/init.d-posix/airframes/6011_gazebo-classic_typhoon_h480

# Make initial builds (for faster startup in the future)
RUN DONT_RUN=1 make px4_sitl gazebo-classic_typhoon_h480__ksql_airport
