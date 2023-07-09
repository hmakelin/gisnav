FROM px4io/px4-dev-ros2-foxy

LABEL maintainer="Harri Makelin <hmakelin@protonmail.com>"

# gstreamer and Gazebo plugins for getting video stream out of gazebo
RUN apt-get update && \
    apt-get -y dist-upgrade && \
    apt-get -y install ros-foxy-gazebo-ros-pkgs gstreamer1.0-plugins-bad  \
      gstreamer1.0-libav gstreamer1.0-gl

# Install PX4-Autopilot
RUN git clone --branch v1.14.0-beta2 --single-branch \
        https://github.com/PX4/PX4-Autopilot.git --recursive && \
    cd PX4-Autopilot && \
    bash Tools/setup/ubuntu.sh --no-nuttx

# Add ROS camera plugin for SITL and HIL simulation, setup ksql_airport.world for HIL
COPY * /
RUN pip install lxml && \
    python3 merge_xml.py typhoon_h480_camera.xml PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/typhoon_h480/typhoon_h480.sdf.jinja && \
    python3 merge_xml.py iris_hitl_camera.xml PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_hitl/iris_hitl.sdf && \
    python3 merge_xml.py iris_hitl_ksql_airport.world PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/hitl_iris.world

# Configure XRCE-DDS bridge
RUN python3 merge_yaml.py dds_topics.yaml PX4-Autopilot/src/modules/microdds_client/dds_topics.yaml

# Make initial builds (for faster startup in the future)
RUN cd PX4-Autopilot && \
    DONT_RUN=1 make px4_sitl gazebo-classic_typhoon_h480__ksql_airport

# Apply configuration files
#RUN cat 6011_typhoon_h480 >> PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/6011_gazebo-classic_typhoon_h480

RUN rm -rf /var/lib/apt/lists/* && \
    apt clean

WORKDIR PX4-Autopilot
