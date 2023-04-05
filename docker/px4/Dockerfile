FROM px4io/px4-dev-ros2-foxy

LABEL maintainer="Harri Makelin <hmakelin@protonmail.com>"

# gstreamer and Gazebo plugins for getting video stream out of gazebo
RUN apt-get update && \
    apt-get -y dist-upgrade && \
    apt-get -y install ros-foxy-gazebo-ros-pkgs gstreamer1.0-plugins-bad  \
    gstreamer1.0-libav gstreamer1.0-gl

# Install PX4-Autopilot
RUN sudo apt-get -y install -o Dpkg::Options::="--force-overwrite" \
        libignition-common3-core-dev libignition-common3-av-dev expect && \
    git clone --branch v1.13.3 --single-branch \
        https://github.com/PX4/PX4-Autopilot.git --recursive && \
    cd PX4-Autopilot && \
    bash Tools/setup/ubuntu.sh --no-nuttx

# Add ROS camera plugin
COPY * /
RUN pip install lxml && \
    python3 merge_xml.py ros_camera.xml PX4-Autopilot/Tools/sitl_gazebo/models/typhoon_h480/typhoon_h480.sdf.jinja

# Make initial PX4 build (for faster startup in the future)
RUN cd PX4-Autopilot && \
    DONT_RUN=1 make px4_sitl gazebo_typhoon_h480__ksql_airport

# Apply configuration files
RUN cat 6011_typhoon_h480 >> PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/6011_typhoon_h480

RUN rm -rf /var/lib/apt/lists/* && \
    apt clean

WORKDIR PX4-Autopilot
