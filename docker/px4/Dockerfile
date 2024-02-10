FROM ubuntu:focal

LABEL maintainer="Harri Makelin <hmakelin@protonmail.com>"

# Update and upgrade packages, then install required packages
RUN apt-get update && \
    apt-get -y upgrade && \
    DEBIAN_FRONTEND=noninteractive apt-get -y install git python3-pip lsb-release sudo wget gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl && \
    rm -rf /var/lib/apt/lists/* && \
    apt clean

# Clone and setup PX4-Autopilot
RUN git clone --branch v1.14.0 --single-branch --depth 1 \
        https://github.com/PX4/PX4-Autopilot.git && \
    cd PX4-Autopilot && \
    git -c submodule."Tools/simulation/jmavsim/jMAVSim".update=none \
        -c submodule."Tools/flightgear_bridge".update=none \
        -c submodule."Tools/simulation/jsbsim/jsbsim_bridge".update=none \
        -c submodule."platforms/nuttx/NuttX/apps".update=none \
        -c submodule."platforms/nuttx/NuttX/nuttx".update=none \
        submodule update --init --recursive && \
    bash Tools/setup/ubuntu.sh

# Build and install MAVLink router
RUN cd $HOME && \
    git clone --branch v3 --depth 1 https://github.com/mavlink-router/mavlink-router.git && \
    cd mavlink-router && \
    git submodule update --init --recursive && \
    apt install -y git ninja-build pkg-config gcc g++ systemd && \
    pip3 install meson>=0.55 && \
    meson setup build . && \
    ninja -C build && \
    ninja -C build install

WORKDIR PX4-Autopilot

COPY . .

# Setup ksql_airport.world for HIL
# empy must be version 3.3.4, see below commit
# https://github.com/PX4/PX4-containers/commit/80df74db476adbf7a71d08c9b392023756cc1bb0
RUN pip install --no-cache-dir lxml empy==3.3.4 && \
    python3 merge_xml.py iris_hitl_ksql_airport.world Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/hitl_iris.world

# Configure XRCE-DDS bridge
RUN python3 merge_yaml.py dds_topics.yaml src/modules/uxrce_dds_client/dds_topics.yaml

# Apply configuration files
#RUN cat 6011_typhoon_h480 >> ROMFS/px4fmu_common/init.d-posix/airframes/6011_gazebo-classic_typhoon_h480

# The hostname of the gscam service in the network is assumed to be gisnav-gscam-1
ENV PX4_VIDEO_HOST_IP=gisnav-gscam-1

RUN mkdir -p /etc/mavlink-router && \
    mv etc/mavlink-router/main.conf /etc/mavlink-router/main.conf && \
    chmod +x ./entrypoint.sh

# Make initial builds (for faster startup in the future)
RUN DONT_RUN=1 make px4_sitl gazebo-classic_typhoon_h480__ksql_airport

# No health check - if simulation dies makes no sense to restart via autoheal

ENTRYPOINT ["./entrypoint.sh"]
