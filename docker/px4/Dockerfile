FROM ubuntu:focal

# Update and upgrade packages, then install required packages
RUN apt-get update && \
    apt-get -y upgrade && \
    DEBIAN_FRONTEND=noninteractive apt-get -y install git libnss-mdns socat python3-pip lsb-release sudo wget gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl && \
    rm -rf /var/lib/apt/lists/* && \
    apt clean

# Clone and setup PX4-Autopilot
RUN git clone --branch v1.14.2 --single-branch --depth 1 \
        https://github.com/PX4/PX4-Autopilot.git && \
    cd PX4-Autopilot && \
    git -c submodule."Tools/simulation/jmavsim/jMAVSim".update=none \
        -c submodule."Tools/flightgear_bridge".update=none \
        -c submodule."Tools/simulation/jsbsim/jsbsim_bridge".update=none \
        #-c submodule."platforms/nuttx/NuttX/apps".update=none \
        #-c submodule."platforms/nuttx/NuttX/nuttx".update=none \
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

# Do not copy Dockerfile so that it can be edited without invalidating this
# layer
COPY 6011_typhoon_h480 merge_xml.py merge_yaml.py dds_topics.yaml iris_hitl_ksql_airport.world iris_hitl_camera.xml etc .

# Setup ksql_airport.world for HIL, and add camera plugin for hitl_iris model
RUN pip install --no-cache-dir lxml empy==3.3.4 \
    && python3 merge_xml.py Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds/hitl_iris.world iris_hitl_ksql_airport.world \
    && python3 merge_xml.py Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/iris_hitl/iris_hitl.sdf iris_hitl_camera.xml --camera

# Configure XRCE-DDS bridge
# Increase ping timeout and retries to ensure we do not disconnect too
# early (running in separate containers so expect some latency and temporary
# unavailability of the micro-ros agent while it starting)
RUN python3 merge_yaml.py dds_topics.yaml src/modules/uxrce_dds_client/dds_topics.yaml \
    && sed -i -e 's/500_ms/1000_ms/g' -e 's/num_pings_missed > 2/num_pings_missed > 5/g' src/modules/uxrce_dds_client/uxrce_dds_client.cpp

# Apply configuration files
RUN cat 6011_typhoon_h480 >> ROMFS/px4fmu_common/init.d-posix/airframes/6011_gazebo-classic_typhoon_h480

# Look for secondary GPS i.e. GISNav (GPS 2)
# Use first unassigned available serial port, i.e. /dev/ttyS4
# baudrate 9600 default
# https://docs.px4.io/v1.14/en/hardware/serial_port_mapping.html#default-px4board
#RUN echo "gps start -e /dev/ttyS4 -p nmea -d /dev/ttyS4 -b 115200 -g 115200" >> ROMFS/px4fmu_common/init.d-posix/rcS \
#    && echo "CONFIG_BOARD_SERIAL_GPS1=\"/dev/ttyS4\"" >> boards/px4/sitl/default.px4board \
#    && echo "CONFIG_BOARD_SERIAL_GPS2=\"/dev/ttyS4\"" >> boards/px4/sitl/default.px4board

# TODO: configure GPS serial ports in environment variables
# Cube Orange: GPS 1 -> ttyS2, GPS 2 --> ttyS5
RUN echo "gps start -d /dev/ttyS2 -e /dev/ttyS5 -p ubx" >> ROMFS/px4fmu_common/init.d-posix/rcS \
    && echo "CONFIG_BOARD_SERIAL_GPS1=\"/dev/ttyS2\"" >> boards/px4/sitl/default.px4board \
    && echo "CONFIG_BOARD_SERIAL_GPS2=\"/dev/ttyS5\"" >> boards/px4/sitl/default.px4board

# Fix uXRCE-DDS bridge host (uXRCE-DDS middleware not on localhost)
# GISNAV_COMPANION_IP needs to be set in entrypoint.sh
RUN sed -i "s/127\.0\.0\.1/\$GISNAV_COMPANION_IP/" ROMFS/px4fmu_common/init.d-posix/rcS

# Support building firmware for pixhawk and FMUK66 board specifically
COPY support_fmuk_nuttx_upload.sed .
RUN sudo apt-get update \
    && apt-get -y install gcc-arm-none-eabi \
    && sed -i -f support_fmuk_nuttx_upload.sed platforms/nuttx/cmake/upload.cmake

COPY entrypoint.sh .
RUN mkdir -p /etc/mavlink-router && \
    mv mavlink-router/main.conf /etc/mavlink-router/main.conf && \
    chmod +x ./entrypoint.sh

# Stream video to PX4_VIDEO_HOST_IP instead of hard-coded 127.0.0.1
RUN echo $PX4_VIDEO_HOST_IP && sed -i "s/<udpHost>127.0.0.1<\/udpHost>/<udpHost>${PX4_VIDEO_HOST_IP}<\/udpHost>/g" Tools/simulation/gazebo-classic/sitl_gazebo-classic/models/typhoon_h480/typhoon_h480.sdf.jinja

# Set this before building targets
ARG PX4_VIDEO_HOST_IP
ENV PX4_VIDEO_HOST_IP=${PX4_VIDEO_HOST_IP}

# Make initial builds (for faster startup in the future)
RUN DONT_RUN=1 make px4_sitl gazebo-classic_typhoon_h480__ksql_airport \
    && DONT_RUN=1 make px4_sitl_default gazebo-classic

# No health check - if simulation dies makes no sense to restart via autoheal
ENTRYPOINT ["./entrypoint.sh"]
