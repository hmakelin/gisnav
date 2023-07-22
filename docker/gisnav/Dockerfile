ARG ROS_VERSION
FROM ros:${ROS_VERSION} AS builder

ARG ROS_VERSION

WORKDIR /opt/colcon_ws/src/

COPY . gisnav/

RUN apt-get update --fix-missing && \
    apt-get -y dist-upgrade && \
    chmod +x gisnav/docker/gisnav/entrypoint.sh

# Install GISNav dependencies
RUN cd gisnav/gisnav && \
    rosdep update && \
    rosdep install --from-paths . -y -r --ignore-src && \
    apt-get -y install python3-pip && \
    pip install . .[mock_gps_node] .[dev]

# Install GIS related dependencies
RUN apt-get -y install software-properties-common python3-pip wget && \
    add-apt-repository ppa:ubuntugis/ppa && \
    apt-get update && \
    apt-get -y install gdal-bin libgdal-dev geographiclib-tools && \
    geographiclib-get-geoids egm96-5 && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

# Custom ROS messages (px4_msgs and mavros mavlink gimbal protocl v2 support)
# Edits px4_msgs CMakeLists.txt so that only SensorGps message is built
# Also comments out the lines corresponding to the disabled mavros_msgs CMakeLists.txt

# List required mavros messages
ENV ENABLED_MSGS="HilGPS GimbalDeviceAttitudeStatus GimbalManagerSetAttitude GimbalDeviceSetAttitude Altitude HomePosition HilGPS GPSINPUT"
# All available mavros messages
ENV ALL_MSGS="ADSBVehicle ActuatorControl Altitude AttitudeTarget CamIMUStamp CameraImageCaptured CellularStatus CommandCode CompanionProcessStatus DebugValue ESCInfo ESCInfoItem ESCStatus ESCStatusItem ESCTelemetry ESCTelemetryItem EstimatorStatus ExtendedState FileEntry GimbalDeviceAttitudeStatus GimbalDeviceInformation GimbalDeviceSetAttitude GimbalManagerInformation GimbalManagerSetAttitude GimbalManagerSetPitchyaw GimbalManagerStatus GPSINPUT GPSRAW GPSRTK GlobalPositionTarget HilActuatorControls HilControls HilGPS HilSensor HilStateQuaternion HomePosition LandingTarget LogData LogEntry MagnetometerReporter ManualControl Mavlink MountControl NavControllerOutput OnboardComputerStatus OpticalFlowRad OverrideRCIn Param ParamEvent ParamValue PlayTuneV2 PositionTarget RCIn RCOut RTCM RTKBaseline RadioStatus State StatusText TerrainReport Thrust TimesyncStatus Trajectory Tunnel VehicleInfo VfrHud Vibration Waypoint WaypointList WaypointReached WheelOdomStamped"

RUN git clone --branch release/1.14 https://github.com/px4/px4_msgs.git && \
    sed -i 's/\*\.msg/SensorGps.msg/g' px4_msgs/CMakeLists.txt && \
    git clone --branch gimbal-protocol-v2-plugin \
      https://github.com/adinkra-labs/mavros_feature_gimbal-protocol-v2-plugin.git mavros && \
    sed -i 's/tf2_eigen.hpp/tf2_eigen.h/g' mavros/mavros_extras/src/plugins/gimbal_control.cpp && \
    for msg in $ALL_MSGS; do \
    if [[ $ENABLED_MSGS != *"$msg"* ]]; then \
      sed -i "s|msg/${msg}.msg|#&|g" mavros/mavros_msgs/CMakeLists.txt; \
    fi; \
    done

# Find site-packages directory and assign its location to a build argument
RUN PYTHON_SITE_PACKAGES=$(python3 -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())") && \
    echo "PYTHON_SITE_PACKAGES=$PYTHON_SITE_PACKAGES" > /tmp/python_site_packages.env

# end MAVROS extras and plugins
# Build
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/${ROS_VERSION}/setup.bash && \
    cd .. && \
    colcon build --packages-ignore mavros mavros_extras libmavconn

# multi-stage build - runtime stage
FROM ros:${ROS_VERSION}

ARG ROS_VERSION
ENV ROS_VERSION=${ROS_VERSION}

# Load PYTHON_SITE_PACKAGES environment variable from a file
COPY --from=builder /tmp/python_site_packages.env /tmp/
RUN export PYTHON_SITE_PACKAGES=$(cat /tmp/python_site_packages.env | cut -d'=' -f2)

COPY --from=builder /opt/colcon_ws/install /opt/colcon_ws/install
COPY --from=builder /usr/share/GeographicLib /usr/share/GeographicLib
COPY --from=builder /opt/colcon_ws/src/gisnav/docker/gisnav/entrypoint.sh /entrypoint.sh
COPY --from=builder /opt/colcon_ws/src/gisnav/docker/gisnav/disable_shared_memory.xml /disable_shared_memory.xml
COPY --from=builder ${PYTHON_SITE_PACKAGES} ${PYTHON_SITE_PACKAGES}

ENV FASTRTPS_DEFAULT_PROFILES_FILE /disable_shared_memory.xml

HEALTHCHECK --interval=10s --timeout=3s \
  CMD /bin/bash -c "source /opt/ros/${ROS_VERSION}/setup.bash && \
  ros2 node list | grep -q cv_node && \
  ros2 node list | grep -q gis_node" || exit 1

WORKDIR /opt/colcon_ws/src/gisnav/gisnav
SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/entrypoint.sh"]
