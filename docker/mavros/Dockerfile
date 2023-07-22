ARG ROS_VERSION
FROM ros:${ROS_VERSION}

ARG ROS_VERSION
ENV ROS_VERSION=${ROS_VERSION}

LABEL maintainer="Harri Makelin <hmakelin@protonmail.com>"

# Combine apt update, dist-upgrade and cleaning up in one layer
RUN apt update --fix-missing && \
    apt -y dist-upgrade && \
    apt -y install geographiclib-tools && \
    geographiclib-get-geoids egm96-5

WORKDIR /opt/colcon_ws

# Get mavros and egm96-5 geoid
RUN mkdir src && \
    cd src && \
    git clone --branch gimbal-protocol-v2-plugin \
      https://github.com/adinkra-labs/mavros_feature_gimbal-protocol-v2-plugin.git mavros && \
    rosdep update && \
    rosdep install --from-paths . --ignore-src -y

SHELL ["/bin/bash", "-c"]

# MAVROS extras and plugins
# This section is brittle (TODO: fix mavros version/commit) but should make our
# image leaner and quicker to build

# List required mavros plugins, extras, messages and services
# TODO: do not include servo_state_publisher
ENV ENABLED_PLUGINS="command local_position global_position home_position altitude"
ENV ENABLED_EXTRAS="gps_input hil gimbal_control"
ENV ENABLED_MSGS="Mavlink RCOut JointState GimbalDeviceInformation GimbalManagerSetPitchyaw GimbalManagerStatus GimbalManagerInformation HilControls HilActuatorControls HilStateQuaternion HilGPS HilSensor OpticalFlowRad RCIn GimbalDeviceAttitudeStatus GimbalManagerSetAttitude GimbalDeviceSetAttitude Altitude HomePosition HilGPS GPSINPUT"
ENV ENABLED_SRVS="CommandLong CommandAck CommandInt CommandBool CommandHome CommandTOL CommandTriggerControl CommandTriggerInterval CommandVtolTransition EndpointAdd EndpointDel GimbalGetInformation GimbalManagerCameraTrack GimbalManagerConfigure GimbalManagerPitchyaw GimbalManagerSetRoi"

# All available mavros plugins, extras, messages and services based on CMakeLists.txt
ENV ALL_PLUGINS="actuator_control altitude command dummy ftp geofence global_position home_position imu local_position manual_control mission_protocol_base nav_controller_output param rallypoint rc_io setpoint_accel setpoint_attitude setpoint_position setpoint_raw setpoint_trajectory setpoint_velocity sys_status sys_time waypoint wind_estimation"
ENV ALL_EXTRAS="3dr_radio adsb cam_imu_sync camera cellular_status companion_process_status debug_value distance_sensor esc_status esc_telemetry fake_gps gps_input gps_rtk gps_status guided_target hil landing_target log_transfer mag_calibration_status mocap_pose_estimate mount_control obstacle_distance odom onboard_computer_status optical_flow play_tune px4flow rangefinder terrain trajectory tunnel vfr_hud vibration vision_pose_estimate vision_speed_estimate wheel_odometry"
ENV ALL_MSGS="ADSBVehicle ActuatorControl Altitude AttitudeTarget CamIMUStamp CameraImageCaptured CellularStatus CommandCode CompanionProcessStatus DebugValue ESCInfo ESCInfoItem ESCStatus ESCStatusItem ESCTelemetry ESCTelemetryItem EstimatorStatus ExtendedState FileEntry GimbalDeviceAttitudeStatus GimbalDeviceInformation GimbalDeviceSetAttitude GimbalManagerInformation GimbalManagerSetAttitude GimbalManagerSetPitchyaw GimbalManagerStatus GPSINPUT GPSRAW GPSRTK GlobalPositionTarget HilActuatorControls HilControls HilGPS HilSensor HilStateQuaternion HomePosition LandingTarget LogData LogEntry MagnetometerReporter ManualControl Mavlink MountControl NavControllerOutput OnboardComputerStatus OpticalFlowRad OverrideRCIn Param ParamEvent ParamValue PlayTuneV2 PositionTarget RCIn RCOut RTCM RTKBaseline RadioStatus State StatusText TerrainReport Thrust TimesyncStatus Trajectory Tunnel VehicleInfo VfrHud Vibration Waypoint WaypointList WaypointReached WheelOdomStamped"
ENV ALL_SRVS="CommandAck CommandBool CommandHome CommandInt CommandLong CommandTOL CommandTriggerControl CommandTriggerInterval CommandVtolTransition EndpointAdd EndpointDel FileChecksum FileClose FileList FileMakeDir FileOpen FileRead FileRemove FileRemoveDir FileRename FileTruncate FileWrite GimbalGetInformation GimbalManagerCameraTrack GimbalManagerConfigure GimbalManagerPitchyaw GimbalManagerSetRoi LogRequestData LogRequestEnd LogRequestList MessageInterval MountConfigure ParamGet ParamPull ParamPush ParamSet ParamSetV2 SetMavFrame SetMode StreamRate VehicleInfoGet WaypointClear WaypointPull WaypointPush WaypointSetCurrent"

# This script comments out the lines corresponding to the disabled plugins and extras in CMakeLists.txt
RUN for plugin in $ALL_PLUGINS; do \
    if [[ $ENABLED_PLUGINS != *"$plugin"* ]]; then \
      sed -i "s|src/plugins/${plugin}.cpp|#&|g" src/mavros/mavros/CMakeLists.txt; \
    fi; \
    done && \
    for extra in $ALL_EXTRAS; do \
      if [[ $ENABLED_EXTRAS != *"$extra"* ]]; then \
        sed -i "s|src/plugins/${extra}.cpp|#&|g" src/mavros/mavros_extras/CMakeLists.txt; \
      fi; \
    done && \
    for msg in $ALL_MSGS; do \
    if [[ $ENABLED_MSGS != *"$msg"* ]]; then \
      sed -i "s|msg/${msg}.msg|#&|g" src/mavros/mavros_msgs/CMakeLists.txt; \
    fi; \
    done && \
    for srv in $ALL_SRVS; do \
      if [[ $ENABLED_SRVS != *"$srv"* ]]; then \
        sed -i "s|srv/${srv}.srv|#&|g" src/mavros/mavros_msgs/CMakeLists.txt; \
      fi; \
    done

# end MAVROS extras and plugins

# Custom MAVROS patches to make this work
# The clock type one is needed for ArduPilot SITL
RUN source /opt/ros/${ROS_VERSION}/setup.bash && \
    sed -i 's/tf2_eigen.hpp/tf2_eigen.h/g' \
      src/mavros/mavros_extras/src/plugins/gimbal_control.cpp && \
    sed -i 's/rclcpp::Time last_pos_time;/rclcpp::Time last_pos_time{0, 0, get_clock()->get_clock_type()};/' \
      src/mavros/mavros_extras/src/plugins/gps_input.cpp && \
    colcon build

COPY * /

RUN rm -rf /var/lib/apt/lists/* && \
    apt clean && \
    cd / && \
    chmod 755 disable_shared_memory.xml entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
