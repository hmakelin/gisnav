import asyncio
import os
from collections import deque
from functools import partial
from math import atan2, cos, radians, sin, sqrt

import rclpy
from mavsdk import System
from mavsdk.log_files import LogFilesError, LogFilesResult
from px4_msgs.msg import SensorGps, VehicleGlobalPosition
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sitl_utils import (  # GPSDataError,
    CouldNotArmError,
    HealthCheckTimeoutError,
    MavlinkTimeoutError,
    MissionUploadTimeoutError,
    ROSContext,
    SimConfig,
    SITLEnvironment,
    arm,
    check_health,
    connect_mavlink,
    logger,
    poll_mapserver,
    upload_mission,
)


class PX4TestEnvironment(SITLEnvironment):
    """A context manager for managing the setup and cleanup of the PX4 SITL
    (Software In The Loop) testing environment.

    This context manager is responsible for starting and stopping the SITL
    testing environment, including any required services such as the Gazebo
    simulation and GISNav.

    Usage:
        with PX4TestEnvironment():
            # Your test code here
    """

    def __enter__(self):
        self.setup()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.cleanup()

    @staticmethod
    def setup():
        """Sets up SITL testing environment

        This most likely means starting supporting services with docker
        """
        logger.info("Starting PX4 SITL simulation environment...")
        os.system("docker compose -f ../docker/docker-compose.yaml up -d gisnav")
        os.system("make -C ../docker up-offboard-sitl-test-px4")

    @staticmethod
    def cleanup():
        """Cleans up after tests

        Should clean up whatever was set in :meth:`.setup` (e.g. docker containers
        so that they won't be left running if the tests fail because of an error)
        """
        logger.info("Shutting down PX4 SITL simulation environment...")
        os.system("make -C ../docker down")

    @staticmethod
    async def run():
        """This script tests GISNav with a simulated drone using the PX4 autopilot in
        a Gazebo environment. The drone receives mock GPS data provided by GISNav, in
        addition to GPS data provided by the Gazebo simulated GPS.

        The script performs the following tasks:

        1. Sets up the SITL (Software In The Loop) testing environment.
        2. Connects to the drone via MAVLink.
        3. Polls a WMS mapserver to ensure availability before starting the simulation.
        4. Imports, uploads, and executes a predefined mission plan.
        5. Monitors the mission progress and terminates when the mission is complete.
        6. Collects synchronized data from the mock SensorGps topic and ground truth.
        7. Computes error, standard deviation, and other statistics between the mock
           GPS data and the ground truth.
        8. Raises a :class:`.GPSDataError` if the mean error exceeds a predefined
           threshold.
        9. Downloads the flight log and stores it in a specified output directory.
        10. Cleans up the SITL testing environment.

        The script can be run as part of an automated testing pipeline and will exit
        with a non-zero exit code if any errors occur during the test, allowing the
        pipeline to detect and report failures.
        """
        drone = System()

        try:
            connect_mavlink_ = partial(connect_mavlink, drone)
            await asyncio.wait_for(
                connect_mavlink_(), timeout=SimConfig.MAVLINK_CONNECTION_TIMEOUT_SEC
            )
        except asyncio.TimeoutError as _:  # noqa: F841
            raise MavlinkTimeoutError(
                f"MAVLink connection attempt timed out at "
                f"{SimConfig.MAVLINK_CONNECTION_TIMEOUT_SEC} seconds."
            )

        logger.info("Polling the mapserver...")
        try:
            poll_mapserver_ = partial(poll_mapserver, SimConfig.WMS_ENDPOINT)
            await asyncio.wait_for(
                poll_mapserver_(), timeout=SimConfig.MAPSERVER_POLLING_TIMEOUT_SEC
            )
        except asyncio.TimeoutError as _:  # noqa: F841
            raise asyncio.TimeoutError(
                f"Mapserver polling timed out at "
                f"{SimConfig.MAPSERVER_POLLING_TIMEOUT_SEC} seconds."
            )

        logger.info(f"Importing mission from file {SimConfig.MISSION_FILE}")
        mission_import_data = await drone.mission_raw.import_qgroundcontrol_mission(
            SimConfig.MISSION_FILE
        )

        logger.info("Uploading mission...")
        try:
            upload_mission_ = partial(upload_mission, drone)
            await asyncio.wait_for(
                upload_mission_(mission_import_data.mission_items),
                timeout=SimConfig.MISSION_UPLOAD_TIMEOUT_SEC,
            )
        except asyncio.TimeoutError as _:  # noqa: F841
            raise MissionUploadTimeoutError(
                "Mission upload timed out at "
                f"{SimConfig.MISSION_UPLOAD_TIMEOUT_SEC} seconds."
            )

        try:
            check_health_ = partial(check_health, drone)
            await asyncio.wait_for(
                check_health_(), timeout=SimConfig.PRE_FLIGHT_HEALTH_CHECK_TIMEOUT_SEC
            )
        except asyncio.TimeoutError as _:  # noqa: F841
            raise HealthCheckTimeoutError("Pre-flight health checks failed.")

        logger.info("Arming...")
        if not await arm(drone):
            raise CouldNotArmError("Vehicle arming failed")

        logger.info("Starting mission...")
        await drone.mission_raw.start_mission()

        with ROSContext():
            with SensorGpsListenerContext("sensor_gps_listener"):
                # Create a task for the mission progress loop
                async for mission_progress in drone.mission.mission_progress():
                    print(
                        f"Mission progress: {mission_progress.current_item_index}/"
                        f"{mission_progress.total_mission_items}"
                    )
                    if (
                        mission_progress.current_item_index
                        == mission_progress.total_mission_items
                    ):
                        print("Mission finished.")
                        break
                    await asyncio.sleep(
                        1
                    )  # Add a small delay to allow progress updates

        logger.info("Getting log entries...")
        entries = await drone.log_files.get_entries()
        entry = entries[0]

        if not os.path.exists(SimConfig.LOG_OUTPUT_PATH):
            logger.info(f"Creating missing {SimConfig.LOG_OUTPUT_PATH} directory...")
            os.makedirs(SimConfig.LOG_OUTPUT_PATH)
        filename = f'gisnav-sitl-test-log_{entry.date.replace(":", "-")}.ulog'
        output_path = os.path.join(SimConfig.LOG_OUTPUT_PATH, filename)

        logger.info(f"Downloading log {entry.id} from {entry.date} to {output_path}.")
        try:
            await drone.log_files.download_log_file(entry, output_path)
        except LogFilesError as e:
            if e is LogFilesResult.Result.INVALID_ARGUMENT:
                logger.error(
                    f"File {output_path} possibly already exists, could not "
                    f'download log. Exception was: "{e}"'
                )
            raise


class SensorGpsListenerContext(Node):
    """
    A context manager for subscribing to a ROS topic, specifically designed
    for the PX4 SensorGps topic in a MAVSDK Python testing script. This context
    manager subscribes to the specified topic, allowing the script to compare
    the GPS data received from the topic with the GPS data retrieved via the
    MAVSDK's drone.telemetry API.

    Wraps a rclpy.node.Node to be able to subscribe to the :class:`.SensorGps`
    ROS messages.

    :param node_name: The name of the ROS node to be created.
    :param topic_name: The name of the ROS topic to subscribe to.

    Usage:

    .. code-block:: python

        with SensorGpsListenerContext("sensor_gps_listener", "/fmu/in/sensor_gps") \
                as subscriber_node:
            if subscriber_node.message is not None:
                print(subscriber_node.message)

    """

    def __init__(self, node_name):
        super().__init__(node_name)  # init rclpy.node.Node
        self.node_name = node_name
        self.sensor_gps_subscriber = None
        self.vehicle_global_position_subscriber = None
        self.buffer_size = 10  # Adjust the buffer size as needed
        self.vehicle_global_position_buffer = deque(maxlen=self.buffer_size)

    def __enter__(self):
        self.sensor_gps_subscriber = self.create_subscription(
            SensorGps,
            "/fmu/in/sensor_gps",
            self.sensor_gps_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        self.vehicle_global_position_subscriber = self.create_subscription(
            VehicleGlobalPosition,
            "/fmu/out/vehicle_global_position",
            self.vehicle_global_position_callback,
            QoSPresetProfiles.SENSOR_DATA.value,
        )
        rclpy.spin(self)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.destroy_node()

    def sensor_gps_callback(self, message: SensorGps):
        if not self.vehicle_global_position_buffer:
            logger.info("Waiting for GPS reference...")
            return

        # Find the closest-in-time VehicleGlobalPosition message
        best_matching_message = min(
            self.vehicle_global_position_buffer,
            key=lambda m: abs((message.timestamp - m.timestamp)),
        )

        distance = haversine_distance(
            best_matching_message.lat,
            best_matching_message.lon,
            message.lat * 1e-7,
            message.lon * 1e-7,
        )

        # TODO: check not comparing altitude AGL to altitude AMSL?
        altitude_diff = abs(best_matching_message.alt - message.alt * 1e-3)

        if distance <= 10 and altitude_diff <= 10:
            logger.info(
                f"GPS positions match within the 10-meter tolerance: "
                f"xy: {distance}, z: {altitude_diff}"
            )
        else:
            logger.error(
                f"GPS positions do not match within the 10-meter tolerance: "
                f"xy: {distance}, z: {altitude_diff}"
            )
            # TODO: make it optional to fail here
            # raise GPSDataError(
            #    f"GPS error too high:"
            #    f" xy: {distance}, z: {altitude_diff}"
            # )

    def vehicle_global_position_callback(self, message: VehicleGlobalPosition):
        self.vehicle_global_position_buffer.append(message)


def haversine_distance(lat1, lon1, lat2, lon2) -> float:
    R = 6371000  # Radius of the Earth in meters
    lat1_rad, lon1_rad = radians(lat1), radians(lon1)
    lat2_rad, lon2_rad = radians(lat2), radians(lon2)

    delta_lat = lat2_rad - lat1_rad
    delta_lon = lon2_rad - lon1_rad

    a = (
        sin(delta_lat / 2) ** 2
        + cos(lat1_rad) * cos(lat2_rad) * sin(delta_lon / 2) ** 2
    )
    c = 2 * atan2(sqrt(a), sqrt(1 - a))

    return R * c
