import asyncio
import logging
import os
from abc import ABC, abstractmethod

import aiohttp
import rclpy
from mavsdk import System
from mavsdk.action import ActionError

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s]: %(message)s",
    datefmt="%Y-%m-%d %H:%M:%S",
)
logger = logging.getLogger(__name__)


class SimConfig:
    """
    A configuration class for the SITL (Software in the Loop) simulation.

    This class contains constants used throughout the script for configuring
    various aspects of the simulation, such as connection settings, mission file
    paths, timeouts, and data collection intervals.

    :ivar str SYS_ADDR: The system address for MAVLink connection.
    :ivar str MISSION_FILE: The path to the mission file.
    :ivar int MAVLINK_CONNECTION_TIMEOUT_SEC: The timeout for MAVLink
        connection in seconds.
    :ivar int MAPSERVER_POLLING_TIMEOUT_SEC: The timeout for MapServer
        connection in seconds.
    :ivar int PRE_FLIGHT_HEALTH_CHECK_TIMEOUT_SEC: The timeout for pre-flight
        health checks in seconds.
    :ivar str LOG_OUTPUT_PATH: The path for storing log output files.
    :ivar str WMS_ENDPOINT: The MapServer WMS endpoint URL
    :ivar int GPS_DATA_COLLECTION_INTERVAL_SEC: The interval for collecting
        GPS data in seconds.
    """

    SYS_ADDR = "udp://0.0.0.0:14550"
    MISSION_FILE = os.path.join(
        os.path.dirname(__file__), "../../../docker/qgc/ksql_airport_px4.plan"
    )
    # PX4 Gazebo container startup is too slow, fix once stable PX4 v1.14 is out
    MAVLINK_CONNECTION_TIMEOUT_SEC = 120
    MAPSERVER_POLLING_TIMEOUT_SEC = 10
    PRE_FLIGHT_HEALTH_CHECK_TIMEOUT_SEC = 30
    LOG_OUTPUT_PATH = os.path.join(os.path.dirname(__file__), "output")
    WMS_ENDPOINT = "http://localhost:80/wms"
    GPS_DATA_COLLECTION_INTERVAL_SEC = 5


class SITLTestError(Exception):
    """Base class for custom exceptions related to SITL tests."""


class MapServerError(SITLTestError):
    """Raised when the mapserver is not available."""


class MavlinkTimeoutError(SITLTestError):
    """Raised when the MAVLink connection attempt times out."""


class GPSDataError(SITLTestError):
    """Raised when the mean error exceeds the threshold."""


class HealthCheckTimeoutError(SITLTestError):
    """Raised when the health checks time out"""


class CouldNotArmError(SITLTestError):
    """Raised when the health checks time out"""


class ROSContext:
    """Context inside of which to run ROS nodes"""

    def __enter__(self):
        rclpy.init()

    def __exit__(self, exc_type, exc_val, exc_tb):
        rclpy.shutdown()


async def arm(drone: System) -> bool:
    """Attempt to arm the drone up to 3 times.

    :param drone: Instance of the connected drone System
    :return: A boolean indicating whether the drone was successfully armed or not
    """
    attempt = 0
    max_attempts = 5
    while attempt < max_attempts:
        logger.info(f"Arming attempt {attempt + 1}/{max_attempts}")
        try:
            await drone.action.arm()
            logger.info("Armed successfully")
            break
        except ActionError as e:
            logger.warning(f"Arming failed: {e}")
            attempt += 1
            if attempt == max_attempts:
                logger.error(f"Arming failed after {max_attempts} attempts. Exiting...")
                return False
            await asyncio.sleep(5)

    return True


async def check_wms_endpoint_availability(url):
    """Check if the WMS endpoint is available."""
    async with aiohttp.ClientSession() as session:
        try:
            async with session.get(url) as response:
                if response.status == 200:
                    return True
                else:
                    return False
        except aiohttp.ClientError:
            return False


async def poll_mapserver(url):
    """Poll the mapserver until a valid response is received."""
    while True:
        is_wms_available = await check_wms_endpoint_availability(url)
        if is_wms_available:
            logger.info("WMS endpoint is available.")
            break
        else:
            logger.info("WMS endpoint is not available. Retrying in 5 seconds...")
            await asyncio.sleep(5)


async def connect_mavlink(drone):
    """Connects to drone via MAVLink"""
    logger.info(
        f'Connecting to drone at "{SimConfig.SYS_ADDR}" '
        f"(timeout {SimConfig.MAVLINK_CONNECTION_TIMEOUT_SEC} sec)..."
    )
    await drone.connect(system_address=SimConfig.SYS_ADDR)
    async for state in drone.core.connection_state():
        # This might take a while assuming the Docker containers have not had
        # time to start yet
        if state.is_connected:
            logger.info("Connection successful.")
            break


async def check_health(drone):
    """Goes through (pre-flight) health checks"""
    logger.info(
        f"Going through health-checks "
        f"(timeout {SimConfig.PRE_FLIGHT_HEALTH_CHECK_TIMEOUT_SEC} sec)..."
    )
    async for health in drone.telemetry.health():
        if (
            health.is_global_position_ok
            and health.is_local_position_ok
            and health.is_home_position_ok
            and health.is_accelerometer_calibration_ok
            and health.is_gyrometer_calibration_ok
            and health.is_magnetometer_calibration_ok
            and health.is_armable
        ):
            logger.info("Health-checks OK")
            break
        await asyncio.sleep(1)


class SITLEnvironment(ABC):
    """A context manager for managing the setup and cleanup of the SITL
    (Software In The Loop) testing environment.

    This context manager is responsible for starting and stopping the SITL
    testing environment, including any required services such as the Gazebo
    simulation and GISNav.

    Usage:
        with SITLEnvironment():
            # Your test code here
    """

    def __enter__(self):
        self.setup()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.cleanup()

    @staticmethod
    @abstractmethod
    def setup():
        """Sets up SITL testing environment

        This most likely means starting supporting services with docker
        """

    @staticmethod
    @abstractmethod
    def cleanup():
        """Cleans up after tests

        Should clean up whatever was set in :meth:`.setup` (e.g. docker containers
        so that they won't be left running if the tests fail because of an error)
        """

    @staticmethod
    @abstractmethod
    async def run(self):
        """Executes the SITL test script"""
