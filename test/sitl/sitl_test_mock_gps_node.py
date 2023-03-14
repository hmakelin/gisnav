#!/usr/bin/env python3
"""Script for testing :class:`.MockGPSNode` in SITL simulation"""
import asyncio
import os
import sys
from functools import partial

from mavsdk import System
from mavsdk.log_files import LogFilesError, LogFilesResult

DOCKER_CONTAINERS = [
    "gisnav-mapserver-1",
    "gisnav-px4-1",
    "gisnav-micro-ros-agent-1",
    "gisnav-gisnav-1",
]
SYS_ADDR = "udp://0.0.0.0:14550"
MISSION_FILE = os.path.join(os.path.dirname(__file__), "../assets/ksql_airport.plan")
MAVLINK_CONNECTION_TIMEOUT_SEC = 30
PRE_FLIGHT_HEALTH_CHECK_TIMEOUT_SEC = 30
LOG_OUTPUT_PATH = os.path.join(os.path.dirname(__file__), "output")


async def run():
    """Runs the SITL test"""
    drone = System()

    try:
        connect_mavlink_ = partial(connect_mavlink, drone)
        await asyncio.wait_for(
            connect_mavlink_(), timeout=MAVLINK_CONNECTION_TIMEOUT_SEC
        )
    except asyncio.TimeoutError as _:  # noqa: F841
        raise asyncio.TimeoutError(
            f"MAVLink connection attempt timed out at "
            f"{MAVLINK_CONNECTION_TIMEOUT_SEC} seconds."
        )

    # TODO: poll mapserver until a valid response is received

    print(f"Importing mission from file {MISSION_FILE}")
    mission_import_data = await drone.mission_raw.import_qgroundcontrol_mission(
        MISSION_FILE
    )

    print("Uploading mission...")
    await drone.mission_raw.upload_mission(mission_import_data.mission_items)

    try:
        check_health_ = partial(check_health, drone)
        await asyncio.wait_for(
            check_health_(), timeout=PRE_FLIGHT_HEALTH_CHECK_TIMEOUT_SEC
        )
    except asyncio.TimeoutError as _:  # noqa: F841
        raise asyncio.TimeoutError("Pre-flight health checks failed.")

    # TODO: figure out why this is needed
    await asyncio.sleep(5)

    print("Arming...")
    await drone.action.arm()

    print("Starting mission...")
    await drone.mission_raw.start_mission()

    async for mission_progress in drone.mission_raw.mission_progress():
        if 0 < mission_progress.current < mission_progress.total:
            print(
                f"Mission progress: "
                f"{mission_progress.current}/{mission_progress.total}"
            )
        else:
            print("Mission finished.")
            break

    print("Getting log entries...")
    entries = await drone.log_files.get_entries()
    entry = entries[0]

    if not os.path.exists(LOG_OUTPUT_PATH):
        print(f"Creating missing {LOG_OUTPUT_PATH} directory...")
        os.makedirs(LOG_OUTPUT_PATH)
    filename = f'gisnav-sitl-test-log_{entry.date.replace(":", "-")}.ulog'
    output_path = os.path.join(LOG_OUTPUT_PATH, filename)

    print(f"Downloading log {entry.id} from {entry.date} to {output_path}.")
    try:
        await drone.log_files.download_log_file(entry, output_path)
    except LogFilesError as e:
        if e is LogFilesResult.Result.INVALID_ARGUMENT:
            print(
                f"File {output_path} possibly already exists, could not "
                f'download log. Exception was: "{e}"'
            )
        else:
            raise


async def connect_mavlink(drone):
    """Connects to drone via MAVLink"""
    print(
        f'Connecting to drone at "{SYS_ADDR}" '
        f"(timeout {MAVLINK_CONNECTION_TIMEOUT_SEC} sec)..."
    )
    await drone.connect(system_address=SYS_ADDR)
    async for state in drone.core.connection_state():
        # This might take a while assuming the Docker containers have not had
        # time to start yet
        if state.is_connected:
            print("Connection successful.")
            break


async def check_health(drone):
    """Goes through (pre-flight) health checks"""
    print(
        f"Going through health-checks "
        f"(timeout {PRE_FLIGHT_HEALTH_CHECK_TIMEOUT_SEC} sec)..."
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
            print("Health-checks OK")
            break


def setup():
    """Sets up SITL testing environment

    This most likely means starting supporting services with docker
    """
    print("Starting SITL environment...")
    os.system("docker start " + " ".join(DOCKER_CONTAINERS))


def cleanup():
    """Cleans up after tests

    Should clean up whatever was set in :meth:`.setup` (e.g. docker containers
    so that they won't be left running if the tests fail because of an error)
    """
    print("Shutting down SITL environment...")
    for container in DOCKER_CONTAINERS:
        os.system(f"docker kill {container}")


if __name__ == "__main__":
    setup()
    loop = asyncio.get_event_loop()
    try:
        loop.run_until_complete(run())
    except Exception:
        # TODO: handle exceptions here
        raise
    finally:
        cleanup()
    sys.exit(0)
