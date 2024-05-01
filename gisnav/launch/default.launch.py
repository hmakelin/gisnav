"""Launches GISNav with PX4 SITL simulation configuration

Allows providing :class:`NMEANode` serial ``port`` and ``baudrate`` parameters as launch
arguments with the same names. This is useful if we want to e.g. override the default
outbound serial port when doing development on a local machine where the default serial
port is most likely being used for something else.

The `make dev` command for example uses socat to create a virtual serial port
(pseudo-tty) and then passes it as launch argument to this launch script.
"""
import os
from typing import Final

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription  # type: ignore
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir
from launch_ros.actions import Node

_PACKAGE_NAME: Final = "gisnav"


def generate_launch_description():
    """Generates launch description with PX4 Fast DDS bridge adapter"""
    package_share_dir = get_package_share_directory(_PACKAGE_NAME)

    # Declare a launch argument for the serial port
    # We override this e.g. in local development where we use socat to generate us a
    # virtual serial port (pseudo-tty) as we assume the default /dev/ttyS1 is already
    # taken on the development machine
    ld = LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([ThisLaunchFileDir(), "/base.launch.py"])
            ),
        ]
    )

    # Add serial port and baudrate from launch args if provided
    port_config = LaunchConfiguration("port")
    # TODO: fix this - the defaults here should not override node's own default.
    #  We want to use the node's own defaults if the launch arguments here are
    #  not provided instead of re-declaring and overriding the defaults here in
    #  the launch file. Alternatively pass these as arguments not parameters to
    #  the node.
    ld.add_action(
        DeclareLaunchArgument(
            "port",
            default_value="/dev/ttyS1",
            description="Outbound serial port for NMEA node",
        )
    )
    ld.add_action(
        Node(
            package=_PACKAGE_NAME,
            name="nmea_node",
            namespace=_PACKAGE_NAME,
            executable="nmea_node",
            parameters=[
                os.path.join(package_share_dir, "launch/params/nmea_node.yaml"),
                {"port": port_config},
            ],
        )
    )
    return ld
