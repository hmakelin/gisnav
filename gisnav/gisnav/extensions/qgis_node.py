"""This module contains a :term:`ROS 2` node for interacting with an in-memory
PostgreSQL database. The node enables real-time data visualization in QGIS
by subscribing to specific GISNav :term:`core` output messages and updating the
database accordingly.

This node enables real-time visualization of data in QGIS without the need for
file I/O, aiding in development and debugging.

.. todo::
    Currently SensorGps message (PX4) only (implement GPSINPUT to support
    ArduPilot)
"""
from typing import Final, Union

import psycopg2
from rclpy.node import Node
from geographic_msgs.msg import BoundingBox
from px4_msgs.msg import SensorGps
from rclpy.qos import QoSPresetProfiles

from ..decorators import ROS
from .. import messaging
from ..constants import (
    ROS_NAMESPACE,
    GIS_NODE_NAME,
    ROS_TOPIC_RELATIVE_FOV_BOUNDING_BOX,
)
from .mock_gps_node import MockGPSNode


class QGISNode(Node):
    """:term:`ROS 2` node that subscribes to GISNav :term:`core` output
    messages and updates an in-memory PostgreSQL database for visualization in
    QGIS.
    """

    DATABASE_CONFIG: Final = {
        'dbname': 'gisnav_db',
        'user': 'gisnav',
        'password': 'gisnav',
        'host': 'localhost',
        'port': 5432,  # default PostgreSQL port is 5432
        'gps_table': 'gps_data_table',  # table for mock GPS data
        'bounding_box_table': 'bounding_box_table',  # table for bbox data
    }
    """Postgres database config used by this node."""

    def __init__(self, *args, **kwargs):
        """Class initializer

        :param args: Positional arguments to parent :class:`.Node` constructor
        :param kwargs: Keyword arguments to parent :class:`.Node` constructor
        """
        super().__init__(*args, **kwargs)

        # Initialize ROS subscriptions by calling the decorated properties once
        self.bounding_box
        self.sensor_gps

        # Connect to the PostgreSQL database
        self._db_connection = psycopg2.connect(self.DATABASE_CONFIG)

    def __del__(self):
        """Class destructor to close database connection"""
        if self._db_connection:
            self._db_connection.close()

    def _update_database(self, msg: Union[SensorGps, BoundingBox]) -> None:
        """Updates the PostgreSQL database with the received ROS 2 message data

        :param msg: :class:`px4_msgs.msg.SensorGps` or
            :class:`geographic_msgs.msg.BoundingBox` message containing
            data to insert into the database
        """
        with self._db_connection.cursor() as cursor:
            if isinstance(msg, SensorGps):
                query = """
                    INSERT INTO {gps_table} (latitude, longitude, altitude)
                    VALUES (%s, %s, %s);
                """.format(gps_table=self.DATABASE_CONFIG['gps_table'])
                cursor.execute(query, (msg.lat * 1e-7, msg.lon * 1e-7, msg.alt * 1e-3))

            elif isinstance(msg, BoundingBox):
                query = """
                    INSERT INTO {bbox_table} (min_latitude, min_longitude,
                                              max_latitude, max_longitude)
                    VALUES (%s, %s, %s, %s);
                """.format(bbox_table=self.DATABASE_CONFIG['bounding_box_table'])
                cursor.execute(query, (msg.min_latitude, msg.min_longitude,
                                       msg.max_latitude, msg.max_longitude))

            self._db_connection.commit()

    @property
    @ROS.max_delay_ms(messaging.DELAY_DEFAULT_MS)
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_FOV_BOUNDING_BOX.replace("~", GIS_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=_update_database,
    )
    def bounding_box(self) -> None:
        """:term:`Bounding box` of approximate :term:`vehicle` :term:`camera`
        :term:`FOV` location published via :attr:`.GisNode.bounding_box`
        """

    @property
    @ROS.max_delay_ms(messaging.DELAY_DEFAULT_MS)
    @ROS.subscribe(
        MockGPSNode.ROS_TOPIC_SENSOR_GPS,
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=_update_database,
    )
    def sensor_gps(self) -> None:
        """Subscribed mock :term:`GNSS` :term:`message` published by
        :class:`.MockGPSNode`
        """
