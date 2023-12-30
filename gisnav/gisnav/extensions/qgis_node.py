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
from typing import Final, Union, Optional

import psycopg2
from rclpy.node import Node
from geographic_msgs.msg import BoundingBox
from px4_msgs.msg import SensorGps
from rclpy.qos import QoSPresetProfiles
from rclpy.timer import Timer
from rcl_interfaces.msg import ParameterDescriptor

from ..decorators import ROS, narrow_types
from .. import messaging
from ..constants import (
    ROS_NAMESPACE,
    BBOX_NODE_NAME,
    ROS_TOPIC_RELATIVE_FOV_BOUNDING_BOX,
)
from .mock_gps_node import MockGPSNode


class QGISNode(Node):
    """:term:`ROS 2` node that subscribes to GISNav :term:`core` output
    messages and updates an in-memory PostgreSQL database for visualization in
    QGIS.
    """

    ROS_D_SQL_POLL_RATE = 0.1
    """Default :term:`SQL` client connection attempt poll rate in Hz"""

    _ROS_PARAM_DESCRIPTOR_READ_ONLY: Final = ParameterDescriptor(read_only=True)
    """A read only ROS parameter descriptor"""

    DATABASE_CONFIG: Final = {
        'dbname': 'gisnav',
        'user': 'gisnav',
        'password': 'gisnav',
        'host': 'localhost',
        'port': 5432,  # default PostgreSQL port is 5432
    }
    """Postgres database config used by this node."""

    DEBUG_GPS_TABLE: Final = 'temp_gps_table'
    """Table name for mock GPS location"""

    DEBUG_BBOX_TABLE: Final = 'temp_bbox_table'
    """Table name for :term:`FOV` :term:`bounding box`"""

    def __init__(self, *args, **kwargs):
        """Class initializer

        :param args: Positional arguments to parent :class:`.Node` constructor
        :param kwargs: Keyword arguments to parent :class:`.Node` constructor
        """
        super().__init__(*args, **kwargs)

        # Initialize ROS subscriptions by calling the decorated properties once
        self.bounding_box
        self.sensor_gps

        sql_poll_rate = self.sql_poll_rate
        assert sql_poll_rate is not None
        self._db_connection = None  # TODO add type hint if possible
        self._connect_sql_timer: Optional[Timer] = self._create_connect_sql_timer(
            sql_poll_rate
        )

    @narrow_types
    def _create_connect_sql_timer(self, poll_rate: float) -> Timer:
        """Returns a timer that reconnects :term:`WMS` client when needed

        :param poll_rate: WMS connection status poll rate for the timer (in Hz)
        :return: The :class:`.Timer` instance
        """
        if poll_rate <= 0:
            error_msg = (
                f"WMS connection status poll rate must be positive ("
                f"{poll_rate} Hz provided)."
            )
            self.get_logger().error(error_msg)
            raise ValueError(error_msg)
        timer = self.create_timer(1 / poll_rate, self._try_sql_client_instantiation)
        return timer

    def _try_sql_client_instantiation(self) -> None:
        """Attempts to instantiate :attr:`._db_connection`

        Destroys :attr:`._connect_sql_timer` if instantiation is successful
        """

        @narrow_types(self)
        def _connect_sql(config: dict, poll_rate: float):
            try:
                assert self._db_connection is None
                self.get_logger().info("Connecting to SQL server...")
                self._db_connection = psycopg2.connect(**config)
                self.get_logger().info("SQL connection established.")

                # We have the SQL client instance - we can now destroy the timer
                assert self._db_connection is not None
                self._connect_sql_timer.destroy()
                self._create_tables()
            except psycopg2.OperationalError as _:  # noqa: F841
                # Expected error if no connection
                self.get_logger().error(
                    f"Could not instantiate SQL client due to connection error, "
                    f"trying again in {1 / poll_rate} seconds..."
                )
                assert self._db_connection is None
            except Exception as e:
                # TODO: handle other exception types
                self.get_logger().error(
                    f"Could not instantiate SQL client due to unexpected exception "
                    f"type ({type(e)}), trying again in {1 / poll_rate} seconds..."
                )
                assert self._db_connection is None

        if self._db_connection is None:
            _connect_sql(
                self.DATABASE_CONFIG, self.sql_poll_rate
            )

    def __del__(self):
        """Class destructor to close database connection"""
        if self._db_connection:
            self._db_connection.close()

    @property
    @ROS.parameter(ROS_D_SQL_POLL_RATE, descriptor=_ROS_PARAM_DESCRIPTOR_READ_ONLY)
    def sql_poll_rate(self) -> Optional[float]:
        """:term:`SQL` connection attempt poll rate in Hz"""

    def _create_tables(self):
        """Create temporary tables for storing SensorGps and BoundingBox data."""
        create_gps_table_query = f"""
            CREATE TEMPORARY TABLE IF NOT EXISTS {self.get_name()}_{self.DEBUG_GPS_TABLE} (
                id SERIAL PRIMARY KEY,
                latitude DOUBLE PRECISION,
                longitude DOUBLE PRECISION,
                altitude DOUBLE PRECISION
            ) ON COMMIT PRESERVE ROWS;
        """
        create_bbox_table_query = f"""
            CREATE TEMPORARY TABLE IF NOT EXISTS {self.get_name()}_{self.DEBUG_BBOX_TABLE} (
                id SERIAL PRIMARY KEY,
                min_latitude DOUBLE PRECISION,
                min_longitude DOUBLE PRECISION,
                max_latitude DOUBLE PRECISION,
                max_longitude DOUBLE PRECISION
            ) ON COMMIT PRESERVE ROWS;
        """
        with self._db_connection.cursor() as cursor:
            cursor.execute(create_gps_table_query)
            cursor.execute(create_bbox_table_query)
            self._db_connection.commit()

    def _update_database(self, msg: Union[SensorGps, BoundingBox]) -> None:
        """Updates the PostgreSQL database with the received ROS 2 message data

        :param msg: :class:`px4_msgs.msg.SensorGps` or
            :class:`geographic_msgs.msg.BoundingBox` message containing
            data to insert into the database
        """
        with self._db_connection.cursor() as cursor:
            if isinstance(msg, SensorGps):
                query = f"""
                    INSERT INTO {self.DEBUG_GPS_TABLE} (latitude, longitude, altitude)
                    VALUES (%s, %s, %s);
                """
                try:
                    cursor.execute(query, (msg.lat * 1e-7, msg.lon * 1e-7, msg.alt * 1e-3))
                except psycopg2.errors.UndefinedTable:
                    self.get_logger().error(
                        f"Table f{self.DEBUG_GPS_TABLE} does not exist. Cannot insert SensorGps message."
                    )

            elif isinstance(msg, BoundingBox):
                query = f"""
                    INSERT INTO {self.DEBUG_BBOX_TABLE} (min_latitude, min_longitude,
                                              max_latitude, max_longitude)
                    VALUES (%s, %s, %s, %s);
                """
                try:
                    cursor.execute(query, (msg.min_latitude, msg.min_longitude, msg.max_latitude, msg.max_longitude))
                except psycopg2.errors.UndefinedTable:
                    self.get_logger().error(
                        f"Table f{self.DEBUG_BBOX_TABLE} does not exist. Cannot insert BoundingBox message."
                    )

            self._db_connection.commit()

    @property
    # @ROS.max_delay_ms(messaging.DELAY_DEFAULT_MS)
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_FOV_BOUNDING_BOX.replace("~", BBOX_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def bounding_box(self) -> Optional[BoundingBox]:
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
    def sensor_gps(self) -> Optional[SensorGps]:
        """Subscribed mock :term:`GNSS` :term:`message` published by
        :class:`.MockGPSNode`
        """
