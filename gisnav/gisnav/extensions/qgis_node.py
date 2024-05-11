"""This module contains :class:`.QGISNode`, an extension ROS node that
subscribes to and stores :attr:`.UORBNode.sensor_gps` messages in a PostGIS
database.

The node enables real-time data visualization in QGIS by via the PostGIS database.
"""
from typing import Final, Optional

import psycopg2
from px4_msgs.msg import SensorGps
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.timer import Timer

from .._decorators import ROS, narrow_types
from ..constants import ROS_TOPIC_SENSOR_GPS


class QGISNode(Node):
    """:term:`ROS 2` node that that subscribes to and stores
    :attr:`.UORBNode.sensor_gps` messages in a PostGIS database.
    """

    ROS_D_SQL_POLL_RATE = 0.1
    """Default for :attr:`.sql_poll_rate`"""

    _ROS_PARAM_DESCRIPTOR_READ_ONLY: Final = ParameterDescriptor(read_only=True)
    """A read only ROS parameter descriptor"""

    DATABASE_CONFIG: Final = {
        "dbname": "gisnav",
        "user": "gisnav",
        "password": "gisnav",
        "host": "gisnav-postgres-1",
        "port": 5432,  # default PostgreSQL port is 5432
    }
    """Postgres database config used by this node."""

    DEBUG_GPS_TABLE: Final = "gps_table"
    """Table name for mock GPS location"""

    def __init__(self, *args, **kwargs):
        """Class initializer

        :param args: Positional arguments to parent :class:`.Node` constructor
        :param kwargs: Keyword arguments to parent :class:`.Node` constructor
        """
        super().__init__(*args, **kwargs)

        # Initialize ROS subscriptions by calling the decorated properties once
        self.sensor_gps

        sql_poll_rate = self.sql_poll_rate
        assert sql_poll_rate is not None
        self._db_connection = None  # TODO add type hint if possible
        self._connect_sql_timer: Timer = self._create_connect_sql_timer(sql_poll_rate)

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
            _connect_sql(self.DATABASE_CONFIG, self.sql_poll_rate)

    def __del__(self):
        """Class destructor to close database connection"""
        if hasattr(self, "_db_connection") and self._db_connection is not None:
            self._db_connection.close()

    @property
    @ROS.parameter(ROS_D_SQL_POLL_RATE, descriptor=_ROS_PARAM_DESCRIPTOR_READ_ONLY)
    def sql_poll_rate(self) -> Optional[float]:
        """SQL connection attempt poll rate in Hz"""

    def _create_tables(self):
        """Create (and recreate if exist) temporary tables for storing
        :attr:`UORBNode.SensorGps` data as PostGIS geometries.
        """
        drop_gps_table_query = f"""
            DROP TABLE IF EXISTS {self.DEBUG_GPS_TABLE};
        """
        create_gps_table_query = f"""
            CREATE UNLOGGED TABLE {self.DEBUG_GPS_TABLE} (
                id SERIAL PRIMARY KEY,
                geom GEOMETRY(Point, 4326),
                altitude DOUBLE PRECISION
            );
        """

        with self._db_connection.cursor() as cursor:
            cursor.execute("CREATE EXTENSION IF NOT EXISTS postgis;")
            # Drop the tables if they exist - we do not want to persist old
            # debugging data
            cursor.execute(drop_gps_table_query)
            cursor.execute(create_gps_table_query)
            self._db_connection.commit()

    def _update_database(self, msg: SensorGps) -> None:
        """Updates the PostgreSQL database with the received ROS 2 message data

        :param msg: :class:`.SensorGps` message containing data to insert into the
            database
        """
        if self._db_connection is None:
            self.get_logger().error(
                f"SQL client not yet instantiated, could not insert message: {msg}."
            )
            return None

        with self._db_connection.cursor() as cursor:
            if isinstance(msg, SensorGps):
                query = f"""
                    INSERT INTO {self.DEBUG_GPS_TABLE} (geom, altitude)
                    VALUES (ST_SetSRID(ST_MakePoint(%s, %s), 4326), %s);
                """
                try:
                    cursor.execute(
                        query, (msg.lon * 1e-7, msg.lat * 1e-7, msg.alt * 1e-3)
                    )
                except psycopg2.errors.UndefinedTable:
                    self.get_logger().error(
                        f"Table f{self.DEBUG_GPS_TABLE} does not exist. "
                        f"Cannot insert SensorGps message."
                    )

            self._db_connection.commit()

    @property
    @ROS.subscribe(
        ROS_TOPIC_SENSOR_GPS,
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=_update_database,
    )
    def sensor_gps(self) -> Optional[SensorGps]:
        """Subscribed :attr:`.UORBNode.sensor_gps` mock GPS message"""
