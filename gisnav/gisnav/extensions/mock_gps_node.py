"""GISNav :term:`extension` :term:`node` that publishes mock GPS (GNSS) messages"""
import json
import socket
from datetime import datetime
from typing import Final, Optional, Tuple

import numpy as np
from geometry_msgs.msg import PoseStamped, Vector3, Vector3Stamped, TransformStamped
from gps_time import GPSTime
from px4_msgs.msg import SensorGps
import rclpy
import tf2_ros
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from rclpy.timer import Timer
from pyproj import Transformer
from ..decorators import ROS, narrow_types

from .. import messaging
from .._data import Attitude
from ..constants import (
    GIS_NODE_NAME,
    ROS_NAMESPACE,
    ROS_TOPIC_RELATIVE_SCALING,
)

_ROS_PARAM_DESCRIPTOR_READ_ONLY: Final = ParameterDescriptor(read_only=True)
"""A read only ROS parameter descriptor"""


class MockGPSNode(Node):
    """A node that publishes a mock GPS message over the microRTPS bridge

    .. mermaid::

        graph LR
            tf[tf]

            subgraph MAVROS
                navsatfix[mavros/global_position/global]
            end

            subgraph GISNode
                scaling[gisnav/gis_node/scaling]
            end

            subgraph MockGPSNode
                sensor_gps[fmu/in/sensor_gps]
                gps_input
            end

            tf -->|'geometry_msgs/TransformStamped camera->wgs84_unscaled'| MockGPSNode
            scaling -->|geometry_msgs/Vector3Stamped| MockGPSNode
            sensor_gps -->|px4_msgs.msg.SensorGps| micro-ros-agent:::hidden
            gps_input -->|GPSINPUT over UDP| MAVLink:::hidden

    """

    ROS_D_USE_SENSOR_GPS: Final = True
    """Set to ``False`` to use :class:`mavros_msgs.msg.GPSINPUT` message for
    :term:`ArduPilot`, :class:`px4_msgs.msg.SensorGps` for :term:`PX4` otherwise.
    """

    ROS_D_UDP_HOST: Final = "127.0.0.1"
    """MAVProxy GPSInput plugin default host

    .. note::
        Only used if :attr:`use_sensor_gps` is ``False``
    """

    ROS_D_UDP_PORT: Final = 25100
    """MAVProxy GPSInput plugin default port

    .. note::
        Only used if :attr:`use_sensor_gps` is ``False``
    """

    ROS_D_PUBLISH_RATE = 1.0
    """Default mock GPS message publish rate in Hz"""

    ROS_D_DEM_VERTICAL_DATUM = 5703
    """Default :term:`DEM` vertical datum"""

    ROS_TOPIC_SENSOR_GPS: Final = "/fmu/in/sensor_gps"
    """Absolute :term:`topic` into which this :term:`node` publishes
    :attr:`.sensor_gps`
    """

    def __init__(self, *args, **kwargs):
        """Class initializer

        :param args: Positional arguments to parent :class:`.Node` constructor
        :param kwargs: Keyword arguments to parent :class:`.Node` constructor
        """
        super().__init__(*args, **kwargs)

        if self.use_sensor_gps:
            self._mock_gps_pub = self.create_publisher(
                SensorGps,
                messaging.ROS_TOPIC_SENSOR_GPS,
                QoSPresetProfiles.SENSOR_DATA.value,
            )
            self._socket = None
        else:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._mock_gps_pub = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        publish_rate = self.publish_rate
        assert publish_rate is not None
        self._publish_timer: Optional[Timer] = self._create_publish_timer(
            publish_rate
        )

        # Subscribe to scaling
        self.scaling

    @property
    @ROS.parameter(ROS_D_USE_SENSOR_GPS, descriptor=_ROS_PARAM_DESCRIPTOR_READ_ONLY)
    def use_sensor_gps(self) -> Optional[bool]:
        """:term:`ROS` parameter flag indicating outgoing mock :term:`GPS` message
        should be published as :class:`px4_msgs.msg.SensorGps` for :term:`PX4`
        instead of as :class:`mavros_msgs.msg.GPSINPUT` for :term:`ArduPilot`.
        """

    @property
    @ROS.parameter(ROS_D_UDP_HOST, descriptor=_ROS_PARAM_DESCRIPTOR_READ_ONLY)
    def udp_host(self) -> Optional[str]:
        """:term:`ROS` parameter MAVProxy GPSInput plugin host name or IP address"""

    @property
    @ROS.parameter(ROS_D_UDP_PORT, descriptor=_ROS_PARAM_DESCRIPTOR_READ_ONLY)
    def udp_port(self) -> Optional[int]:
        """:term:`ROS` parameter MAVProxy GPSInput plugin port"""

    @property
    @ROS.parameter(ROS_D_PUBLISH_RATE, descriptor=_ROS_PARAM_DESCRIPTOR_READ_ONLY)
    def publish_rate(self) -> Optional[float]:
        """Mock :term:`GPS` :term:`message` publish rate in Hz"""

    @property
    @ROS.parameter(ROS_D_DEM_VERTICAL_DATUM, descriptor=_ROS_PARAM_DESCRIPTOR_READ_ONLY)
    def dem_vertical_datum(self) -> Optional[int]:
        """:term:`DEM` vertical datum (must match DEM that is published in
        :attr:`.GISNode.orthoimage`)
        """

    @narrow_types
    def _create_publish_timer(self, publish_rate: float) -> Timer:
        """Returns a timer that publishes the output mock :term:`GPS`
        :term:`message` at regular intervals

        :param publish_rate: Publish rate in Hz, see (:attr:`.publish_rate`)
        :return: The :class:`.Timer` instance
        """
        if publish_rate <= 0:
            error_msg = (
                f"Mock GPS publish rate rate must be positive ({publish_rate} "
                f"Hz provided)."
            )
            self.get_logger().error(error_msg)
            raise ValueError(error_msg)
        timer = self.create_timer(
            1 / publish_rate,
            self.publish_sensor_gps if self.use_sensor_gps else self.publish_gps_input
        )
        return timer

    @property
    @ROS.max_delay_ms(messaging.DELAY_DEFAULT_MS)
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_SCALING.replace("~", GIS_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def scaling(self) -> Optional[Vector3Stamped]:
        """Subscribed :term:`camera` to :term:`WGS 84 unscaled <WGS 84>` frame
        scaling vector, or None if not available

        .. seealso::
            :attr:`.GISNode.scaling`
        """

    @ROS.publish(
        ROS_TOPIC_SENSOR_GPS,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def publish_sensor_gps(self) -> Optional[SensorGps]:
        """Outgoing mock :term:`GNSS` :term:`message` when :attr:`use_sensor_gps`
        is ``True``

        Uses the release/1.14 tag version of :class:`px4_msgs.msg.SensorGps`
        """

        def _dot_product(vector1: Vector3, vector2: Vector3) -> Vector3:
            return Vector3(
                x=vector1.x * vector2.x,
                y=vector1.y * vector2.y,
                z=vector1.z * vector2.z
            )

        @narrow_types(self)
        def _sensor_gps(
            camera_to_wgs84_unscaled: TransformStamped,
            scaling: Vector3Stamped,
            device_id: int,
        ) -> SensorGps:

            # TODO
            camera_to_wgs84_unscaled.transform.translation = _dot_product(
                camera_to_wgs84_unscaled.transform.translation, scaling.vector)

            translation, rotation = (camera_to_wgs84_unscaled.transform.translation,
                                     camera_to_wgs84_unscaled.transform.rotation)

            # TODO: check yaw sign (NED or ENU?)
            # TODO: get vehicle yaw (heading) not camera yaw
            camera_yaw_degrees = messaging.extract_yaw(
                rotation
            )
            camera_yaw_degrees = int(camera_yaw_degrees % 360)
            # MAVLink definition 0 := not available
            camera_yaw_degrees = 360 if camera_yaw_degrees == 0 else camera_yaw_degrees
            yaw_rad = np.radians(camera_yaw_degrees)

            satellites_visible = np.iinfo(np.uint8).max
            timestamp = messaging.usec_from_header(camera_to_wgs84_unscaled.header)

            eph = 10.0
            epv = 1.0

            msg = SensorGps()
            msg.timestamp = int(timestamp)
            msg.timestamp_sample = int(timestamp)
            msg.device_id = device_id
            # msg.device_id = 0
            msg.fix_type = 3
            msg.s_variance_m_s = 5.0  # not estimated, use default cruise speed
            msg.c_variance_rad = np.nan

            msg.lat = int(translation.y * 1e7)
            msg.lon = int(translation.x * 1e7)

            altitudes = self._convert_to_wgs84(translation.y, translation.x, translation.z, self.dem_vertical_datum)
            if altitudes is not None:
                alt_ellipsoid, alt_amsl = altitudes
            else:
                # todo Fix this - return type should be SensorGps
                return None

            msg.alt_ellipsoid = int(alt_ellipsoid * 1e3)
            msg.alt = int(alt_amsl * 1e3)

            msg.eph = eph
            msg.epv = epv
            msg.hdop = 0.0
            msg.vdop = 0.0
            msg.noise_per_ms = 0
            msg.automatic_gain_control = 0
            msg.jamming_state = 0
            msg.jamming_indicator = 0
            msg.vel_m_s = 0.0
            msg.vel_n_m_s = 0.0
            msg.vel_e_m_s = 0.0
            msg.vel_d_m_s = 0.0
            msg.cog_rad = np.nan
            msg.vel_ned_valid = True
            msg.timestamp_time_relative = 0
            msg.satellites_used = satellites_visible
            msg.time_utc_usec = msg.timestamp
            msg.heading = float(yaw_rad)
            msg.heading_offset = 0.0
            msg.heading_accuracy = 0.0

            return msg

        # TODO: fix interpolation - reference/wgs84 frames are not continuous,
        #  i.e. they jump around which means tf interpolation should not be
        #  applied. Must specify specific timestamps to connect camera pose
        #  estimation chain to correct reference frame.
        camera_to_wgs84_unscaled = messaging.get_transform(
            self, "wgs_84_unscaled", "camera",
            rclpy.time.Time()
        )

        return _sensor_gps(
            camera_to_wgs84_unscaled,
            self.scaling,
            self._device_id,
        )

    @property
    # @ROS.publish(
    #    ROS_TOPIC_GPS_INPUT,
    #    QoSPresetProfiles.SENSOR_DATA.value,
    # )
    def publish_gps_input(self) -> Optional[dict]:  # Optional[GPSINPUT]
        """Outgoing mock :term:`GNSS` :term:`message` when :attr:`use_sensor_gps`
        is ``False``

        .. note::
            Does not use :class:`mavros_msgs.msg.GPSINPUT` message over MAVROS,
            sends a :term:`MAVLink` GPS_INPUT message directly to :term:`ArduPilot`.
        """

        @narrow_types
        def _gps_input(
            vehicle_estimated_pose: PoseStamped,
        ) -> Optional[dict]:
            # TODO: check yaw sign (NED or ENU?)
            q = messaging.as_np_quaternion(vehicle_estimated_pose.orientation)
            yaw = Attitude(q=q).yaw
            yaw = int(np.degrees(yaw % (2 * np.pi)))
            yaw = 360 if yaw == 0 else yaw  # MAVLink definition 0 := not available

            satellites_visible = np.iinfo(np.uint8).max
            timestamp = messaging.usec_from_header(vehicle_estimated_pose.header)

            eph = 10.0
            epv = 1.0

            gps_time = GPSTime.from_datetime(datetime.utcfromtimestamp(timestamp / 1e6))

            # TODO: lat, lon, alt
            msg = dict(
                usec=timestamp,
                gps_id=0,
                ignore_flags=0,
                time_week=gps_time.week_number,
                time_week_ms=int(gps_time.time_of_week * 1e3),
                fix_type=3,
                # lat=int(vehicle_estimated_geopose.pose.position.latitude * 1e7),
                # lon=int(vehicle_estimated_geopose.pose.position.longitude * 1e7),
                # alt=vehicle_estimated_altitude.amsl,
                horiz_accuracy=eph,
                vert_accuracy=epv,
                speed_accuracy=5.0,
                hdop=0.0,
                vdop=0.0,
                vn=0.0,
                ve=0.0,
                vd=0.0,
                satellites_visible=satellites_visible,
                yaw=yaw * 100,
            )

            # TODO: handle None host or port
            self._socket.sendto(
                f"{json.dumps(msg)}".encode("utf-8"), (self.udp_host, self.udp_port)
            )

            return msg

        # TODO
        return None
        # return _gps_input(
        #    self.vehicle_estimated_geopose, self.vehicle_estimated_altitude
        # )

    @property
    def _device_id(self) -> int:
        """Generates a device ID for the outgoing `px4_msgs.SensorGps` message"""
        # For reference, see:
        # https://docs.px4.io/main/en/middleware/drivers.html and
        # https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/drv_sensor.h
        # https://docs.px4.io/main/en/gps_compass/

        # DRV_GPS_DEVTYPE_SIM (0xAF) + dev 1 + bus 1 + DeviceBusType_UNKNOWN
        # = 10101111 00000001 00001 000
        # = 11469064
        return 11469064

    @narrow_types
    def _convert_to_wgs84(self, lat: float, lon: float, elevation: float, epsg_code: int) -> Optional[Tuple[float, float]]:
        """
        Convert :term:`elevation` or :term:`altitude` from a specified vertical
        datum to :term:`WGS 84`.

        :param lat: Latitude in decimal degrees.
        :param lon: Longitude in decimal degrees.
        :param elevation: Elevation in the specified datum.
        :param epsg_code: EPSG code of the vertical datum.
        :return: A tuple containing :term:`elevation` above :term:`WGS 84` and
            :term:`AMSL`.
        """
        # EPSG code for WGS 84 and a common mean sea level datum (e.g., EGM96)
        epsg_wgs84 = 4326
        epsg_msl = 5773  # Example: EGM96

        # Create transformers
        transformer_to_wgs84 = Transformer.from_crs(
            f"EPSG:{epsg_code}",
            f"EPSG:{epsg_wgs84}",
            always_xy=True
        )
        transformer_to_msl = Transformer.from_crs(
            f"EPSG:{epsg_code}",
            f"EPSG:{epsg_msl}",
            always_xy=True
        )

        # Perform the transformations
        _, _, wgs84_elevation = transformer_to_wgs84.transform(lon, lat, elevation)
        _, _, msl_elevation = transformer_to_msl.transform(lon, lat, elevation)

        return wgs84_elevation, msl_elevation
