"""This module contains the :class:`.MockGPSNode` :term:`extension` :term:`node`
that publishes mock GPS (GNSS) messages to autopilot middleware
"""
import json
import socket
from datetime import datetime
from typing import Final, Optional, Tuple

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import PoseStamped
from gps_time import GPSTime
from px4_msgs.msg import SensorGps
from pyproj import Transformer
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from .. import _transformations as tf_
from .._decorators import ROS, narrow_types
from ..constants import (
    POSE_NODE_NAME,
    ROS_NAMESPACE,
    ROS_TOPIC_RELATIVE_CAMERA_ESTIMATED_POSE,
    ROS_TOPIC_SENSOR_GPS,
    FrameID,
)

_ROS_PARAM_DESCRIPTOR_READ_ONLY: Final = ParameterDescriptor(read_only=True)
"""A read only ROS parameter descriptor"""


class MockGPSNode(Node):
    """A node that publishes a mock GPS message to autopilot middleware"""

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

    # EPSG code for WGS 84 and a common mean sea level datum (e.g., EGM96)
    _EPSG_WGS84 = 4326
    _EPSG_MSL = 5773  # Example: EGM96

    def __init__(self, *args, **kwargs):
        """Class initializer

        :param args: Positional arguments to parent :class:`.Node` constructor
        :param kwargs: Keyword arguments to parent :class:`.Node` constructor
        """
        super().__init__(*args, **kwargs)

        if self.use_sensor_gps:
            self._mock_gps_pub = self.create_publisher(
                SensorGps,
                ROS_TOPIC_SENSOR_GPS,
                QoSPresetProfiles.SENSOR_DATA.value,
            )
            self._socket = None
        else:
            self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self._mock_gps_pub = None

        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # Create transformers
        self._transformer_to_wgs84 = Transformer.from_crs(
            f"EPSG:{self.dem_vertical_datum}",
            f"EPSG:{self._EPSG_WGS84}",
            always_xy=True,
        )
        self._transformer_to_msl = Transformer.from_crs(
            f"EPSG:{self.dem_vertical_datum}", f"EPSG:{self._EPSG_MSL}", always_xy=True
        )

        # Subscribe
        self.pose

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
    @ROS.parameter(ROS_D_DEM_VERTICAL_DATUM, descriptor=_ROS_PARAM_DESCRIPTOR_READ_ONLY)
    def dem_vertical_datum(self) -> Optional[int]:
        """:term:`DEM` vertical datum (must match DEM that is published in
        :attr:`.GISNode.orthoimage`)
        """

    def _pose_cb(self, msg: PoseStamped) -> None:
        frame_id: FrameID = msg.header.frame_id

        if frame_id.startswith("+proj"):
            # This pose as a proj string so we know it's an earth-fixed frame and we
            # can create a mock GPS message out of it. The query frames used in VO
            # are not suitable since they do not contain a georeference like a proj
            # string.

            # we do not use msg.header.stamp because we want to interpolation
            # up to the reference frame. The reference frame itself is discontinous,
            # but the rotated and cropped frame leading to it is not, so interpolation
            # should work.
            stamp = rclpy.time.Time()

            # For the mock GPS message we are interested in base_link, not
            # camera_optical pose. So we get that transform instead.
            transform_stamped = tf_.get_transform(
                self,
                "base_link",
                frame_id,
                stamp,
            )

            # TODO: transform_stamped should not be None, figure out why it sometimes is

            if transform_stamped is not None:
                pose_stamped = tf_.transform_to_pose(transform_stamped)
                pose_stamped.header.frame_id = frame_id  # TODO: fix this
                self._publish(pose_stamped)

    @property
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_CAMERA_ESTIMATED_POSE.replace("~", POSE_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
        callback=_pose_cb,
    )
    def pose(self) -> Optional[PoseStamped]:
        """Camera estimated pose"""

    def _publish(self, pose_stamped: PoseStamped) -> None:
        @narrow_types(self)
        def _publish_inner(pose_stamped: PoseStamped) -> None:
            # Convert to WGS 84 coordinates (altitude in meters AGL)
            frame_id: FrameID = pose_stamped.header.frame_id

            M = tf_.proj_to_affine(frame_id)
            H, r, t = tf_.pose_stamped_to_matrices(pose_stamped)

            # M has translations in the 4th column so we add 1 to the translation vector
            assert t.shape == (3,)
            t = M @ np.append(t, 1)

            self.get_logger().info(f"position {str(t)}")

            timestamp = tf_.usec_from_header(pose_stamped.header)

            vehicle_yaw_degrees = tf_.extract_yaw(pose_stamped.pose.orientation)
            vehicle_yaw_degrees = int(vehicle_yaw_degrees % 360)
            # MAVLink yaw definition 0 := not available
            vehicle_yaw_degrees = (
                360 if vehicle_yaw_degrees == 0 else vehicle_yaw_degrees
            )

            lat = int(t[1] * 1e7)
            lon = int(t[0] * 1e7)

            altitudes = self._convert_to_wgs84(
                t[1],
                t[0],
                t[2],
            )
            if altitudes is not None:
                alt_ellipsoid, alt_amsl = altitudes
            else:
                return None

            satellites_visible = np.iinfo(np.uint8).max
            eph = 10.0
            epv = 1.0

            if self.use_sensor_gps:
                self.sensor_gps(
                    lat,
                    lon,
                    alt_ellipsoid,
                    alt_amsl,
                    vehicle_yaw_degrees,
                    self._device_id,
                    timestamp,
                    eph,
                    epv,
                    satellites_visible,
                )
            else:
                self.gps_input(
                    lat,
                    lon,
                    alt_amsl,
                    vehicle_yaw_degrees,
                    timestamp,
                    eph,
                    epv,
                    satellites_visible,
                )

        _publish_inner(pose_stamped)

    @narrow_types
    @ROS.publish(
        ROS_TOPIC_SENSOR_GPS,
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def sensor_gps(
        self,
        lat: int,
        lon: int,
        altitude_ellipsoid: float,
        altitude_amsl: float,
        yaw_degrees: int,
        device_id: int,
        timestamp: int,
        eph: float,
        epv: float,
        satellites_visible: int,
    ) -> Optional[SensorGps]:
        """Outgoing mock :term:`GNSS` :term:`message` when :attr:`use_sensor_gps`
        is ``True``

        Uses the release/1.14 tag version of :class:`px4_msgs.msg.SensorGps`
        """

        yaw_rad = np.radians(yaw_degrees)

        msg = SensorGps()
        msg.timestamp = int(timestamp)
        msg.timestamp_sample = int(timestamp)
        msg.device_id = device_id
        # msg.device_id = 0
        msg.fix_type = 3
        msg.s_variance_m_s = 5.0  # not estimated, use default cruise speed
        msg.c_variance_rad = np.nan
        msg.lat = lat
        msg.lon = lon
        msg.alt_ellipsoid = int(altitude_ellipsoid * 1e3)
        msg.alt = int(altitude_amsl * 1e3)
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

    @narrow_types
    # @ROS.publish(
    #    ROS_TOPIC_GPS_INPUT,
    #    QoSPresetProfiles.SENSOR_DATA.value,
    # )
    def gps_input(
        self,
        lat: int,
        lon: int,
        altitude_amsl: float,
        yaw_degrees: int,
        timestamp: int,
        eph: float,
        epv: float,
        satellites_visible: int,
    ) -> Optional[dict]:  # Optional[GPSINPUT]
        """Outgoing mock :term:`GNSS` :term:`message` when :attr:`use_sensor_gps`
        is ``False``

        .. note::
            Does not use :class:`mavros_msgs.msg.GPSINPUT` message over MAVROS,
            sends a :term:`MAVLink` GPS_INPUT message directly to :term:`ArduPilot`.
        """
        gps_time = GPSTime.from_datetime(datetime.utcfromtimestamp(timestamp / 1e6))

        msg = dict(
            usec=timestamp,
            gps_id=0,
            ignore_flags=0,
            time_week=gps_time.week_number,
            time_week_ms=int(gps_time.time_of_week * 1e3),
            fix_type=3,
            lat=lat,
            lon=lon,
            alt=altitude_amsl,
            horiz_accuracy=eph,
            vert_accuracy=epv,
            speed_accuracy=5.0,
            hdop=0.0,
            vdop=0.0,
            vn=0.0,
            ve=0.0,
            vd=0.0,
            satellites_visible=satellites_visible,
            yaw=yaw_degrees * 100,
        )

        # TODO: handle None host or port
        self._socket.sendto(
            f"{json.dumps(msg)}".encode("utf-8"), (self.udp_host, self.udp_port)
        )

        return msg
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
    def _convert_to_wgs84(
        self, lat: float, lon: float, elevation: float
    ) -> Optional[Tuple[float, float]]:
        """
        Convert :term:`elevation` or :term:`altitude` from a specified vertical
        datum to :term:`WGS 84`.

        :param lat: Latitude in decimal degrees.
        :param lon: Longitude in decimal degrees.
        :param elevation: Elevation in the specified datum.
        :return: A tuple containing :term:`elevation` above :term:`WGS 84` and
            :term:`AMSL`.
        """
        _, _, wgs84_elevation = self._transformer_to_wgs84.transform(
            lon, lat, elevation
        )
        _, _, msl_elevation = self._transformer_to_msl.transform(lon, lat, elevation)

        return wgs84_elevation, msl_elevation
