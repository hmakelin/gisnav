from typing import Optional, Tuple

import numpy as np
from geographic_msgs.msg import BoundingBox, GeoPoseStamped
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import Altitude
from pygeodesy.geoids import GeoidPGM
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

from gisnav import messaging
from gisnav._decorators import ROS
from gisnav.static_configuration import (
    GIS_NODE_NAME,
    ROS_NAMESPACE,
    ROS_TOPIC_RELATIVE_CAMERA_QUATERNION,
    ROS_TOPIC_RELATIVE_GROUND_TRACK_ELEVATION,
    ROS_TOPIC_RELATIVE_GROUND_TRACK_GEOPOSE,
    ROS_TOPIC_RELATIVE_ORTHOIMAGE,
    ROS_TOPIC_RELATIVE_VEHICLE_ALTITUDE,
    ROS_TOPIC_RELATIVE_VEHICLE_GEOPOSE,
)
from gisnav_msgs.msg import OrthoImage3D  # type: ignore


class StateListenerNode(Node):
    """
    A ROS node that subscribes to GISNav's output messages

    :param name: Node name
    """

    def __init__(self, name):
        super().__init__(name)
        # TODO: do not hard code path
        self._egm96 = GeoidPGM("/usr/share/GeographicLib/geoids/egm96-5.pgm", kind=-3)

        # Subscribe to topics
        self.vehicle_geopose
        self.vehicle_altitude
        self.orthoimage
        self.ground_track_geopose
        self.ground_track_elevation
        self.camera_quaternion

    @property
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_ORTHOIMAGE.replace("~", GIS_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def orthoimage(self) -> Optional[OrthoImage3D]:
        """Subscribed :term:`orthoimage` for :term:`pose` estimation"""

    @property
    @ROS.max_delay_ms(messaging.DELAY_DEFAULT_MS)
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_GROUND_TRACK_ELEVATION.replace("~", GIS_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def ground_track_elevation(self) -> Optional[Altitude]:
        """:term:`Ground track` :term:`elevation`, or None if not available or
        too old
        """

    @property
    @ROS.max_delay_ms(messaging.DELAY_DEFAULT_MS)
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_GROUND_TRACK_GEOPOSE.replace("~", GIS_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def ground_track_geopose(self) -> Optional[GeoPoseStamped]:
        """:term:`Ground track` :term:`geopose`, or None if not available or too old

        Complementary to the :attr:`ground_track_elevation`, includes lat and lon
        in atomic message
        """

    @property
    @ROS.max_delay_ms(messaging.DELAY_DEFAULT_MS)
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_VEHICLE_ALTITUDE.replace("~", GIS_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def vehicle_altitude(self) -> Optional[Altitude]:
        """Altitude of vehicle, or None if unknown or too old"""

    @property
    # @ROS.max_delay_ms(messaging.DELAY_DEFAULT_MS)
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_CAMERA_QUATERNION.replace("~", GIS_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def camera_quaternion(self) -> Optional[Quaternion]:
        """:term:`Camera` :term:`orientation` as :class:`geometry_msgs.msg.Quaternion`
        message, or None if not available

        Quaternion origin is defined as facing down :term:`nadir` in :term:`NED`
        frame, with top side of image facing north.
        """

    @property
    # @ROS.max_delay_ms(messaging.DELAY_DEFAULT_MS)  # TODO: re-enable
    @ROS.subscribe(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_VEHICLE_GEOPOSE.replace("~", GIS_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def vehicle_geopose(self) -> Optional[GeoPoseStamped]:
        """Vehicle GeoPoseStamped, or None if not available or too old"""

    @property
    def _egm96_height(self) -> Optional[float]:
        """:term:`Vehicle` :term:`ground track` :term:`EGM96` ellipsoid
        :term:`elevation`
        """
        if self.ground_track_geopose is not None:
            # possibly slightly different from vehicle global position
            return self._egm96.height(
                self.ground_track_geopose.pose.position.latitude,
                self.ground_track_geopose.pose.position.longitude,
            )

        if self.vehicle_geopose is not None:
            return self._egm96.height(
                self.vehicle_geopose.pose.position.latitude,
                self.vehicle_geopose.pose.position.longitude,
            )

        return None

    @staticmethod
    def _assert_latitude(
        geopose_msg: GeoPoseStamped, lat: float, tolerance_meters: float = 1.0
    ) -> None:
        """Asserts given :term:`vehicle` or :term:`ground track` :term:`WGS 84`
        latitude in degrees with a given tolerance in meters.

        Compares input latitude against appropriate :term:`geopose` message

        :param geopose_msg: Vehicle or ground track geopose
        :param lat: Vehicle or ground track WGS 84 latitude in degrees
        :param tolerance_meters: Tolerance in meters for latitude comparison
        :raise: AssertionError if latitude difference exceeds the given
            tolerance in meters
        """
        meters_per_degree = 111000  # Approximate value
        difference_meters = (
            abs(geopose_msg.pose.position.latitude - lat) * meters_per_degree
        )

        assert difference_meters <= tolerance_meters, (
            f"Expected latitude: {lat}, but got: {geopose_msg.pose.position.latitude}. "
            f"Difference: {difference_meters} meters."
        )

    @staticmethod
    def _assert_longitude(
        geopose_msg: GeoPoseStamped, lon: float, tolerance_meters: float = 1.0
    ) -> None:
        """Asserts given :term:`vehicle` or :term:`ground track` :term:`WGS 84`
        longitude in degrees with a given tolerance in meters.

        Compares input longitude against appropriate :term:`geopose` message

        :param geopose_msg: Vehicle or ground track geopose
        :param lon: Vehicle or ground track WGS 84 longitude in degrees
        :param tolerance_meters: Tolerance in meters for longitude comparison
        :raise: AssertionError if longitude difference exceeds the given
            tolerance in meters
        """
        # TODO: do not use latitude from the message being tested, assert
        #  latitude together with longitude
        # approximation for the length of a degree of longitude as
        # a function of latitude
        latitude_rad = np.radians(geopose_msg.pose.position.latitude)
        meters_per_degree_longitude = (
            111412.84 * np.cos(latitude_rad)
            - 93.5 * np.cos(3 * latitude_rad)
            + 0.118 * np.cos(5 * latitude_rad)
        )

        difference_meters = (
            abs(geopose_msg.pose.position.longitude - lon) * meters_per_degree_longitude
        )

        assert difference_meters <= tolerance_meters, (
            f"Expected longitude: {lon}, "
            f"but got: {geopose_msg.pose.position.longitude}. "
            f"Difference: {difference_meters} meters."
        )

    def _assert_amsl_altitude(self, geopose_msg, altitude_msg, alt_amsl_meters: float):
        """Asserts given :term:`vehicle` or :term:`ground track` :term:`AMSL`
        :term:`altitude` or :term:`elevation` in meters

        Compares input altitude or elevation against appropriate :term:`geopose`
        and altitude :term:`ROS` messages.

        :param geopose_msg: Vehicle or ground track geopose
        :param altitude_msg: Vehicle (or ground track) altitude (elevation) message
        :param alt_amsl_meters: Vehicle (or ground track) AMSL altitude
            (elevation) in meters
        :raise: AssertionError if altitude does not match received ROS messages
        """
        if geopose_msg is not None:
            egm96_height = self._egm96_height
            if self._egm96_height is not None:
                vehicle_alt_amsl_from_geopose = (
                    geopose_msg.pose.position.altitude - egm96_height
                )
                assert vehicle_alt_amsl_from_geopose == alt_amsl_meters, (
                    f"Expected AMSL altitude: {alt_amsl_meters}, "
                    f"but got: {vehicle_alt_amsl_from_geopose}"
                )
        elif altitude_msg is not None:
            assert (
                altitude_msg.amsl == alt_amsl_meters
            ), f"Expected altitude: {alt_amsl_meters}, but got: {altitude_msg.amsl}"
        else:
            assert False, (
                f"Expected AMSL altitude: {alt_amsl_meters}, "
                f"but no altitude message received."
            )

    def _assert_ellipsoid_altitude(
        self, geopose_msg, altitude_msg, alt_ellipsoid_meters: float
    ):
        """Asserts given :term:`vehicle` or :term:`ground track` :term:`ellipsoid`
        :term:`altitude` or :term:`elevation` in meters

        Compares input altitude or elevation against appropriate :term:`geopose`
        and altitude :term:`ROS` messages.

        :param geopose_msg: Vehicle or ground track geopose
        :param altitude_msg: Vehicle (or ground track) altitude (elevation) message
        :param vehicle_alt_ellipsoid_meters: Vehicle (or ground track) ellipsoid
            altitude (elevation) in meters
        :raise: AssertionError if altitude does not match received ROS messages
        """
        if geopose_msg is not None:
            assert geopose_msg.pose.position.altitude == alt_ellipsoid_meters, (
                f"Expected altitude: {alt_ellipsoid_meters}, "
                f"but got: {geopose_msg.pose.position.altitude}"
            )
        elif altitude_msg is not None:
            egm96_height = self._egm96_height
            if self._egm96_height is not None:
                vehicle_alt_ellipsoid_from_altitude = altitude_msg.amsl + egm96_height
                assert alt_ellipsoid_meters == vehicle_alt_ellipsoid_from_altitude, (
                    f"Expected ellipsoid altitude: {alt_ellipsoid_meters}, "
                    f"but got: {vehicle_alt_ellipsoid_from_altitude}"
                )
        else:
            assert False, (
                f"Expected ellipsoid altitude: {alt_ellipsoid_meters}, "
                f"but no altitude message received."
            )

    def _assert_pitch(self, quaternion: Quaternion, pitch_degrees: float) -> None:
        """Asserts given :term:`camera` pitch in degrees

        Origin is defined as facing :term:`nadir`, with image top side facing
        north.

        :param quaternion: Camera quaternion
        :param pitch_degrees: Camera pitch in degrees in NED frame with origin
            defined as facing down nadir
        :raise: AssertionError if pitch does not match received :term:`ROS`
            camera quaternion
        """
        if quaternion is not None:
            _, pitch, __ = self._get_yaw_pitch_roll_degrees_from_quaternion(
                self.camera_quaternion
            )
            assert (
                pitch == pitch_degrees
            ), f"Expected camera pitch: {pitch_degrees}, but got: {pitch}."
        else:
            assert (
                False
            ), f"Expected pitch of {pitch_degrees}, but no ROS message received."

    def _assert_roll(self, quaternion: Quaternion, roll_degrees: float) -> None:
        """Asserts given :term:`camera` roll in degrees

        Origin is defined as facing :term:`nadir`, with image top side facing
        north.

        :param quaternion: Camera quaternion
        :param roll_degrees: Camera roll in degrees in NED frame with origin
            defined as facing down nadir
        :raise: AssertionError if roll does not match received :term:`ROS`
            camera quaternion
        """
        if quaternion is not None:
            _, __, roll = self._get_yaw_pitch_roll_degrees_from_quaternion(
                self.camera_quaternion
            )
            assert (
                roll == roll_degrees
            ), f"Expected camera roll: {roll_degrees}, but got: {roll}."
        else:
            assert (
                False
            ), f"Expected roll of {roll_degrees}, but no ROS message received."

    def _assert_yaw(self, quaternion: Quaternion, yaw_degrees: float) -> None:
        """Asserts given :term:`camera` yaw in degrees

        Origin is defined as facing :term:`nadir`, with image top side facing
        north.

        :param quaternion: Camera quaternion
        :param yaw_degrees: Camera yaw in degrees in NED frame with origin
            defined as facing down nadir
        :raise: AssertionError if yaw does not match received :term:`ROS`
            camera quaternion
        """
        if quaternion is not None:
            yaw, _, __ = self._get_yaw_pitch_roll_degrees_from_quaternion(
                self.camera_quaternion
            )
            assert (
                yaw == yaw_degrees
            ), f"Expected camera yaw: {yaw_degrees}, but got: {yaw}."
        else:
            assert False, f"Expected yaw of {yaw_degrees}, but no ROS message received."

    def assert_state(
        self,
        vehicle_lat: Optional[float] = None,
        vehicle_lon: Optional[float] = None,
        vehicle_alt_amsl_meters: Optional[float] = None,
        vehicle_alt_ellipsoid_meters: Optional[float] = None,
        ground_track_lat: Optional[float] = None,
        ground_track_lon: Optional[float] = None,
        ground_track_elevation_amsl_meters: Optional[float] = None,
        ground_track_elevation_ellipsoid_meters: Optional[float] = None,
        vehicle_heading_ned: Optional[float] = None,
        camera_pitch_ned_deg: Optional[float] = None,
        camera_yaw_ned_deg: Optional[float] = None,
        camera_roll_ned_deg: Optional[float] = None,
        bbox: Optional[BoundingBox] = None,
    ) -> None:
        """
        Asserts latest state

        In an example use case you would only need to assert a specific subset
        of the input arguments (a partial state) that you are testing, all other
        messages being None by default (=no message received).

        :param vehicle_lat: :term`Vehicle` :term:`WGS 84` latitude coordinate
            in degrees
        :param vehicle_lon: :term`Vehicle` :term:`WGS 84` longitude coordinate
            in degrees
        :param vehicle_alt_amsl_meters: :term`Vehicle` :term:`AMSL` :term:`altitude`
            in meters
        :param vehicle_alt_ellipsoid_meters: :term`Vehicle` :term:`ellipsoid`
            :term:`altitude` in meters
        :param ground_track_lat: :term:`Ground track` :term:`WGS 84` latitude
            coordinate in degrees
        :param ground_track_lon: :term:`Ground track` :term:`WGS 84` longitude
            coordinate in degrees
        :param ground_track_elevation_amsl_meters: :term:`Ground track` :term:`AMSL`
            :term:`elevation` in meters
        :param ground_track_elevation_ellipsoid_meters: :term:`Ground track`
            :term:`ellipsoid` :term:`elevation` in meters
        :param vehicle_heading_ned: Vehicle heading in :term:`NED` frame in degrees
        :param camera_pitch_ned_deg: :term:`Camera` pitch angle in :term:`NED` frame
            in degrees. Origin is defined as facing :term:`nadir`, with image
            top side facing north.
        :param camera_yaw_ned_deg: :term:`Camera` yaw angle in :term:`NED` frame
            in degrees. Origin is defined as facing :term:`nadir`, with image
            top side facing north.
        :param camera_roll_ned_deg: :term:`Camera` roll angle in :term:`NED` frame
            in degrees. Origin is defined as facing :term:`nadir`, with image
            top side facing north.
        :param bbox: :term:`Bounding box` for the :term:`orthoimage`, optional.
        """
        # Assert the vehicle GeoPose and Altitude
        if vehicle_lat is not None:
            self._assert_latitude(self.vehicle_geopose, vehicle_lat)

        if vehicle_lon is not None:
            self._assert_longitude(self.vehicle_geopose, vehicle_lon)

        if vehicle_alt_amsl_meters is not None:
            self._assert_amsl_altitude(
                self.vehicle_geopose, self.vehicle_altitude, vehicle_alt_amsl_meters
            )

        if vehicle_alt_ellipsoid_meters is not None:
            self._assert_ellipsoid_altitude(
                self.vehicle_geopose,
                self.vehicle_altitude,
                vehicle_alt_ellipsoid_meters,
            )

        # Assert the ground track GeoPose and Altitude
        if ground_track_lat is not None:
            self._assert_latitude(self.ground_track_geopose, ground_track_lat)

        if ground_track_lon is not None:
            self._assert_longitude(self.ground_track_geopose, ground_track_lon)

        if ground_track_elevation_amsl_meters is not None:
            self._assert_amsl_altitude(
                self.ground_track_geopose,
                self.ground_track_elevation,
                ground_track_elevation_amsl_meters,
            )

        if ground_track_elevation_ellipsoid_meters is not None:
            self._assert_ellipsoid_altitude(
                self.ground_track_geopose,
                self.ground_track_elevation,
                ground_track_elevation_ellipsoid_meters,
            )

        if camera_pitch_ned_deg is not None:
            self._assert_pitch(self.camera_quaternion, camera_pitch_ned_deg)

        if camera_yaw_ned_deg is not None:
            self._assert_yaw(self.camera_quaternion, camera_yaw_ned_deg)

        if camera_roll_ned_deg is not None:
            self._assert_roll(self.camera_quaternion, camera_roll_ned_deg)

        if bbox is not None:
            raise NotImplementedError

        if vehicle_heading_ned is not None:
            raise NotImplementedError

    @staticmethod
    def _get_yaw_pitch_roll_degrees_from_quaternion(
        quaternion: Quaternion,
    ) -> Tuple[float, float, float]:
        """
        Calculate and return yaw, pitch, and roll in degrees from a given quaternion.

        .. todo::
            This method overlaps with
            :class:`.CVNode._get_yaw_pitch_degrees_from_quaternion`

        :param quaternion: A quaternion :term:`ROS` message with origin defined
            as facing :term:`nadir`, with image top side facing north.
        :return: A tuple of yaw, pitch and roll in degrees
        """
        # Unpack quaternion
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        # Calculate yaw, pitch, and roll directly from the quaternion
        yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        pitch = np.arcsin(2.0 * (w * y - z * x))
        roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))

        # Convert yaw, pitch, and roll from radians to degrees
        yaw_degrees = yaw * 180.0 / np.pi
        pitch_degrees = pitch * 180.0 / np.pi
        roll_degrees = roll * 180.0 / np.pi

        return yaw_degrees, pitch_degrees, roll_degrees
