from typing import Optional

import rclpy
from geographic_msgs.msg import BoundingBox, GeoPoseStamped
from geometry_msgs.msg import Quaternion
from mavros_msgs.msg import Altitude, GimbalDeviceAttitudeStatus, HomePosition
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image, NavSatFix

from gisnav import messaging
from gisnav._decorators import ROS
from gisnav.static_configuration import (
    CV_NODE_NAME,
    GIS_NODE_NAME,
    ROS_NAMESPACE,
    ROS_TOPIC_RELATIVE_CAMERA_QUATERNION,
    ROS_TOPIC_RELATIVE_GROUND_TRACK_ELEVATION,
    ROS_TOPIC_RELATIVE_GROUND_TRACK_GEOPOSE,
    ROS_TOPIC_RELATIVE_ORTHOIMAGE,
    ROS_TOPIC_RELATIVE_VEHICLE_ALTITUDE,
    ROS_TOPIC_RELATIVE_VEHICLE_ESTIMATED_ALTITUDE,
    ROS_TOPIC_RELATIVE_VEHICLE_ESTIMATED_GEOPOSE,
    ROS_TOPIC_RELATIVE_VEHICLE_GEOPOSE,
)
from gisnav_msgs.msg import OrthoImage3D  # type: ignore


class StateListenerNode(Node):
    """
    A ROS node that subscribes to GISNav's output messages

    :param Node: Inherits from rclpy's Node class
    """

    def __init__(self):
        super().__init__("state_listener")
        # TODO: do not hard code path
        self._egm96 = GeoidPGM("/usr/share/GeographicLib/geoids/egm96-5.pgm", kind=-3)

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

    def assert_state(
        self,
        vehicle_lat: Optional[float] = None,
        vehicle_lon: Optional[float] = None,
        vehicle_alt_amsl_meters: Optional[float] = None,
        ground_track_lat: Optional[float] = None,
        ground_track_lon: Optional[float] = None,
        ground_track_alt_amsl_meters: Optional[float] = None,
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
        :param vehicle_alt_ellipsoid_meters: :term`Vehicle` :term:`AMSL` :term:`altitude`
            in meters
        :param ground_track_lat: :term:`Ground track` :term:`WGS 84` latitude
            coordinate in degrees
        :param ground_track_lon: :term:`Ground track` :term:`WGS 84` longitude
            coordinate in degrees
        :param ground_track_alt_amsl_meters: :term:`Ground track` :term:`AMSL`
            :term:`elevation` in meters
        :param ground_track_alt_ellipsoid_meters: :term:`Ground track` :term:`AMSL`
            :term:`elevation` in meters
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
        # TODO: terrain elevation, terrain geopose, camera quaternion (not in mermaid graph)
        """
        # Assert the vehicle GeoPose and Altitude
        if vehicle_lat is not None:
            assert (
                self.vehicle_geopose.pose.position.latitude == vehicle_lat
            ), f"Expected latitude: {vehicle_lat}, but got: {self.vehicle_geopose.latitude}"

        if vehicle_lon is not None:
            assert (
                self.vehicle_geopose.pose.position.longitude == vehicle_lon
            ), f"Expected longitude: {vehicle_lon}, but got: {self.vehicle_geopose.longitude}"

        if vehicle_alt_amsl_meters is not None:
            if self.vehicle_geopose is not None:
                egm96_height = self._egm96_height
                if self._egm96_height is not None:
                    vehicle_alt_amsl_from_geopose = (
                        self.vehicle_geopose.pose.position.altitude - egm96_height
                    )
                    assert (
                        vehicle_alt_amsl_from_geopose == vehicle_alt_amsl_meters
                    ), f"Expected AMSL altitude: {vehicle_alt_amsl_meters}, but got: {vehicle_alt_amsl_from_geopose}"
            elif self.vehicle_altitude is not None:
                assert (
                    self.vehicle_altitude.amsl == vehicle_alt_amsl_meters
                ), f"Expected altitude: {vehicle_alt_amsl_meters}, but got: {self.vehicle_altitude.amsl}"
            else:
                assert (
                    False
                ), f"Expected AMSL altitude: {vehicle_alt_amsl_meters}, but no altitude message received."

        if vehicle_alt_ellipsoid_meters is not None:
            if self.vehicle_geopose is not None:
                assert (
                    self.vehicle_geopose.pose.position.altitude
                    == vehicle_alt_ellipsoid_meters
                ), f"Expected altitude: {vehicle_alt_ellipsoid_meters}, but got: {self.vehicle_geopose.pose.position.altitude}"
            elif self.vehicle_altitude is not None:
                egm96_height = self._egm96_height
                if self._egm96_height is not None:
                    vehicle_alt_ellipsoid_from_altitude = (
                        self.vehicle_geopose.amsl + egm96_height
                    )
                    assert (
                        vehicle_alt_ellipsoid_meters
                        == vehicle_alt_ellipsoid_from_altitude
                    ), f"Expected ellipsoid altitude: {vehicle_alt_ellipsoid_meters}, but got: {vehicle_alt_ellipsoid_from_altitude}"
            else:
                assert (
                    False
                ), f"Expected ellipsoid altitude: {vehicle_alt_ellipsoid_meters}, but no altitude message received."

        # Assert the ground track GeoPose and Altitude
        if ground_track_lat is not None:
            assert (
                self.ground_track_geopose.latitude == ground_track_lat
            ), f"Expected latitude: {vehicle_lat}, but got: {self.ground_track_geopose.latitude}"

        if ground_track_lon is not None:
            assert (
                self.ground_track_geopose.longitude == ground_track_lon
            ), f"Expected longitude: {vehicle_lon}, but got: {self.ground_track_geopose.longitude}"

        if ground_track_alt_amsl_meters is not None:
            if self.self.ground_track_geopose is not None:
                egm96_height = self._egm96_height
                if self._egm96_height is not None:
                    ground_track_alt_amsl_from_geopose = (
                        self.ground_track_geopose.pose.position.altitude - egm96_height
                    )
                    assert (
                        vehicle_alt_amsl_from_geopose == vehicle_alt_amsl_meters
                    ), f"Expected ground track AMSL elevation: {ground_track_alt_amsl_meters}, but got: {ground_track_alt_amsl_from_geopose}"
            elif self.ground_track_elevation is not None:
                assert (
                    self.ground_track_elevation.amsl == ground_track_alt_amsl_meters
                ), f"Expected ground track AMSL elevation: {ground_track_alt_amsl_meters}, but got: {self.ground_track_elevation}"
            else:
                assert (
                    False
                ), f"Expected ground track AMSL elevation: {ground_track_alt_amsl_meters}, but no elevation message received."

        if ground_track_elevation_ellipsoid_meters is not None:
            if self.self.ground_track_geopose is not None:
                assert (
                    self.ground_track_geopose.pose.position.altitude
                    == ground_track_elevation_ellipsoid_meters
                ), f"Expected ground track ellipsoid elevation: {ground_track_elevation_ellipsoid_meters}, but got: {self.ground_track_geopose.pose.position.altitude}"
            elif self.ground_track_elevation is not None:
                egm96_height = self._egm96_height
                if self._egm96_height is not None:
                    ground_track_elevation_ellipsoid_from_altitude = (
                        self.ground_track_elevation.amsl + egm96_height
                    )
                    assert (
                        ground_track_elevation_ellipsoid_from_altitude
                        == ground_track_elevation_ellipsoid_meters
                    ), f"Expected ground track ellipsoid elevation: {ground_track_elevation_ellipsoid_meters}, but got: {ground_track_elevation_ellipsoid_from_altitude}"
            else:
                assert (
                    False
                ), f"Expected ground track ellipsoid elevation: {ground_track_elevation_ellipsoid_meters}, but no elevation message received."

        if camera_pitch_ned_deg is not None:
            raise NotImplementedError

        if camera_yaw_ned_deg is not None:
            raise NotImplementedError

        if camera_roll_ned_deg is not None:
            raise NotImplementedError

        if bbox is not None:
            raise NotImplementedError

        if vehicle_heading_ned is not None:
            raise NotImplementedError
