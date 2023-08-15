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
    def altitude(self) -> Optional[Altitude]:
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
    def geopose(self) -> Optional[GeoPoseStamped]:
        """Vehicle GeoPoseStamped, or None if not available or too old"""

    def assert_state(
        self,
        vehicle_lat: Optional[float] = None,
        vehicle_lon: Optional[float] = None,
        vehicle_alt_amsl_meters: Optional[float] = None,
        vehicle_heading_ned: Optional[float] = None,
        camera_pitch_ned_deg: Optional[float] = None,
        camera_yaw_ned_deg: Optional[float] = None,
        camera_roll_ned_deg: Optional[float] = None,
        home_lat: Optional[float] = None,
        home_lon: Optional[float] = None,
        home_elevation_ellipsoid_meters: Optional[float] = None,
        calibration_file: Optional[str] = None,
        image_file: Optional[str] = None,
        dem_file: Optional[str] = None,
        bbox: Optional[BoundingBox] = None,
    ) -> None:
        """
        Asserts latest state

        In an example use case you would only need to assert a specific subset
        of the input arguments (a partial state) that you are testing, all other
        messages being None by default (=no message received).

        :param vehicle_lat: Vehicle :term:`WGS 84` latitude coordinate in degrees
        :param vehicle_lon: Vehicle :term:`WGS 84` longitude coordinate in degrees
        :param vehicle_alt_amsl_meters: Vehicle :term:`AMSL` :term:`altitude`
            in meters
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
        :param home_lat: Home :term:`WGS 84` latitude coordinate in degrees
        :param home_lon: Home :term:`WGS 84` longitude coordinate in degrees
        :param home_elevation_ellipsoid_meters: Home :term:`ellipsoid`
            :term:`elevation` in meters
        :param calibration_file: Path to the :term:`camera` calibration matrix
            file, containing camera intrinsic parameters
        :param image_file: Path to the image file to be loaded for the
            :term:`query image` :term:`ROS` message
        :param dem_file: Path to the :term:`DEM` file (saved as a NumPy array),
            optional. If not provided, a zero array will be used for the DEM.
        :param bbox: :term:`Bounding box` for the :term:`orthoimage`, optional.
        # TODO: terrain elevation, terrain geopose, camera quaternion (not in mermaid graph)
        """
        raise NotImplementedError
