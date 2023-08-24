from typing import Optional, Tuple

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geographic_msgs.msg import BoundingBox, GeoPose
from geometry_msgs.msg import Pose, Quaternion
from mavros_msgs.msg import Altitude, GimbalDeviceAttitudeStatus, HomePosition
from pygeodesy.ellipsoidalNvector import LatLon, Nvector
from pygeodesy.geoids import GeoidPGM
from pygeodesy.ltpTuples import Enu
from pygeodesy.ltp import LocalCartesian
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import CameraInfo, Image, NavSatFix

from gisnav._decorators import ROS
from gisnav.static_configuration import (
    GIS_NODE_NAME,
    ROS_NAMESPACE,
    ROS_TOPIC_RELATIVE_ORTHOIMAGE,
)
from gisnav_msgs.msg import OrthoImage3D


class MockStatePublisherNode(Node):
    """
    A :term:`ROS` node that publishes sets of :term:`messages` mocking the
    external interfaces of the :term:`core` ROS nodes.

    :param name: Node name
    """

    D_VEHICLE_LAT: float = 37.523640
    D_VEHICLE_LON: float = -122.255122
    D_VEHICLE_ALT_ELLIPSOID_METERS: float = 120.0
    D_VEHICLE_ALT_AMSL_METERS: float = (
        120.0  # TODO : make consistent with ellipsoid alt
    )
    D_VEHICLE_HEADING_NED_DEG: float = 0.0
    D_CAMERA_PITCH_NED_DEG: float = 0.0
    D_CAMERA_YAW_NED_DEG: float = 0.0
    D_CAMERA_ROLL_NED_DEG: float = 0.0
    D_HOME_LAT: float = 37.523640
    D_HOME_LON: float = -122.255122
    D_HOME_ELEVATION_ELLIPSOID_METERS: float = 0.0
    D_DEM = np.zeros(
        (735, 735), np.uint16
    )  # TODO: should be uint16 because 255 meters is not enough
    D_ORTHOPHOTO = np.zeros((735, 735, 3), np.uint8)
    D_BBOX = None  # TODO

    def __init__(self, name):
        super().__init__(name)
        self._cv_bridge = CvBridge()
        # TODO: do not hard code path
        self._egm96 = GeoidPGM("/usr/share/GeographicLib/geoids/egm96-5.pgm", kind=-3)

    @ROS.publish(
        "camera/camera_info",
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def camera_info(self, k: np.ndarray) -> CameraInfo:
        """
        Publishes a :class:`sensor_msgs.msg.CameraInfo` :term:`ROS` message
        based on given camera intrinsics matrix.

        :param k: Camera intrinsics matrix of shape (3, 3)
        :return: A :class:`sensor_msgs.msg.CameraInfo` message representing
            the camera intrinsics
        """
        camera_info = CameraInfo()
        camera_info.height, camera_info.width = 2 * int(k[0][2]), 2 * int(k[1][2])
        camera_info.k = k.reshape(-1)
        return camera_info

    @ROS.publish(
        "camera/image_raw",
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def image(self, image_rgb8: np.ndarray) -> Image:
        """
        Publishes a :class:`sensor_msgs.msg.Image` :term:`ROS` message
        based on image matrix (height, width, channels) where channels is 3 (RGB)

        :param image_rgb8: RGB image (height, width, channels). Will be encoded
            to rgb8 (even if e.g. np.float32).
        :return: A :class:`sensor_msgs.msg.Image` message
        """
        # return self._cv_bridge.cv2_to_imgmsg(image, encoding="passthrough")
        return self._cv_bridge.cv2_to_imgmsg(image_rgb8, encoding="rgb8")

    @ROS.publish(
        "mavros/global_position/global",
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def nav_sat_fix(
        self,
        vehicle_lat_degrees: float,
        vehicle_lon_degrees: float,
        vehicle_alt_ellipsoid_meters: float,
    ) -> NavSatFix:
        """
        Publishes a :class:`sensor_msgs.msg.NavSatFix` :term:`ROS` message
        based on given :term:`vehicle` :term:`WGS 84` latitude and longitude
        coordinates and :term:`ellipsoide` altitude in meters.

        :param vehicle_lat_degrees: Vehicle WGS 84 latitude coordinate in degrees
        :param vehicle_lon_degrees: Vehicle WGS 84 longitude coordinate in degrees
        :param vehicle_alt_ellipsoid_meters: Vehicle ellipsoid altitude in meters
        :return: A :class:`sensor_msgs.msg.NavSatFix` message representing
            the vehicle's :term:`global position`
        """
        navsatfix_msg = NavSatFix()
        navsatfix_msg.latitude = vehicle_lat_degrees
        navsatfix_msg.longitude = vehicle_lon_degrees
        navsatfix_msg.altitude = vehicle_alt_ellipsoid_meters
        return navsatfix_msg

    @ROS.publish(
        "gisnav/gis_node/vehicle/geopose",
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def geopose(
        self,
        vehicle_lat_degrees: float,
        vehicle_lon_degrees: float,
        vehicle_alt_ellipsoid_meters: float,
        vehicle_heading_ned: float,
    ) -> GeoPose:
        """
        Publishes a :term:`geographic_msgs.msg.GeoPose` :term:`ROS` message
        based on given :term:`vehicle` :term:`WGS 84` latitude and longitude
        coordinates and :term:`ellipsoid` altitude in meters, and heading
        in :term:`NED` frame in degrees.

        :param vehicle_lat_degrees: Vehicle WGS 84 latitude coordinate in degrees
        :param vehicle_lon_degrees: Vehicle WGS 84 longitude coordinate in degrees
        :param vehicle_alt_ellipsoid_meters: Vehicle ellipsoid altitude in meters
        :param vehicle_heading_ned: Vehicle heading in :term:`NED` frame in degrees
        :return: A :term:`geographic_msgs.msg.GeoPose` message representing the
            vehicle's :term:`geopose`
        """
        geopose_msg = GeoPose()
        geopose_msg.position.latitude = vehicle_lat_degrees
        geopose_msg.position.longitude = vehicle_lon_degrees
        geopose_msg.position.altitude = vehicle_alt_ellipsoid_meters
        geopose_msg.orientation = heading_to_quaternion(vehicle_heading_ned)
        return geopose_msg

    @ROS.publish(
        "gisnav/gis_node/vehicle/altitude",
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def altitude(
        self, vehicle_alt_amsl_meters: float, vehicle_alt_agl_meters: float
    ) -> Altitude:
        """
        Publishes a :term:`mavros_msgs.msg.Altitude` :term:`ROS` message
        based on given :term:`vehicle` :term:`WGS 84` :term:`AMSL` and :term:`AGL`
        altitudes in meters.

        :param vehicle_alt_amsl_meters: Vehicle altitude AMSL in meters
        :param vehicle_alt_agl_meters: Vehicle altitude AGL in meters
        :return: A :term:`mavros_msgs.msg.Altitude` message representing the
            vehicle's altitude
        """
        altitude_msg = Altitude()
        altitude_msg.amsl = vehicle_alt_amsl_meters
        altitude_msg.terrain = vehicle_alt_agl_meters
        return altitude_msg

    @ROS.publish(
        "mavros/home_position/home",
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def home_position(
        self, home_lat: float, home_lon: float, home_elevation_ellipsoid_meters: float
    ) -> HomePosition:
        """
        Generates a :class:`mavros_msgs.msg.HomePosition` :term:`ROS` message
        based on the given home :term:`global position` :term:`WGS 84` latitude and
        longitude in degrees, and :term:`AMSL` :term:`elevation` in meters.

        :param home_lat: Home WGS 84 latitude coordinate in degrees
        :param home_lon: Home WGS 84 longitude coordinate in degrees
        :param home_elevation_ellipsoid_meters: Home AMSL elevation in meters
        :return: A :class:`mavros_msgs.msg.HomePosition` message representing
            the vehicle's home :term:`global position`
        """
        home_position_msg = HomePosition()
        home_position_msg.geo.latitude = home_lat
        home_position_msg.geo.longitude = home_lon
        home_position_msg.geo.altitude = home_elevation_ellipsoid_meters
        return home_position_msg

    @ROS.publish(
        f"/{ROS_NAMESPACE}"
        f'/{ROS_TOPIC_RELATIVE_ORTHOIMAGE.replace("~", GIS_NODE_NAME)}',
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def orthoimage(
        self, image_file: str, bbox: BoundingBox, dem_file: str = None
    ) -> OrthoImage3D:
        """
        Publishes a :class:`gisnav_msgs.msg.OrthoImage3D` :term:`ROS` message
        based on the given image file and optional :term:`DEM` file.

        :param image_file: Path to the image file
        :param bbox: Bounding box for the orthoimage
        :param dem_file: Path to the DEM file (saved as a NumPy array), optional
        :return: A :class:`gisnav_msgs.msg.OrthoImage3D` message representing
            the loaded orthoimage and DEM
        """
        ortho_image_3d_msg = OrthoImage3D()

        # Load the image from the file
        image_data = cv2.imread(image_file)
        image_msg = Image()
        image_msg.height, image_msg.width, image_msg.channels = image_data.shape
        image_msg.encoding = "bgr8"  # Assuming the image is in BGR format
        image_msg.data = image_data.flatten().tolist()
        ortho_image_3d_msg.img = image_msg

        # Load the DEM from the file or create a zero array if not provided
        dem_msg = Image()
        if dem_file:
            dem_data = np.load(dem_file)
        else:
            dem_data = np.zeros_like(
                image_data[:, :, 0]
            )  # Assuming the DEM has the same dimensions as the image
        dem_msg.height, dem_msg.width = dem_data.shape
        dem_msg.encoding = "mono16"  # Assuming the DEM is a single-channel 16-bit image
        dem_msg.data = dem_data.flatten().tolist()
        ortho_image_3d_msg.dem = dem_msg

        # Set the bounding box if provided
        if bbox:
            ortho_image_3d_msg.bbox = bbox

        return ortho_image_3d_msg

    @ROS.publish(
        "mavros/local_position/pose",
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def local_position(
        self,
        vehicle_lat: float = D_VEHICLE_LAT,
        vehicle_lon: float = D_VEHICLE_LON,
        vehicle_alt_ellipsoid_meters: float = D_VEHICLE_ALT_ELLIPSOID_METERS,
        home_lat: float = D_HOME_LAT,
        home_lon: float = D_HOME_LON,
        home_elevation_ellipsoid_meters: float = D_HOME_ELEVATION_ELLIPSOID_METERS,
    ) -> Pose:
        """Publishes a :class:`geometry_msgs.msg.Pose` :term:`ROS` message
        based on the given :term:`vehicle` :term:`global position` and
        :term:`home` position in :term:`NED` frame.

        :param vehicle_lat: Vehicle :term:`WGS 84` latitude coordinate in degrees
        :param vehicle_lon: Vehicle :term:`WGS 84` longitude coordinate in degrees
        :param vehicle_alt_ellipsoid_meters: Vehicle :term:`ellipsoid` :term:`altitude`
            in meters
        :param home_lat: Home :term:`WGS 84` latitude coordinate in degrees
        :param home_lon: Home :term:`WGS 84` longitude coordinate in degrees
        :param home_elevation_ellipsoid_meters: Home :term:`ellipsoid`
            :term:`elevation` in meters
        :return: A :class:`geometry_msgs.msg.Pose` message representing the vehicle's
            local position
        """

        def _wgs84_to_enu(lat, lon, alt, origin_lat, origin_lon, origin_alt):
            """Convert WGS84 to ENU using pygeodesy"""
            # Define the origin point (home position)
            origin = LatLon(origin_lat, origin_lon, origin_alt)
            local_cartesian = LocalCartesian(origin)

            # Convert vehicle position to local cartesian coordinates
            local_coords = local_cartesian.forward(LatLon(lat, lon, alt))

            # Extract ENU coordinates from the Local9Tuple
            e, n, u = local_coords.x, local_coords.y, local_coords.z

            return e, n, u

        e, n, u = _wgs84_to_enu(
            vehicle_lat,
            vehicle_lon,
            vehicle_alt_ellipsoid_meters,
            home_lat,
            home_lon,
            home_elevation_ellipsoid_meters,
        )

        pose = Pose()
        pose.position.x = e
        pose.position.y = n
        pose.position.z = u

        return pose

    @ROS.publish(
        "mavros/gimbal_control/device/attitude_status",
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def gimbal_device_attitude_status(
        self,
        camera_pitch_ned_deg: float,
        camera_yaw_ned_deg: float,
        camera_roll_ned_deg: float,
    ) -> GimbalDeviceAttitudeStatus:
        """
        Publishes a :class:`mavros_msgs.msg.GimbalDeviceAttitudeStatus` :term:`ROS`
            message based on the given :term:`camera` :term:`orientation`
            in :term:`NED` frame.

        :param camera_pitch_ned_deg: Camera pitch in NED frame in degrees
        :param camera_yaw_ned_deg: Camera yaw in NED frame in degrees
        :param camera_roll_ned_deg: Camera roll in NED frame in degrees
        :return: A :class:`mavros_msgs.msg.GimbalDeviceAttitudeStatus` message
            representing the camera's orientation
        """
        gimbal_attitude_msg = GimbalDeviceAttitudeStatus()

        # Convert the nadir frame orientation to NED frame
        pitch_rad = np.radians(
            -camera_pitch_ned_deg
        )  # Inverting pitch to match NED frame
        yaw_rad = np.radians(camera_yaw_ned_deg)
        roll_rad = np.radians(camera_roll_ned_deg)

        # Convert Euler angles to quaternion
        q = self.quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)
        quaternion = Quaternion(
            w=q[0],
            x=q[1],
            y=q[2],
            z=q[3],
        )

        gimbal_attitude_msg.q = quaternion
        return gimbal_attitude_msg

    def publish_mavros_state(
        self,
        vehicle_lat: float = D_VEHICLE_LAT,
        vehicle_lon: float = D_VEHICLE_LON,
        vehicle_alt_ellipsoid_meters: float = D_VEHICLE_ALT_ELLIPSOID_METERS,
        vehicle_alt_amsl_meters: Optional[float] = None,
        camera_pitch_ned_deg: float = D_CAMERA_PITCH_NED_DEG,
        camera_yaw_ned_deg: float = D_CAMERA_YAW_NED_DEG,
        camera_roll_ned_deg: float = D_CAMERA_ROLL_NED_DEG,
        home_lat: float = D_HOME_LAT,
        home_lon: float = D_HOME_LON,
        home_elevation_ellipsoid_meters: float = D_HOME_ELEVATION_ELLIPSOID_METERS,
    ) -> None:
        """
        Publishes :term:`ROS` messages describing the state of the :term:`vehicle`
        described as received from :term:`MAVROS` in the :term:`core` configuration.

        The default state has the vehicle heading facing north with camera at origin
        in NED frame, defined as pointing down nadir with image top facing north
        with the vehicle as well. :term:`Global position` is the starting location
        of the KSQL airport simulation with :term:`ellipsoid` altitude
        at 120 meters. :term:`Home position` is defined as the simulation
        starting location.

        # TODO: fix home position ellipsoide altitude (AMSL altitude should be
            around 1.7 meters)

        :param vehicle_lat: Vehicle :term:`WGS 84` latitude coordinate in degrees
        :param vehicle_lon: Vehicle :term:`WGS 84` longitude coordinate in degrees
        :param vehicle_alt_ellipsoid_meters: Vehicle :term:`ellipsoid` :term:`altitude`
            in meters
        :param vehicle_alt_amsl_meters: Vehicle :term:`altitude` :term:`AMSL` in meters
            Takes precedence over ``vehicle_alt_ellipsoid_meters`` if provided.
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
        """
        if vehicle_alt_amsl_meters is not None:
            vehicle_alt_ellipsoid_meters = vehicle_alt_amsl_meters + self._egm96.height(
                vehicle_lat,
                vehicle_lon,
            )

        self.nav_sat_fix(vehicle_lat, vehicle_lon, vehicle_alt_ellipsoid_meters)
        self.home_position(home_lat, home_lon, home_elevation_ellipsoid_meters)
        self.gimbal_device_attitude_status(
            camera_pitch_ned_deg, camera_yaw_ned_deg, camera_roll_ned_deg
        )
        self.local_position(
            vehicle_lat,
            vehicle_lon,
            vehicle_alt_ellipsoid_meters,
            home_lat,
            home_lon,
            home_elevation_ellipsoid_meters,
        )

    def publish_camera_state(
        self,
        intrinsics_matrix: np.ndarray = np.array(
            [[205, 0, 240], [0, 205, 320], [0, 0, 1]], np.float64
        ),
        image: np.ndarray = np.zeros((480, 640, 3), np.uint8),
    ) -> None:
        """
        Publishes :term:`ROS` messages describing the state of the :term:`camera.

        :param intrinsics_matrix: Camera intrinsics matrix of shape (3, 3)
        :param image: Camera :term:`query image` of shape (height, width, channels)
            where channels is 3 (RGB)
        """
        self.camera_info(intrinsics_matrix)
        self.image(image)

    # TODO: add default values
    def publish_gisnode_state(
        self,
        vehicle_lat: float = D_VEHICLE_LAT,
        vehicle_lon: float = D_VEHICLE_LON,
        vehicle_alt_agl_meters: float = D_VEHICLE_ALT_AMSL_METERS,
        vehicle_heading_ned: float = D_VEHICLE_HEADING_NED_DEG,
        camera_pitch_ned_deg: float = D_CAMERA_PITCH_NED_DEG,
        camera_yaw_ned_deg: float = D_CAMERA_YAW_NED_DEG,
        camera_roll_ned_deg: float = D_CAMERA_ROLL_NED_DEG,
        dem: np.ndarray = D_DEM,
        orthophoto: np.ndarray = D_ORTHOPHOTO,
        bbox: BoundingBox = D_BBOX,
    ) -> None:
        """
        Publishes :term:`ROS` messages describing the state of the :term:`vehicle`
        as received from :class:`.GISNode` and subscribed to by :class:`.CVNode`.

        :param vehicle_lat: Vehicle :term:`WGS 84` latitude coordinate in degrees
        :param vehicle_lon: Vehicle :term:`WGS 84` longitude coordinate in degrees
        :param vehicle_alt_agl_meters: Vehicle :term:`altitude` :term:`AGL` in meters
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
        :param dem: :term:`DEM` (saved as a NumPy array) raster
        :param orthophoto: :term:`Orthophoto` (saved as a NumPy array) raster
        :param bbox: :term:`Bounding box` for the :term:`orthoimage`, optional.
        """
        # TODO: add many more here
        self.geopose(
            vehicle_lat, vehicle_lon, vehicle_alt_amsl_meters, vehicle_heading_ned
        )
        self.altitude(vehicle_alt_agl_meters)
        self.orthoimage(orthophoto, dem, bbox)
        # TODO: terrain elevation, terrain geopose, camera quaternion (not in mermaid graph)

        self.camera_quaternion(
            camera_pitch_ned_deg, camera_yaw_ned_deg, camera_roll_ned_deg
        )

    @staticmethod
    def quaternion_from_euler(
        roll: float, pitch: float, yaw: float
    ) -> Tuple[float, float, float, float]:
        """Returns a (w, x, y, z) quaternion tuple

        :param roll: Roll euler angle in degrees
        :param pitch: Pitch euler angle in degrees
        :param yaw: Yaw euler angle in degrees
        :return: A (w, x, y, z) quaternion tuple
        """
        roll, pitch, yaw = tuple(map(np.radians, (roll, pitch, yaw)))

        # Calculate the sine and cosine values
        cy = np.cos(yaw * 0.5)
        sy = np.sin(yaw * 0.5)
        cp = np.cos(pitch * 0.5)
        sp = np.sin(pitch * 0.5)
        cr = np.cos(roll * 0.5)
        sr = np.sin(roll * 0.5)

        # Calculate the quaternion components
        w = cr * cp * cy + sr * sp * sy
        x = sr * cp * cy - cr * sp * sy
        y = cr * sp * cy + sr * cp * sy
        z = cr * cp * sy - sr * sp * cy

        return [w, x, y, z]
