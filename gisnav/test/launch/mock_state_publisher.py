import rclpy
from geographic_msgs.msg import BoundingBox, GeoPose
from mavros_msgs.msg import Altitude, GimbalDeviceAttitudeStatus, HomePosition
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image, NavSatFix

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

    def __init__(self, name):
        super().__init__(name)

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
        assert False  # TODO use gisnav to ros quaternion utility function
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
        self, home_lat: float, home_lon: float, home_elevation_amsl_meters: float
    ) -> HomePosition:
        """
        Generates a :class:`mavros_msgs.msg.HomePosition` :term:`ROS` message
        based on the given home :term:`global position` :term:`WGS 84` latitude and
        longitude in degrees, and :term:`AMSL` :term:`elevation` in meters.

        :param home_lat: Home WGS 84 latitude coordinate in degrees
        :param home_lon: Home WGS 84 longitude coordinate in degrees
        :param home_elevation_amsl_meters: Home AMSL elevation in meters
        :return: A :class:`mavros_msgs.msg.HomePosition` message representing
            the vehicle's home :term:`global position`
        """
        home_position_msg = HomePosition()
        home_position_msg.latitude = home_lat
        home_position_msg.longitude = home_lon
        home_position_msg.altitude = home_elevation_amsl_meters
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
        quaternion = quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)

        gimbal_attitude_msg.q = quaternion
        return gimbal_attitude_msg

    def publish_mavros_state(
        self,
        vehicle_lat: float,
        vehicle_lon: float,
        vehicle_alt_agl_meters: float,
        vehicle_heading_ned: float,
        camera_pitch_ned_deg: float,
        camera_yaw_ned_deg: float,
        camera_roll_ned_deg: float,
        home_lat: float,
        home_lon: float,
        home_elevation_ellipsoid_meters: float,
    ) -> None:
        """
        Publishes :term:`ROS` messages describing the state of the :term:`vehicle`
        described as received from :term:`MAVROS` in the :term:`core` configuration.

        :param vehicle_lat: Vehicle :term:`WGS 84` latitude coordinate in degrees
        :param vehicle_lon: Vehicle :term:`WGS 84` longitude coordinate in degrees
        :param vehicle_alt_ellipsoid_meters: Vehicle :term:`ellipsoid` :term:`altitude`
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
        """
        self.nav_sat_fix(vehicle_lat, vehicle_lon, vehicle_alt_ellipsoid_meters)
        self.home_position(self, home_lat, home_lon, home_elevation_ellipsoid_meters)
        self.gimbal_device_attitude_status(
            camera_pitch_ned_deg, camera_yaw_ned_deg, camera_roll_ned_deg
        )

        # TODO: local position Pose (need home altitude, relative altitude?)
        self.local_position(vehicle_alt_agl_meters)

    def publish_camera_state(
        self,
        calibration_file: str,
        image_file: str,
    ) -> None:
        """
        Publishes :term:`ROS` messages describing the state of the :term:`camera.

        :param calibration_file: Path to the :term:`camera` calibration matrix
            file, containing camera intrinsic parameters
        :param image_file: Path to the image file to be loaded for the
            :term:`query image` :term:`ROS` message
        """

        # Generate the CameraInfo message
        _generate_camera_info_message(calibration_file)

        # Generate the Image message
        _generate_image_message(image_file)

    def publish_gisnode_state(
        self,
        vehicle_lat: float,
        vehicle_lon: float,
        vehicle_alt_agl_meters: float,
        vehicle_heading_ned: float,
        camera_pitch_ned_deg: float,
        camera_yaw_ned_deg: float,
        camera_roll_ned_deg: float,
        dem_file: str = None,
        bbox: BoundingBox = None,
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
        :param dem_file: Path to the :term:`DEM` file (saved as a NumPy array),
            optional. If not provided, a zero array will be used for the DEM.
        :param bbox: :term:`Bounding box` for the :term:`orthoimage`, optional.
        :return: A list or dictionary of :term:`ROS` messages representing the
            input state to the :term:`nodes <node>` being tested.
        """
        # TODO: add many more here
        self.geopose(
            vehicle_lat, vehicle_lon, vehicle_alt_agl_meters, vehicle_heading_ned
        )
        self.altitude(vehicle_alt_agl_meters)
        self.orthoimage(image_file, dem_file, bbox)
        # TODO: terrain elevation, terrain geopose, camera quaternion (not in mermaid graph)

    def quaternion_from_euler(roll, pitch, yaw):
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
