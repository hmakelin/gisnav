import rclpy
from rclpy.node import Node
from gisnav_msgs.msg import OrthoImage3D
from sensor_msgs.msg import NavSatFix, Image
from mavros_msgs.msg import HomePosition, GimbalDeviceAttitudeStatus
from geographic_msgs.msg import GeoPose
from mavros_msgs.msg import Altitude

from gisnav.gisnav._decorators import ROS


class StatePublisher(Node):
    """
    A ROS node that publishes full GISNav state messages.

    This node publishes messages including NavSatFix, GeoPose, Altitude, HomePosition,
    GimbalDeviceAttitudeStatus, Image, and OrthoImage3D. These messages describe the
    full state of the :term:`vehicle`.

    :param Node: Inherits from rclpy's Node class
    """

    def __init__(self):
        super().__init__('state_publisher')

        # Publishers for each message type
        #self.navsatfix_pub = self.create_publisher(NavSatFix, 'mavros/global_position/global', 10)
        #self.geopose_pub = self.create_publisher(GeoPose, 'gisnav/gis_node/vehicle/geopose', 10)
        #self.altitude_pub = self.create_publisher(Altitude, 'gisnav/gis_node/vehicle/altitude', 10)
        #self.home_position_pub = self.create_publisher(HomePosition, 'mavros/home_position/home', 10)
        self.gimbal_attitude_pub = self.create_publisher(GimbalDeviceAttitudeStatus, 'mavros/gimbal_control/device/attitude_status', 10)
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.ortho_image_3d_pub = self.create_publisher(OrthoImage3D, 'gisnav_msgs/OrthoImage3D', 10)

        # Timer to publish messages periodically
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.publish_messages)

    @ROS.publish(
        'mavros/global_position/global',
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def nav_sat_fix(self, vehicle_lat_degrees: float, vehicle_lon_degrees: float, vehicle_alt_agl_meters: float) -> NavSatFix:
        """
        Publishes a :class:`sensor_msgs.msg.NavSatFix` :term:`ROS` message
        based on given :term:`vehicle` :term:`WGS 84` latitude and longitude
        coordinates and :term:`AGL` altitude in meters.

        :param vehicle_lat_degrees: Vehicle WGS 84 latitude coordinate in degrees
        :param vehicle_lon_degrees: Vehicle WGS 84 longitude coordinate in degrees
        :param vehicle_alt_agl_meters: Vehicle altitude AGL in meters
        :return: A :class:`sensor_msgs.msg.NavSatFix` message representing
            the vehicle's :term:`global position`
        """
        navsatfix_msg = NavSatFix()
        navsatfix_msg.latitude = vehicle_lat_degrees
        navsatfix_msg.longitude = vehicle_lon_degrees
        navsatfix_msg.altitude = vehicle_alt_agl_meters
        return navsatfix_msg

    @ROS.publish(
        'gisnav/gis_node/vehicle/geopose',
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
        'gisnav/gis_node/vehicle/altitude',
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def altitude(self, vehicle_alt_agl_meters: float) -> Altitude:
        """
        Generates an Altitude ROS message based on the given vehicle altitude above ground level (AGL).

        Publishes a :term:`mavros_msgs.msg.Altdeitu` :term:`ROS` message
        based on given :term:`vehicle` :term:`AGL` altitude in meters.

        :param vehicle_alt_agl_meters: Vehicle altitude above ground level (AGL) in meters
        :return: An Altitude message representing the vehicle's altitude
        """
        altitude_msg = Altitude()
        altitude_msg.altitude_amsl = vehicle_alt_agl_meters  # Assuming the altitude is AMSL
        # You may need to set other fields in the Altitude message depending on your requirements
        return altitude_msg

    @ROS.publish(
        'mavros/home_position/home',
        QoSPresetProfiles.SENSOR_DATA.value,
    )
    def home_position(
        self, home_lat: float, home_lon: float, home_elevation_ellipsoid_meters: float
    ) -> HomePosition:
        """
        Generates a :class:`mavros_msgs.msg.HomePosition` :term:`ROS` message
        based on the given home :term:`global position` :term:`WGS 84` latitude and
        longitude in degrees, and :term:`AMSL` altitude in meters.

        :param home_lat: Home WGS 84 latitude coordinate in degrees
        :param home_lon: Home WGS 84 longitude coordinate in degrees
        :param home_elevation_ellipsoid_meters: Home :term:`ellipsoid`
            :term:`elevation` in meters
        :return: A HomePosition message representing the vehicle's home position
        """
        home_position_msg = HomePosition()
        home_position_msg.latitude = home_lat
        home_position_msg.longitude = home_lon
        home_position_msg.altitude = home_elevation_ellipsoid_meters
        return home_position_msg

    def publish_full_state(
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
        calibration_file: str,
        image_file: str,
        dem_file: str = None,
        bbox: BoundingBox = None,
    ) -> None:
        """
        Publishes :term:`ROS` messages describing the state of the :term:`vehicle`
        described by the input arguments.

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
        :return: A list or dictionary of :term:`ROS` messages representing the
            input state to the :term:`nodes <node>` being tested.
        """
        # Publish vehicle global position as NavSatFix message
        self.nav_sat_fix(
            vehicle_lat, vehicle_lon, vehicle_alt_agl_meters
        )

        # Publish vehicle global position and heading as GeoPose message
        self.geopose(
            vehicle_lat, vehicle_lon, vehicle_alt_agl_meters, vehicle_heading_ned
        )

        # Publish vehicle altitude as Altitude message
        self.altitude(vehicle_alt_agl_meters)

        # Publish home global position as the HomePosition message
        self.home_position(
            self, home_lat, home_lon, home_elevation_ellipsoid_meters
        )

        # Generate the GimbalDeviceAttitudeStatus message
        gimbal_attitude_msg = _generate_gimbal_attitude_message(
            camera_pitch_ned_deg, camera_yaw_ned_deg, camera_roll_ned_deg
        )
        # Generate the CameraInfo message
        camera_info_msg = _generate_camera_info_message(calibration_file)

        # Generate the Image message
        image_msg = _generate_image_message(image_file)

        # Generate the OrthoImage3D message
        ortho_image_3d_msg = _generate_ortho_image_3d_message(image_file, dem_file, bbox)


