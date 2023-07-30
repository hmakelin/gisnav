from typing import Union

import cv2
import numpy as np
from geographic_msgs.msg import GeoPoint, GeoPose
from geometry_msgs.msg import Pose
from mavros_msgs.msg import Altitude, GimbalDeviceAttitudeStatus, HomePosition
from sensor_msgs.msg import CameraInfo, Image, NavSatFix
from tf.transformations import quaternion_from_euler

from gisnav_msgs.msg import OrthoImage3D

ROSMessage = Union[
    Pose,
    NavSatFix,
    HomePosition,
    GimbalDeviceAttitudeStatus,
    GeoPose,
    GeoPoint,
    CameraInfo,
    Image,
    Altitude,
    OrthoImage3D,
]


# TODO: make bbox mandatory?
def generate_state_messages(
    vehicle_lat: float,
    vehicle_lon: float,
    vehicle_alt_agl_meters: float,
    vehicle_heading_ned: float,
    camera_pitch_ned_deg: float,
    camera_yaw_ned_deg: float,
    camera_roll_ned_deg: float,
    home_lat: float,
    home_lon: float,
    home_altitude: float,
    calibration_file: str,
    image_file: str,
    dem_file: str = None,
    bbox: BoundingBox = None,
) -> Union[List[ROSMessage], Dict[str, ROSMessage]]:
    """
    Generates ROS messages describing the state of the :term:`drone` described by the input parameters

    :param vehicle_lat: Vehicle WGS 84 latitude coordinate in degrees, part of the :term:`Global position`
    :param vehicle_lon: Vehicle WGS 84 longitude coordinate in degrees, part of the :term:`Global position`
    :param vehicle_alt_agl_meters: Vehicle :term:`Altitude` above ground level (AGL) in meters
    :param vehicle_heading_ned: Vehicle heading in North-East-Down (:term:`Nadir`) coordinate system in degrees
    :param camera_pitch_ned_deg: :term:`Camera` pitch angle in North-East-Down coordinate system in degrees
    :param camera_yaw_ned_deg: :term:`Camera` yaw angle in North-East-Down coordinate system in degrees
    :param camera_roll_ned_deg: :term:`Camera` roll angle in North-East-Down coordinate system in degrees
    :param home_lat: Home WGS 84 latitude coordinate in degrees
    :param home_lon: Home WGS 84 longitude coordinate in degrees
    :param home_altitude: Home altitude in meters
    :param calibration_file: Path to the camera calibration matrix file, containing intrinsic parameters
    :param image_file: Path to the image file to be loaded for the :term:`Image` message
    :param dem_file: Path to the DEM file (saved as a NumPy array), optional. If not provided, a zero array will be used for the DEM.
    :param bbox: Bounding box for the orthoimage, optional. Represents the geographical box or rectangle that bounds the area of interest.
    :return: A list or dictionary of ROS messages representing the input state to the nodes being tested
    """
    # Generate the NavSatFix message
    navsatfix_msg = _generate_navsatfix_message(
        vehicle_lat, vehicle_lon, vehicle_alt_agl_meters
    )

    # Generate the GeoPose message
    geopose_msg = _generate_geopose_message(
        vehicle_lat, vehicle_lon, vehicle_alt_agl_meters, vehicle_heading_ned
    )

    # Generate the Altitude message
    altitude_msg = _generate_altitude_message(vehicle_alt_agl_meters)

    # Generate the HomePosition message
    home_position_msg = _generate_home_position_message(
        home_lat, home_lon, home_altitude
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

    # Return the messages as a list or dictionary
    return [
        navsatfix_msg,
        geopose_msg,
        altitude_msg,
        home_position_msg,
        gimbal_attitude_msg,
        camera_info_msg,
        image_msg,
        ortho_image_3d_msg,
    ]


def _generate_geopose_message(
    vehicle_lat: float,
    vehicle_lon: float,
    vehicle_alt_agl_meters: float,
    vehicle_heading_ned: float,
) -> GeoPose:
    """
    Generates a GeoPose ROS message based on the given vehicle latitude, longitude, altitude, and heading.

    :param vehicle_lat: Vehicle WGS 84 latitude coordinate in degrees
    :param vehicle_lon: Vehicle WGS 84 longitude coordinate in degrees
    :param vehicle_alt_agl_meters: Vehicle altitude above ground level (AGL) in meters
    :param vehicle_heading_ned: Vehicle heading in North-East-Down coordinate system in degrees
    :return: A GeoPose message representing the vehicle's geopose
    """
    geopose_msg = GeoPose()
    geopose_msg.position.latitude = vehicle_lat
    geopose_msg.position.longitude = vehicle_lon
    geopose_msg.position.altitude = vehicle_alt_agl_meters

    # Assuming the heading is represented as a quaternion
    # You may need to convert the heading to a quaternion if it's given in another format
    geopose_msg.orientation = heading_to_quaternion(vehicle_heading_ned)

    return geopose_msg


def _generate_navsatfix_message(
    vehicle_lat: float, vehicle_lon: float, vehicle_alt_agl_meters: float
) -> NavSatFix:
    """
    Generates a NavSatFix ROS message based on the given vehicle latitude, longitude, and altitude.

    :param vehicle_lat: Vehicle WGS 84 latitude coordinate in degrees
    :param vehicle_lon: Vehicle WGS 84 longitude coordinate in degrees
    :param vehicle_alt_agl_meters: Vehicle altitude above ground level (AGL) in meters
    :return: A NavSatFix message representing the vehicle's global position
    """
    navsatfix_msg = NavSatFix()
    navsatfix_msg.latitude = vehicle_lat
    navsatfix_msg.longitude = vehicle_lon
    navsatfix_msg.altitude = vehicle_alt_agl_meters
    navsatfix_msg.status.status = NavSatFix.STATUS_FIX  # Assuming a valid fix
    return navsatfix_msg


def _generate_altitude_message(vehicle_alt_agl_meters: float) -> Altitude:
    """
    Generates an Altitude ROS message based on the given vehicle altitude above ground level (AGL).

    :param vehicle_alt_agl_meters: Vehicle altitude above ground level (AGL) in meters
    :return: An Altitude message representing the vehicle's altitude
    """
    altitude_msg = Altitude()
    altitude_msg.altitude_amsl = vehicle_alt_agl_meters  # Assuming the altitude is AMSL
    # You may need to set other fields in the Altitude message depending on your requirements
    return altitude_msg


def _generate_home_position_message(
    home_lat: float, home_lon: float, home_altitude: float
) -> HomePosition:
    """
    Generates a HomePosition ROS message based on the given home latitude, longitude, and altitude.

    :param home_lat: Home WGS 84 latitude coordinate in degrees
    :param home_lon: Home WGS 84 longitude coordinate in degrees
    :param home_altitude: Home altitude in meters
    :return: A HomePosition message representing the vehicle's home position
    """
    home_position_msg = HomePosition()
    home_position_msg.latitude = home_lat
    home_position_msg.longitude = home_lon
    home_position_msg.altitude = home_altitude
    # You may need to set other fields in the HomePosition message depending on your requirements
    return home_position_msg


def _generate_gimbal_attitude_message(
    camera_pitch_ned_deg: float, camera_yaw_ned_deg: float, camera_roll_ned_deg: float
) -> GimbalDeviceAttitudeStatus:
    """
    Generates a GimbalDeviceAttitudeStatus ROS message based on the given camera orientation in nadir frame.

    :param camera_pitch_ned_deg: Camera pitch in nadir frame in degrees
    :param camera_yaw_ned_deg: Camera yaw in nadir frame in degrees
    :param camera_roll_ned_deg: Camera roll in nadir frame in degrees
    :return: A GimbalDeviceAttitudeStatus message representing the camera's orientation
    """
    gimbal_attitude_msg = GimbalDeviceAttitudeStatus()

    # Convert the nadir frame orientation to NED frame
    pitch_rad = np.radians(-camera_pitch_ned_deg)  # Inverting pitch to match NED frame
    yaw_rad = np.radians(camera_yaw_ned_deg)
    roll_rad = np.radians(camera_roll_ned_deg)

    # Convert Euler angles to quaternion
    quaternion = quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)

    gimbal_attitude_msg.q = quaternion
    # You may need to set other fields in the GimbalDeviceAttitudeStatus message depending on your requirements
    return gimbal_attitude_msg


def _generate_camera_info_message(calibration_file: str) -> CameraInfo:
    """
    Generates a CameraInfo ROS message based on the given calibration matrix file.

    :param calibration_file: Path to the calibration matrix file
    :return: A CameraInfo message representing the camera's calibration information
    """
    camera_info_msg = CameraInfo()

    # Load the calibration matrix from the file
    calibration_matrix = np.loadtxt(calibration_file)

    # Set the camera matrix (K) and distortion coefficients (D) if available
    camera_info_msg.K = calibration_matrix[:3, :3].flatten().tolist()
    # You may need to set other fields in the CameraInfo message depending on the contents of the calibration file

    return camera_info_msg


def _generate_image_message(image_file: str) -> Image:
    """
    Generates an Image ROS message based on the given image file.

    :param image_file: Path to the image file
    :return: An Image message representing the loaded image
    """
    image_msg = Image()

    # Load the image from the file
    image_data = cv2.imread(image_file)

    # Convert the image to a ROS-compatible format
    image_msg.height, image_msg.width, image_msg.channels = image_data.shape
    image_msg.encoding = "bgr8"  # Assuming the image is in BGR format
    image_msg.data = image_data.flatten().tolist()

    return image_msg


# TODO: generate this based on provided vehicle latitude?
def _generate_ortho_image_3d_message(
    image_file: str, dem_file: str = None, bbox: BoundingBox = None
) -> OrthoImage3D:
    """
    Generates an OrthoImage3D ROS message based on the given image file and optional DEM file.

    :param image_file: Path to the image file
    :param dem_file: Path to the DEM file (saved as a NumPy array), optional
    :param bbox: Bounding box for the orthoimage, optional
    :return: An OrthoImage3D message representing the loaded orthoimage and DEM
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
