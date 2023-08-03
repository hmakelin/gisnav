from typing import Union

import cv2
import numpy as np
from geographic_msgs.msg import GeoPoint, GeoPose
from geometry_msgs.msg import Pose
from mavros_msgs.msg import Altitude, GimbalDeviceAttitudeStatus, HomePosition
from sensor_msgs.msg import CameraInfo, Image, NavSatFix
from tf.transformations import quaternion_from_euler

from gisnav_msgs.msg import OrthoImage3D







def generate_gimbal_attitude_message(
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


def generate_camera_info_message(calibration_file: str) -> CameraInfo:
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


def generate_image_message(image_file: str) -> Image:
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
def generate_ortho_image_3d_message(
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
