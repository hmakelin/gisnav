import cv2
import numpy as np
from geographic_msgs.msg import GeoPoint, GeoPose
from geometry_msgs.msg import Pose
from mavros_msgs.msg import Altitude, GimbalDeviceAttitudeStatus, HomePosition
from sensor_msgs.msg import CameraInfo, Image, NavSatFix
from tf.transformations import quaternion_from_euler

from gisnav_msgs.msg import OrthoImage3D


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
