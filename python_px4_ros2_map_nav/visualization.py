import cv2
import numpy as np

from scipy.spatial.transform import Rotation
from python_px4_ros2_map_nav.data import OutputData, RPY
from python_px4_ros2_map_nav.transform import make_keypoint


class Visualization:
    """Class for managing and displaying a visualization of keypoint matches

    Creates a visualization with 4 images indicating the map keypoint matches in the top two images and optional visual
    odometry keypoint matches in the bottom two images:

    [current frame][latest map frame]
    [current frame][ previous frame ]

    Intended to be used as debugging aid.
    """

    def __init__(self, name_: str) -> None:
        """Initialize visualization

        :param name: Display name of the visualization
        """
        self.name = name_
        self._map_visualization = None
        self._vo_visualization = None

    def update(self, output_data: OutputData, visual_odometry: bool) -> None:
        """Updates the visualization

        :param output_data: Data to update the visualization with
        :param visual_odometry: True to update visual odometry visualization, False for map visualization
        :return:
        """
        img = self._create_visualization(output_data, self._attitude_text(output_data.attitude), visual_odometry)
        img = img.astype(np.uint8)

        if visual_odometry:
            self._vo_visualization = img
            if self._map_visualization is None:
                self._map_visualization = np.zeros(img.shape, dtype=np.uint8)
        else:
            self._map_visualization = img
            if self._vo_visualization is None:
                self._vo_visualization = np.zeros(img.shape, dtype=np.uint8)

        out = np.vstack((self._map_visualization, self._vo_visualization))
        cv2.imshow(self.name, out)
        cv2.waitKey(1)

    @staticmethod
    def _create_visualization(output_data: OutputData, display_text: str, visual_odometry: bool) -> np.ndarray:
        """Visualizes a homography including keypoint matches and field of view.

        :param output_data: OutputData from the matching
        :param display_text: Text to display on visualization
        :param visual_odometry: True to update visual odometry visualization, False for map visualization
        :return: Visualized image as numpy array
        """
        # Make a list of cv2.DMatches that match mkp_img and mkp_map one-to-one
        kp_count = len(output_data.mkp_img)
        assert kp_count == len(output_data.mkp_map), 'Keypoint counts for img and map did not match.'
        matches = list(map(lambda i_: cv2.DMatch(i_, i_, 0), range(0, kp_count)))

        # Need cv2.KeyPoints for keypoints
        mkp_img = np.apply_along_axis(make_keypoint, 1, output_data.mkp_img)
        mkp_map = np.apply_along_axis(make_keypoint, 1, output_data.mkp_map)

        ref_img = output_data.input.previous_image if visual_odometry else output_data.input.map_cropped
        map_with_fov = cv2.polylines(ref_img.copy(), [np.int32(output_data.fov_pix)], True, 255, 3, cv2.LINE_AA)
        draw_params = dict(matchColor=(0, 255, 0), singlePointColor=None, matchesMask=None, flags=2)
        out = cv2.drawMatches(output_data.input.image_data.image, mkp_img, map_with_fov, mkp_map, matches, None,
                              **draw_params)

        # Add text (need to manually handle newlines)
        for i, text_line in enumerate(display_text.split('\n')):
            y = (i + 1) * 30
            cv2.putText(out, text_line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, 2)

        return out

    @staticmethod
    def _attitude_text(attitude: Rotation):
        """Generates the attitude text to be displayed on the visualization"""
        number_str_len = 7
        accuracy = 2
        gimbal_rpy_deg = RPY(*attitude.as_euler('XYZ', degrees=True))
        gimbal_rpy_text = f'roll: {str(round(gimbal_rpy_deg.roll, accuracy)).rjust(number_str_len)}, ' \
                          f'pitch: {str(round(gimbal_rpy_deg.pitch, accuracy)).rjust(number_str_len)}, ' \
                          f'yaw: {str(round(gimbal_rpy_deg.yaw, accuracy)).rjust(number_str_len)}.'

        return gimbal_rpy_text
