import cv2
import numpy as np

from scipy.spatial.transform import Rotation
from python_px4_ros2_map_nav.data import OutputData, RPY, ImagePair
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

    def update(self, output_data: OutputData, vo_enabled: bool) -> None:
        """Updates the visualization

        TODO: Remove vo_enabled parameter. This parameter is used to get around problem of cv2.imshow hanging when
        trying to update both map and visual odometry visualizations. In this case visualization is only updated
        when visual_odometry==True

        :param output_data: Data to update the visualization with
        ####:param visual_odometry: True to update visual odometry visualization, False for map visualization
        :param vo_enabled: Flag indicating whether visual odometry is enabled
        :return:
        """
        img = self._create_visualization(output_data, self._attitude_text(output_data.attitude))
        img = img.astype(np.uint8)

        # TODO: check that image shapes do not change?
        if not output_data._pose.image_pair.mapful():
            self._vo_visualization = img
            if self._map_visualization is None:
                self._map_visualization = np.zeros(img.shape, dtype=np.uint8)
        else:
            self._map_visualization = img
            if self._vo_visualization is None:
                self._vo_visualization = np.zeros(img.shape, dtype=np.uint8)

        out = np.vstack((self._map_visualization, self._vo_visualization, output_data._pose.image_pair.img.image))
        if vo_enabled:  # TODO: hack to make visualization work - try to get rid of this conditional
            if output_data._pose.image_pair.mapful():
                cv2.imshow(self.name, out)
                cv2.waitKey(1)
        else:
            cv2.imshow(self.name, out)
            cv2.waitKey(1)

    # TODO: optionally return mkp's from worker and pass them onto this function?
    @staticmethod
    def _create_visualization(output_data: OutputData, display_text: str) -> np.ndarray:
        """Visualizes a homography including keypoint matches and field of view.

        :param output_data: OutputData from the matching
        :param display_text: Text to display on visualization
        :return: Visualized image as numpy array
        """
        # Make a list of cv2.DMatches that match mkp_img and mkp_map one-to-one
        #kp_count = len(output_data.mkp_img)
        #assert kp_count == len(output_data.mkp_map), 'Keypoint counts for img and map did not match.'
        #matches = list(map(lambda i_: cv2.DMatch(i_, i_, 0), range(0, kp_count)))

        # Need cv2.KeyPoints for keypoints
        #mkp_img = np.apply_along_axis(make_keypoint, 1, output_data.mkp_img)
        #mkp_map = np.apply_along_axis(make_keypoint, 1, output_data.mkp_map)

        ref_img = output_data._pose.image_pair.ref.image
        map_with_fov = cv2.polylines(ref_img.copy(), [np.int32(output_data.fov_pix)], True, 255, 3, cv2.LINE_AA)
        #draw_params = dict(matchColor=(0, 255, 0), singlePointColor=None, matchesMask=None, flags=2)
        #out = cv2.drawMatches(output_data.input.image_data.image, mkp_img, map_with_fov, mkp_map, matches, None,
        #                      **draw_params)

        # Add text (need to manually handle newlines)
        #for i, text_line in enumerate(display_text.split('\n')):
        #    y = (i + 1) * 30
        #    cv2.putText(out, text_line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, 2)

        #return out
        return map_with_fov

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
