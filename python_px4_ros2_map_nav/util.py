"""Module containing utility functions for map_nav_node."""
import cv2
import numpy as np

from typing import Union, get_args
from collections import namedtuple

from python_px4_ros2_map_nav.assertions import assert_type, assert_ndim, assert_shape

BBox = namedtuple('BBox', 'left bottom right top')  # Convention: https://wiki.openstreetmap.org/wiki/Bounding_Box
LatLon = namedtuple('LatLon', 'lat lon')
LatLonAlt = namedtuple('LatLonAlt', 'lat lon alt')
Dim = namedtuple('Dim', 'height width')
RPY = namedtuple('RPY', 'roll pitch yaw')
TimePair = namedtuple('TimePair', 'local foreign')


class ImageFrame(object):
    """Keeps image frame related data in one place and protects it from corruption."""

    def __init__(self, image: np.ndarray, frame_id: str, timestamp: int) -> None:
        """Initializer

        :param image: Image array
        :param frame_id: Frame id
        :param timestamp: Image timestamp (estimated EKF2 system time from when image was taken)
        """
        assert_type(image, np.ndarray)
        assert_ndim(image, 3)
        assert_type(frame_id, str)
        assert_type(timestamp, int)
        self._image = image
        self._frame_id = frame_id
        self._timestamp = timestamp

        # fov and position expected to be set only once and some time after instantiation, setter methods provided
        self._fov = None
        self._position = None

    @property
    def image(self) -> np.ndarray:
        """Image raster."""
        return self._image

    @property
    def frame_id(self) -> str:
        """ROS 2 frame_id."""
        return self._frame_id

    @property
    def timestamp(self) -> int:
        """EKF2 timestamp."""
        return self._timestamp

    @property
    def fov(self) -> np.ndarray:
        """WGS84 coordinates of image corners (Field of View)."""
        return self._fov

    @fov.setter
    def fov(self, value: np.ndarray) -> None:
        if self._fov is not None:
            raise AttributeError("Modification of property not allowed.")
        assert_type(value, np.ndarray)
        assert_shape(value, (4, 1, 2))
        self._fov = value

    @property
    def position(self) -> LatLonAlt:
        """WGS84 coordinates of position of camera that took the image."""
        return self._position

    @position.setter
    def position(self, value: LatLonAlt) -> None:
        if self._position is not None:
            raise AttributeError("Modification of property not allowed.")
        assert_type(value, LatLonAlt)
        self._position = value


class MapFrame(object):
    """Keeps map frame related data in one place and protects it from corruption."""

    def __init__(self, center: Union[LatLon, LatLonAlt], radius: Union[int, float], bbox: BBox, image: np.ndarray) \
            -> None:
        """Initializer

        :param center: WGS84 coordinates of map center
        :param radius: Radius in meters of circle enclosed by the bounding box
        :param bbox: The bounding box of the map
        :param image: Map raster
        """
        assert_type(bbox, BBox)
        assert_type(center, get_args(Union[LatLon, LatLonAlt]))
        assert_type(radius, get_args(Union[int, float]))
        assert_type(image, np.ndarray)
        assert_ndim(image, 3)
        self._bbox = bbox
        self._center = LatLon(center.lat, center.lon)
        self._radius = radius
        self._image = image

    @property
    def center(self) -> LatLon:
        """WGS84 coordinates of map center."""
        return self._center

    @property
    def radius(self) -> int:
        """Radius of circle in meters enclosed by the map."""
        return self._radius

    @property
    def bbox(self) -> BBox:
        """WGS84 coordinates of map bounding box."""
        return self._bbox

    @property
    def image(self) -> np.ndarray:
        """Map image raster."""
        return self._image


def _make_keypoint(pt: np.ndarray, sz: float = 1.0) -> cv2.KeyPoint:
    """Converts input numpy array to a cv2.KeyPoint.

    :param pt: Keypoint x and y coordinates
    :param sz: Keypoint size
    :return:
    """
    assert_type(pt, np.ndarray)
    assert_shape(pt, (2,))
    return cv2.KeyPoint(pt[0], pt[1], sz)


def visualize_homography(figure_name: str, display_text: str, img_arr: np.ndarray,
                         map_arr: np.ndarray, kp_img: np.ndarray, kp_map: np.ndarray, dst_corners: np.ndarray) \
        -> np.ndarray:
    """Visualizes a homography including keypoint matches and field of view.

    :param figure_name: Display name of visualization
    :param display_text: Display text on top left of image
    :param img_arr: Image array
    :param map_arr: Map array
    :param kp_img: Image keypoints
    :param kp_map: Map keypoints
    :param dst_corners: Field of view corner pixel coordinates on map
    :return: Visualized image as numpy array
    """
    # Make a list of cv2.DMatches that match mkp_img and mkp_map one-to-one
    kp_count = len(kp_img)
    assert kp_count == len(kp_map), 'Keypoint counts for img and map did not match.'
    matches = list(map(lambda i: cv2.DMatch(i, i, 0), range(0, kp_count)))

    # Need cv2.KeyPoints for keypoints
    kp_img = np.apply_along_axis(_make_keypoint, 1, kp_img)
    kp_map = np.apply_along_axis(_make_keypoint, 1, kp_map)

    map_with_fov = cv2.polylines(map_arr, [np.int32(dst_corners)], True, 255, 3, cv2.LINE_AA)
    draw_params = dict(matchColor=(0, 255, 0), singlePointColor=None, matchesMask=None, flags=2)
    out = cv2.drawMatches(img_arr, kp_img, map_with_fov, kp_map, matches, None, **draw_params)

    # Add text (need to manually handle newlines)
    for i, text_line in enumerate(display_text.split('\n')):
        y = (i + 1) * 30
        cv2.putText(out, text_line, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, 2)

    cv2.imshow(figure_name, out)
    cv2.waitKey(1)

    return out
