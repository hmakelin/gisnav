"""Module that contains an adapter for the LoFTR model."""
import os
import sys
import torch
import cv2
import numpy as np

from typing import Tuple
from enum import Enum
from python_px4_ros2_map_nav.assertions import assert_type
from python_px4_ros2_map_nav.keypoint_matchers.keypoint_matcher import KeypointMatcher
from LoFTR.loftr import LoFTR, default_cfg

from ament_index_python.packages import get_package_share_directory


class LoFTRMatcher(KeypointMatcher):
    """Adapter for LoFTR matcher"""

    # TODO: redundant implementation in superglue.py
    class TorchDevice(Enum):
        """Possible devices on which torch tensors are allocated."""
        CPU = 'cpu'
        CUDA = 'cuda'

    WEIGHTS_PATH = 'LoFTR/weights/outdoor_ds.ckpt'
    """Path to model weights - for LoFTR these need to be downloaded separately (see LoFTR README.md)"""

    CONFIDENCE_THRESHOLD = 0.7
    """Confidence threshold for filtering out bad matches"""

    def __init__(self, params: dict) -> None:
        """Initializes instance attributes

        This method is intended to be called inside :meth:`~initializer` together with a global variable declaration
        so that attributes initialized here are also available for :meth:`~worker`. This way we avoid having to declare
        a separate global variable for each attribute.

        :param params: LoFTR config parameters
        """
        self._device = LoFTRMatcher.TorchDevice.CUDA.value if torch.cuda.is_available() else \
            LoFTRMatcher.TorchDevice.CPU.value
        self._model = LoFTR(config=default_cfg)
        weights_path = os.path.join(get_package_share_directory('python_px4_ros2_map_nav'), self.WEIGHTS_PATH)  # TODO: provide as arg to constructor, do not hard-code path here
        self._model.load_state_dict(torch.load(weights_path)['state_dict'])
        self._model = self._model.eval().cuda()

    @property
    def _device(self) -> str:
        """Device on which torch tensors are allocated."""
        return self.__device

    @_device.setter
    def _device(self, value: str) -> None:
        assert_type(value, str)
        self.__device = value

    @property
    def _model(self) -> LoFTR:
        """Holds a :class:`loftr.LoFTR` instance for performing matching"""
        return self.__model

    @_model.setter
    def _model(self, value: LoFTR) -> None:
        assert_type(value, LoFTR)
        self.__model = value

    @staticmethod
    def initializer(params: dict) -> None:
        """Instantiates :class:`~LoFTRMatcher` as a global variable

        This makes the the neural network available to be called by :meth:`~worker` as long as it is called in the same
        process.

        :param params: LoFTR config to be passed to :class:`loftr.LoFTR`
        :return:
        """
        # noinspection PyGlobalUndefined
        global loftr
        loftr = LoFTRMatcher(params)

    @staticmethod
    def worker(img: np.ndarray, map_: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Returns matching keypoints between provided image and map

        Requires a global :class:`~LoFTRMatcher` instance to work.

        :param img: The image captured from drone camera
        :param map_: The map raster of the same area (from e.g. WMS endpoint)
        :return: Tuple of two lists containing matching keypoints in img and map respectively
        """
        try:
            assert_type(loftr, LoFTRMatcher)
            return loftr._match(img, map_)
        except Exception as e:
            raise e  # TODO: need to do anything here or just pass it on?

    def _match(self, img: np.ndarray, map_: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """Uses LoFTR to find matching keypoints between provided image and map

        :param img: The image captured from drone camera
        :param map_: The map raster of the same area (retrieved e.g. from a WMS endpoint)
        :return: Tuple of two numpy arrays containing matched keypoint coordinates in img and map respectively
        """
        img_grayscale = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        map_grayscale = cv2.cvtColor(map_, cv2.COLOR_BGR2GRAY)
        img_tensor = torch.from_numpy(img_grayscale)[None][None].cuda() / 255.
        map_tensor = torch.from_numpy(map_grayscale)[None][None].cuda() / 255.

        batch = {'image0': img_tensor, 'image1': map_tensor}
        
        with torch.no_grad():
            self._model(batch)
            mkp_img = batch['mkpts0_f'].cpu().numpy()
            mkp_map = batch['mkpts1_f'].cpu().numpy()
            conf = batch['mconf'].cpu().numpy()

        valid = conf > self.CONFIDENCE_THRESHOLD
        mkp_img = mkp_img[valid, :]
        mkp_map = mkp_map[valid, :]

        return mkp_img, mkp_map
