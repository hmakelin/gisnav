"""WMS client for requesting map rasters from external endpoint."""
import numpy as np
import cv2

from typing import Optional, Union, Tuple, List

from owslib.wms import WebMapService
from python_px4_ros2_map_nav.assertions import assert_type, assert_ndim
from python_px4_ros2_map_nav.data_classes import BBox, LatLon, LatLonAlt, MapData, get_args


class WMSClient:
    """WMS client for requesting map rasters from external endpoint

    This class defines static initializer and worker methods that are intended to be passed to a separate process in a
    :class:`multiprocessing.pool.Pool`.
    """

    REQUEST_DEFAULT_TIMEOUT = 30
    """Default WMS request timeout in seconds"""

    IMAGE_FORMAT = 'image/jpeg'
    """Requested image format"""

    IMAGE_TRANSPARENCY = False
    """Image background transparency (not supported by jpeg format)"""

    def __init__(self, url: str, version_: str, timeout_: int = REQUEST_DEFAULT_TIMEOUT):
        """Initializes instance attributes

        This method is intended to be called inside :meth:`~initializer` together with a global variable declaration
        so that attributes initialized here are also available for :meth:`~worker`. This way we avoid having to declare
        a separate global variable for each attribute.

        :param url: WMS endpoint url
        :param version_: WMS version (e.g. 1.1.1)
        :param timeout_: WMS request timeout in seconds
        """
        try:
            self._wms = WebMapService(url, version=version_, timeout=timeout_)
        except Exception as e:
            # TODO: handle exception (e.g. ConnectionRefusedError)
            raise e

    @property
    def _wms(self) -> WebMapService:
        """The WMS client instance accessed by :meth:`~initializer`"""
        return self.__wms

    @_wms.setter
    def _wms(self, value: WebMapService) -> None:
        #assert_type(value, WebMapService)  # TODO assert type
        self.__wms = value

    @staticmethod
    def initializer(url: str, version_: str, timeout_: int = REQUEST_DEFAULT_TIMEOUT) -> None:
        """Returns a cached WMS client.

        The WMS requests are intended to be handled in a dedicated process (to avoid blocking the main thread), so this
        function is lru_cache'd to avoid recurrent instantiations every time a WMS request is sent. For example usage, see
        :meth:`python_px4_ros2_map_nav.MapNavNode._wms_pool_worker` method.

        :param url: WMS server endpoint url
        :param version_: WMS server version
        :param timeout_: WMS request timeout seconds
        :return: The cached WMS client
        """
        assert_type(url, str)
        assert_type(version_, str)
        assert_type(timeout_, int)
        # noinspection PyGlobalUndefined
        global wms_client
        wms_client = WMSClient(url, version_, timeout_)

    @staticmethod
    def worker(center: Union[LatLon, LatLonAlt], radius: Union[int, float], bbox: BBox, map_size: Tuple[int, int],
               layer_str: str, srs_str: str) -> MapData:
        """Gets latest map from WMS server for given location, then creates a :class:`~data_classes.MapData` and returns it

        :param center: Center of the map to be retrieved
        :param radius: Radius in meters of the circle to be enclosed by the map
        :param bbox: Bounding box of the map
        :param map_size: Map size tuple (height, width)
        :param layer_str: WMS server layer
        :param srs_str: WMS server SRS
        :return: ~data_classes.MapData containing the map raster and supporting metadata
        """
        assert wms_client is not None
        assert_type(bbox, BBox)
        assert (all(isinstance(x, int) for x in map_size))
        assert_type(layer_str, str)
        assert_type(srs_str, str)
        assert_type(center, get_args(Union[LatLonAlt, LatLon]))
        assert_type(radius, get_args(Union[int, float]))
        map_ = wms_client._get_map(layer_str, srs_str, bbox, map_size, WMSClient.IMAGE_FORMAT,
                                   WMSClient.IMAGE_TRANSPARENCY)
        map_data = MapData(center=center, radius=radius, bbox=bbox, image=map_)
        return map_data

    def _get_map(self, layer_str: str, srs_str: str, bbox_: BBox, size_: Tuple[int, int], format_: str,
                 transparent_: bool) -> np.ndarray:
        """Performs a WMS GetMap request with the supplied keyword arguments

        :param layer_str: WMS server layer
        :param srs_str: WMS server SRS
        :param bbox_: Bounding box of the map
        :param size_: Map size tuple (height, width)
        :param format_: Requested image format
        :param transparent_: Requested image background transparency
        :return: Map raster
        """
        try:
            map_ = self._wms.getmap(layers=[layer_str], srs=srs_str, bbox=bbox_, size=size_, format=format_,
                                    transparent=transparent_)
        except Exception as e:
            # TODO: handle exception
            raise e

        map_ = np.frombuffer(map_.read(), np.uint8)
        map_ = cv2.imdecode(map_, cv2.IMREAD_UNCHANGED)
        assert_type(map_, np.ndarray)
        assert_ndim(map_, 3)
        return map_
