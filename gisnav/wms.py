"""Contains a WMS client for asynchronously requesting map rasters from an external endpoint"""
from __future__ import annotations

import numpy as np
import cv2

from typing import Tuple, List, Optional
from owslib.wms import WebMapService
from owslib.util import ServiceException

from gisnav.assertions import assert_type, assert_ndim
from gisnav.data import MapData, Img


class WMSClient:
    """A WMS client for asynchronously requesting map rasters

    This class defines static initializer and worker methods that are intended to be used with the
    :class:`multiprocessing.pool.Pool` and :class:`multiprocessing.pool.ThreadPool` interfaces.

    .. note::
        You probably should not need to instantiate :class:`.WMSClient` directly from outside this class. Use the
        :meth:`.initializer` to create your client instead, and the :meth:`.worker` to use it.

    .. warning::
        ``OWSLib`` *as of version 0.25.0* uses the Python ``requests`` library under the hood but does not seem to
        document the various exceptions it raises that are passed through by ``OWSLib`` as part of its public API.
        The :meth:`.worker` method is therefore expected to raise `errors and exceptions
        <https://requests.readthedocs.io/en/latest/user/quickstart/#errors-and-exceptions>`_ that are specific to the
        ``requests`` library.

        These errors and exceptions are not handled by the :class:`.WMSClient` to avoid a direct dependency to
        ``requests``. The recommended approach is therefore to handle them as unexpected errors in the error callback
        of the invoking :meth:`multiprocessing.pool.Pool.apply_async`,
        :meth:`multiprocessing.pool.ThreadPool.apply_async`, or similar method.
    """

    WMS_CLIENT_GLOBAL_VAR = '_wms_client'
    """A global :class:`.WMSClient` instance is stored under this name"""

    _IMAGE_TRANSPARENCY = False
    """Image background transparency (not supported by jpeg format)"""

    def __init__(self, url: str, version: str, timeout: int):
        """Initializes instance attributes

        :param url: WMS endpoint url
        :param version: WMS version (e.g. '1.1.1')
        :param timeout: WMS request timeout in seconds
        """
        # Do not handle possible connection related exceptions here (see class docstring)
        self._wms = WebMapService(url, version=version, timeout=timeout)

    @staticmethod
    def initializer(url: str, version_: str, timeout_: int) -> None:
        """Creates a global instance of :class:`.WMSClient` under :py:attr:`.WMS_CLIENT_GLOBAL_VAR`.

        :param url: WMS server endpoint url
        :param version_: WMS server version
        :param timeout_: WMS request timeout seconds
        """
        if WMSClient.WMS_CLIENT_GLOBAL_VAR not in globals():
            globals()[WMSClient.WMS_CLIENT_GLOBAL_VAR] = WMSClient(url, version_, timeout_)

    @staticmethod
    def worker(layers: List[str], styles: List[str], bbox: Tuple[float], map_size: Tuple[int, int], srs_str: str,
               image_format: str, elevation_layers: Optional[List[str]] = None) \
            -> Tuple[np.ndarray, Optional[np.ndarray]]:
        """Requests one or more map imagery and optional height raster layers from the WMS server
        
        TODO: Currently no support for separate arguments for imagery and height layers. Assumes height layer is 
        available at same styles and CRS as imagery layer.

        :param layers: List of requested map layers
        :param styles: Optional styles of same length as layers, use empty strings for default styles
        :param bbox: Bounding box of the map as tuple (left, bottom, right, top)
        :param map_size: Map size tuple (height, width)
        :param srs_str: WMS server SRS
        :param image_format: WMS server requested image format
        :param elevation_layers: Optional layers for height raster
        :return: Tuple of imagery and height (Optional) rasters
        """
        assert WMSClient.WMS_CLIENT_GLOBAL_VAR in globals()
        wms_client: WMSClient = globals()[WMSClient.WMS_CLIENT_GLOBAL_VAR]

        # Do not handle possible requests library related exceptions here (see class docstring)
        try:
            map_ = wms_client._wms.getmap(layers=layers, styles=styles, srs=srs_str, bbox=bbox, size=map_size,
                                          format=image_format, transparent=WMSClient._IMAGE_TRANSPARENCY)
        except ServiceException as _:
            # TODO: handle OWSLib exceptions - currently passed on to error callback
            raise

        map_ = np.frombuffer(map_.read(), np.uint8)
        map_ = cv2.imdecode(map_, cv2.IMREAD_UNCHANGED)
        assert_type(map_, np.ndarray)
        assert_ndim(map_, 3)

        elevation = None
        if elevation_layers is not None:
            try:
                elevation = wms_client._wms.getmap(layers=elevation_layers, styles=styles, srs=srs_str, bbox=bbox,
                                                   size=map_size, format=image_format,
                                                   transparent=False)
            except ServiceException as _:
                # TODO: handle exception for height layer
                pass

        if elevation is not None:
            elevation = np.frombuffer(elevation.read(), np.uint8)  # TODO: handle values above 255?
            elevation = cv2.imdecode(elevation, cv2.IMREAD_GRAYSCALE)
            assert_type(elevation, np.ndarray)
            assert_ndim(elevation, 2)  # Grayscale, no color channels

        return map_, elevation
