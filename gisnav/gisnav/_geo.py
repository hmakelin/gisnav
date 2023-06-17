"""Module containing classes that wrap :class:`geopandas.GeoSeries` for convenience"""
from __future__ import annotations

import warnings
from abc import ABC
from typing import Tuple, TypeVar

import numpy as np
from geopandas import GeoSeries
from sensor_msgs.msg import CameraInfo
from shapely.geometry import Point, Polygon, box

from ._assertions import assert_len, assert_type

warnings.filterwarnings(
    action="ignore", category=UserWarning, message="Geometry is in a geographic CRS."
)


class _GeoObject(ABC):
    """Abstract base class for other GeoSeries wrappers

    Each instance wraps a :class:`geopandas.GeoSeries` instance of length one,
    i.e. containing a single Shapely shape. The GeoSeries class is versatile
    and operates on an entire series, so this wrapper is provided to only expose
    specific functionality that is needed in the application as well as provide
    an abstraction for a 'GeoObject' i.e. a GeoSeries of length 1. For example,
    it is conceptually more convenient to handle a 'GeoPt' object than a
    GeoSeries of length 1 with a single Shapely Point in it.
    """

    DEFAULT_CRS = "epsg:4326"
    """Use WGS 84 latitude and longitude by default"""

    G = TypeVar("G", bound="_GeoObject")

    @property
    def _geoseries(self) -> GeoSeries:
        return self.__geoseries

    @_geoseries.setter
    def _geoseries(self, value: GeoSeries) -> None:
        self.__geoseries = value

    @property
    def crs(self) -> str:
        """Returns current CRS string

        .. note::
            The underlying :class:`pyproj.crs.CRS` is converted to a (lower case)
            string
        """
        return self._geoseries.crs.srs.lower()

    @property
    def coords(self) -> np.ndarray:
        """Returns the wrapped shape as a numpy array"""
        return self._geoseries[0].coords[0]

    def to_crs(self: G, crs: str) -> G:  # TODO: return None? Misleading this way
        """Converts to provided CRS

        :return: The same _GeoObject inheriting instance transformed to new CRS
        """
        if self._geoseries.crs != crs:
            self._geoseries = self._geoseries.to_crs(crs)
        return self

    def to_file(self, filename: str, driver: str = "GeoJSON") -> None:
        """Saves the wrapped shape into a file (e.g. as GeoJSON)

        :param filename: File name
        :param driver: OGR format driver
        """
        self._geoseries.to_file(filename, driver=driver)

    def __post_init__(self):
        """Post-initialization validity checks"""
        # TODO enforce, do not assert
        assert_len(self._geoseries, 1)
        assert self._geoseries.crs is not None


class _GeoPolygon(_GeoObject):
    """Abstract base class for :class:`_GeoObject`s that contain a Shapely Polygon"""

    @property
    def center(self) -> GeoPt:
        """Returns center point of the polygon"""
        return GeoPt(
            self._geoseries.centroid[0].coords[0][0],
            self._geoseries.centroid[0].coords[0][1],
            crs=self.crs,
        )

    @property
    def bounds(self) -> Tuple[float, float, float, float]:
        """Returns (left, bottom, right, top) or (minx, miny, maxx, maxy)
        formatted tuple (e.g. for WMS GetMap)"""
        return self._geoseries[0].bounds

    @property
    def area(self) -> float:
        """Returns area of the bounding box"""
        return self._geoseries.area[0]

    @property
    def length(self) -> float:
        """Returns length of polygon in native CRS"""
        return self._geoseries[0].length

    @property
    def meter_length(self) -> float:
        """Returns length of polygon in meters"""
        return (
            self.center.spherical_adjustment
            * self._geoseries.to_crs("epsg:3857")[0].length
        )

    @property
    def coords(self) -> np.ndarray:
        """Returns a numpy array of the coordinates of the polygon exterior
        ring in counter-clockwise order

        Coordinates provided in counter-clockwise order

        .. note::
            Shapely duplicates the Polygon starting point to close its boundary
            but this property removes the duplicate
        """
        assert_len(self._geoseries[0].exterior.coords, 5)
        exterior_coords = np.array(self._geoseries[0].exterior.coords)[:-1]
        assert_len(exterior_coords, 4)
        return exterior_coords

    def intersection(self, box_: _GeoPolygon) -> GeoTrapezoid:
        """Returns the intersection with the given :class:`._GeoPolygon`"""
        return GeoTrapezoid(self._geoseries.intersection(box_._geoseries))

    def __post_init__(self):
        """Post-initialization validity checks

        :raise: ValueError if wrapped polygon is not valid
        """
        assert_type(self._geoseries[0], Polygon)


class GeoPt(_GeoObject):
    """Wrapper for :class:`geopandas.GeoSeries` that constrains it to a 2D Point
    (a geographical coordinate pair)"""

    def __init__(self, x: float, y: float, crs: str = _GeoObject.DEFAULT_CRS):
        """Creates a geographical coordinate pair

        .. warning::
            X and Y axes are in ENU frame so longitude comes before latitude
            (easting before northing) even though WGS 84 is defined as latitude
            before longitude.

        :param x: ENU frame X axis coordinate (e.g. longitude in WGS 84)
        :param y: ENU frame Y axis coordinate (e.g. latitude in WGS 84)
        :param crs: Coordinate Reference System (CRS) string (e.g. 'epsg:4326'
            for WGS 84) for the coordinate pair
        """
        self._geoseries = GeoSeries([Point(x, y)], crs=crs)

    @property
    def latlon(self) -> Tuple[float, float]:
        """(latitude, longitude) tuple in WGS 84

        .. note:
            This returns latitude and longitude in WGS 84 format, i.e. (y, x)
            in ENU frame
        """
        return self.to_crs("epsg:4326").coords[1::-1]

    @property
    def lat(self) -> float:
        """Latitude in WGS 84"""
        return self.latlon[0]

    @property
    def lon(self) -> float:
        """Longitude in WGS 84"""
        return self.latlon[1]

    @property
    def spherical_adjustment(self):
        """Helper property for correcting distance measured in EPSG:3857
        pseudo-meters into approximate real meters

        Uses a simple spherical model which is accurate enough for expected use cases
        """
        return np.cos(np.radians(abs(self.lat)))


class GeoSquare(_GeoPolygon):
    """Wrapper for :class:`geopandas.GeoSeries` that constrains it to a
    (square shaped) bounding box"""

    def __init__(self, center: GeoPt, radius: float, crs: str = _GeoObject.DEFAULT_CRS):
        """Creates a square bounding box by enveloping a circle of given radius
        at given center

        :param center: Center of the bounding box
        :param radius: Radius of enclosed circle in meters
        :param crs: Coordinate Reference System (CRS) string (e.g. 'epsg:4326')
        """
        self._geoseries = (
            center.to_crs("epsg:3857")
            ._geoseries.buffer(radius / center.spherical_adjustment)
            .to_crs(crs)
            .envelope
        )
        assert_type(self._geoseries[0], Polygon)

    @property
    def coords(self) -> np.ndarray:
        """Returns a numpy array of the corners coordinates of the bbox

        Order should be top-left, bottom-left, bottom-right, top-right (same as
        :meth:`gisnav.transform.create_src_corners`).
        """
        corners = box(*self.bounds).exterior.coords
        corners = np.array(
            [corners[2], corners[3], corners[0], corners[1]]  # tl  # bl  # br  # tr
        )

        return corners


class GeoTrapezoid(_GeoPolygon):
    """Wrapper for :class:`geopandas.GeoSeries` that constrains it to a
    (convex isosceles) trapezoid

    Used to represent camera field of view projected to ground plane.
    """

    def __init__(self, corners: np.ndarray, crs: str = _GeoObject.DEFAULT_CRS):
        """Creates a square bounding box by enveloping a circle of given radius inside

        :param corners: Trapezoid corner coordinates
        :param crs: Coordinate Reference System (CRS) string (e.g. 'epsg:4326')
        :raise: :class:`.GeoValueError` if the trapezoid is not valid (convex
            isosceles trapezoid)
        """
        self._geoseries = GeoSeries([Polygon(corners.squeeze())], crs=crs)

    def __post_init__(self):
        """Post-initialization validity checks"""
        if not (self._geoseries[0].is_valid and not self._is_convex()):
            raise GeoValueError(f"Not a valid convex trapezoid: {self._geoseries[0]}")

    def _is_convex(self) -> bool:
        """Returns True if the geoseries is convex

        :return: True if the geoseries is convex
        """
        return self._geoseries[0].bounds == self._geoseries[0].convex_hull.bounds

    @property
    def square_coords(self) -> np.ndarray:
        """Returns a numpy array of the corners coordinates of the bbox

        Order should be top-left, bottom-left, bottom-right, top-right (same as
        :meth:`gisnav.data.create_src_corners`).
        """
        corners = box(*self.bounds).exterior.coords
        corners = np.array(
            [corners[2], corners[3], corners[0], corners[1]]  # tl  # bl  # br  # tr
        )

        return corners


def get_dynamic_map_radius(
    camera_info: CameraInfo, max_map_radius: int, altitude: float
) -> float:
    """Returns map radius that adjusts for camera altitude to be used for new
    map requests

    :param camera_data: Camera data
    :param max_map_radius: Max map radius
    :param altitude: Altitude of camera in meters
    :return: Suitable map radius in meters
    """
    hfov = 2 * np.arctan(camera_info.width / (2 * camera_info.k[0]))
    map_radius = 1.5 * hfov * altitude  # Arbitrary padding of 50%
    return min(map_radius, max_map_radius)


class GeoValueError(ValueError):
    """Exception returned if a valid GeoObject could not be initialized from
    provided values."""
