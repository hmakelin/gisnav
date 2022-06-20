"""Module containing classes that wrap :class:`geopandas.GeoSeries` for convenience"""
from __future__ import annotations

import numpy as np

from abc import ABC, abstractmethod
from functools import lru_cache
from geopandas import GeoSeries
from shapely.geometry import Point, Polygon, box

from python_px4_ros2_map_nav.assertions import assert_len, assert_type


class _GeoObject(ABC):
    """Abstract base class for other GeoSeries wrappers

    Each _GeoObject wraps a :class:`geopandas.GeoSeries` of length one, i.e. they only contain a single Shapely shape.

    The :class:`_GeoObject` methods expose :class:`geopandas.GeoSeries` methods to
    """

    DEFAULT_CRS = 'epsg:4326'
    """Use WGS 84 latitude and longitude by default"""

    @property
    @abstractmethod
    def _geoseries(self) -> np.ndarray:
        """Returns the contained :class:`geopandas.GeoSeries` instance"""
        pass

    @property
    def crs(self) -> str:
        """Returns current CRS string"""
        return str(self._geoseries.crs)

    @property
    @abstractmethod
    def coordinates(self) -> np.ndarray:
        """Returns the wrapped shape as a numpy array."""
        pass

    def to_crs(self, crs: str) -> _GeoObject:
        """Converts to provided CRS

        :return: The same GeoPoint instance transformed to new CRS
        """
        if self._geoseries.crs != crs:
            self._geoseries = self._geoseries.to_crs(crs)
        return self

    def to_file(self, filename: str, driver: str = 'GeoJSON') -> None:
        """Converts the wrapped GeoSeries to GeoJSON

        :param filename: File name
        :param driver: OGR format driver
        """
        self._geoseries.to_file(filename, driver=driver)

class _GeoPolygon(_GeoObject):
    """Abstract base class for other wrappers that contain GeoSeries with Shapely Polygons"""

    @property
    def center(self) -> GeoPoint:
        """Returns center or centroid point of the polygon"""
        return GeoPoint(*self._geoseries.centroid[0].coords[0], crs=self.crs)

    # TODO: make return numpy array
    @property
    @lru_cache(4)
    def bounds(self) -> Tuple[4*(float,)]:
        """Returns (left, bottom, right, top) or (minx, miny, maxx, maxy) formatted tuple for WMS GetMap requests"""
        return self._geoseries[0].bounds

    def __post_init__(self):
        """Post-initialization validity checks"""
        # TODO: Enforce validity checks instead of asserting
        #assert_len(self._geoseries[0].exterior.coords, 4 + 1)  TODO 4 or 5?
        assert_len(self._geoseries, 1)
        assert_type(self._geoseries[0], Polygon)
        assert self._geoseries.crs is not None
        #assert self._geoseries[0].is_valid  # TODO: handle this like is_isosceles_trapezoid (this is pre-init check, the other one is post init check)

class GeoPoint(_GeoObject):
    """Wrapper for :class:`geopandas.GeoSeries` that constrains it to a 2D Point (geographical coordinate pair)

    The GeoSeries class is very flexible, so this wrapper is provided to only expose specific functionality that is
    needed in the application. It is also more convenient to handle a 'Point' conceptually than a series of length 1
    with a single Point in it.

    Pay attention to the axis order, i.e. (x, y) is (lon, lat) for EPSG:4326.
    """
    # TODO call these x and y instead of x and y (more generic)
    def __init__(self, x: float, y: float, crs: str = _GeoObject.DEFAULT_CRS):
        """Initializes the wrapped GeoSeries

        Not that (x and y) are in ENU frame so easting comes before northing (e.g. lon before lat, even though WGS 84
        is defined as lat before lon).

        :param x: X axis coordinate (longitude)
        :param y: Y axis coordinate (latitude)
        :param crs: Coordinate Reference System (CRS) string (e.g. 'epsg:4326') the (x, y) pair is provided in
        """
        self._geoseries = GeoSeries([Point(x, y)], crs=crs)
        assert_type(self._geoseries[0], Point)

        # TODO: Enforce validity checks instead of asserting
        assert_len(self._geoseries, 1)
        assert_type(self._geoseries[0], Point)
        #assert self._geoseries.crs is not None  # TODO: disabled because fov_pix is handled as GeoTrapezoid, fix it

    # TODO: return numpy array, same as GeoBBox, maybe refactor these both into _GeoObject?
    @property
    @lru_cache(4)
    def coordinates(self) -> Tuple[float, float]:  # TODO: return as numpy array
        """X/Y (ENU frame) tuple in given CRS system units

        Note that this only returns coordinates in the provided CRS units but always in the (x, y) axis
        order, so e.g. WGS84 (lat, lon) pair would be returned as (lon, lat). Use :meth:`~latlon` to get WGS84
        coordinates in the correct order.

        :return: X/Y (e.g. lon/lat because of ENU frame) tuple
        """
        return self._geoseries[0].coords[0]

    @property
    def latlon(self) -> Tuple[float, float]:
        """Convenience property to get lat/lon tuple in WGS 84

        Note that this returns latitude and longitude in different order then :meth:`~get_coordinates`
        """
        return self.to_crs('epsg:4326').coordinates[1::-1]

    @property
    def lat(self) -> float:
        """Convenience property to get latitude in WGS 84"""
        return self.latlon[0]

    @property
    def lon(self) -> float:
        """Convenience property to get longitude in WGS 84"""
        return self.latlon[1]

    def _spherical_adjustment(self):
        """Helper method to adjust distance measured in EPSG:3857 pseudo-meters into approximate real meters

        Uses a simple spherical model which is accurate enough for planned use scenarios
        """
        # TODO: use a precise conversion?
        return abs(np.cos(np.radians(self.lat)))


class GeoBBox(_GeoPolygon):
    """Wrapper for :class:`geopandas.GeoSeries` that constrains it to a bounding box

    Used for (square) map bounding boxes.
    """
    # TODO: have constructor make generic rectangle, and a static method to make square from circle
    def __init__(self, center: GeoPoint, radius: float, crs: str = _GeoObject.DEFAULT_CRS):
        """Creates a square bounding box with a circle of given radius inside

        :param center: Center of the bounding box
        :param radius: Radius of enclosed circle in meters
        :param crs: Coordinate Reference System (CRS) string (e.g. 'epsg:4326')
        """
        # TODO: spherical adjustment uses self.center property, which needs _geoseries defined! so what to do here?
        self._geoseries = center.to_crs('epsg:3857')._geoseries.buffer((1/center._spherical_adjustment()) * radius)\
            .to_crs(crs).envelope
        assert_type(self._geoseries[0], Polygon)

    @property
    @lru_cache(4)
    def coordinates(self) -> np.ndarray:
        """Returns a numpy array of the corners coordinates of the bbox

        Order should be top-left, bottom-left, bottom-right, top-right (same as
        :meth:`python_px4_ros2_map_nav.transform.create_src_corners`).
        """
        # TODO: fix this, hard coded order is prone to breaking even when using box function
        # TODO: why sometimes 5, sometimes 4?
        if len(self._geoseries[0].exterior.coords) == 5:
            corners = box(*self.bounds).exterior.coords[:-1]
        else:
            len(self._geoseries[0].exterior.coords) == 4
            corners = box(*self.bounds).exterior.coords
        corners = np.array([
            corners[2],  # tl
            corners[3],  # bl
            corners[0],  # br
            corners[1]   # tr
        ])
        # (lon, lat) to (lat, lon)
        corners = np.flip(corners, axis=1).reshape(-1, 1, 2)

        return corners

    @property
    def area(self) -> float:
        """Returns area of the box"""
        return self._geoseries.area[0]

    def intersection_area(self, box: GeoBBox):
        """Returns area of the intersection between the two boxes"""
        return self._geoseries.intersection(box._geoseries).area[0]  # TODO: access private attr


class GeoTrapezoid(_GeoPolygon):
    """Wrapper for :class:`geopandas.GeoSeries` that constrains it to a (convex) trapezoid

    Used to represents camera field of view projected to ground plane.
    """
    def __init__(self, corners: np.ndarray, crs: str = _GeoObject.DEFAULT_CRS):
        """Creates a square bounding box with a circle of given radius inside

        :param corners: Trapezoid corner coordinates
        :param crs: Coordinate Reference System (CRS) string (e.g. 'epsg:4326')
        """
        self._geoseries = GeoSeries([Polygon(corners.squeeze())], crs=crs)

    # TODO: how to know which corners are "bottom" corners and which ones are "top" corners?
    #  Order should again be same as in create_src_corners, as in GeoBBox. Need to have some shared trapezoid corner order constant used by these three?
    # Need to force constructor to distinguish between tl, bl, br, and tr?
    @property
    @lru_cache(4)
    def coordinates(self) -> np.ndarray:
        """Returns a numpy array of the corners coordinates of the trapezoid

        Order should be top-left, bottom-left, bottom-right, top-right (same as
        :meth:`python_px4_ros2_map_nav.transform.create_src_corners`).
        """
        # Corners should be provided in tl, bl, br, tr order to constructor
        # TODO: make this less prone to breaking
        # TODO: why sometimes 5, sometimes 4?
        if len(self._geoseries[0].exterior.coords) == 5:
            corners = np.array(self._geoseries[0].exterior.coords[:-1])
        else:
            assert len(self._geoseries[0].exterior.coords) == 4
            corners = np.array(self._geoseries[0].exterior.coords)

        # (lon, lat) to (lat, lon)
        corners = np.flip(corners, axis=1).reshape(-1, 1, 2)

        return corners

    @property
    def length(self) -> float:
        # TODO: does not use crs, just returns raw lenght because fov_pix does not have crs
        """Returns length of polygon"""
        return self._geoseries[0].length

    @property
    def meter_length(self) -> float:
        """Returns length of polygon in meters"""
        return self.center._spherical_adjustment() * self._geoseries.to_crs('epsg:3857')[0].length

