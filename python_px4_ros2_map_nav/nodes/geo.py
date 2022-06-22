"""Module containing classes that wrap :class:`geopandas.GeoSeries` for convenience"""
from __future__ import annotations

import numpy as np

from abc import ABC, abstractmethod
from geopandas import GeoSeries
from shapely.geometry import Point, Polygon, box
from shapely.geometry.polygon import orient

from python_px4_ros2_map_nav.assertions import assert_len, assert_type


class _GeoObject(ABC):
    """Abstract base class for other GeoSeries wrappers

    Each instance wraps a :class:`geopandas.GeoSeries` instance of length one, i.e. containing a single Shapely shape.
    The GeoSeries class is versatile and operates on an entire series, so this wrapper is provided to only expose
    specific functionality that is needed in the application as well as provide an abstraction for a 'GeoObject' i.e. a
    GeoSeries of length 1. For example, it is conceptually more convenient to handle a 'GeoPoint' object than a
    GeoSeries of length 1 with a single Shapely Point in it.
    """

    DEFAULT_CRS = 'epsg:4326'
    """Use WGS 84 latitude and longitude by default"""

    @property
    def crs(self) -> str:
        """Returns current CRS string

        .. note::
            The underlying :class:`pyproj.crs.CRS` is converted to a (lower case) string
        """
        return self._geoseries.crs.srs.lower()

    @property
    def coords(self) -> np.ndarray:
        """Returns the wrapped shape as a numpy array"""
        return self._geoseries[0].coords[0]

    def to_crs(self, crs: str) -> _GeoObject:  # TODO: return None? Misleading this way
        """Converts to provided CRS

        :return: The same GeoPoint instance transformed to new CRS
        """
        if self._geoseries.crs != crs:
            self._geoseries = self._geoseries.to_crs(crs)
        return self

    def to_file(self, filename: str, driver: str = 'GeoJSON') -> None:
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
    def center(self) -> GeoPoint:
        """Returns center point of the polygon"""
        return GeoPoint(*self._geoseries.centroid[0].coords[0], crs=self.crs)

    @property
    def bounds(self) -> Tuple[4*(float,)]:
        """Returns (left, bottom, right, top) or (minx, miny, maxx, maxy) formatted tuple (e.g. for WMS GetMap)"""
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
        return self.center.spherical_adjustment * self._geoseries.to_crs('epsg:3857')[0].length

    @property
    def coords(self) -> np.ndarray:
        """Returns a numpy array of the coordinates of the polygon exterior ring in counter-clockwise order

        Coordinates provided in counter-clockwise order

        .. note::
            Shapely duplicates the Polygon starting point to close its boundary but this property removes the duplicate
        """
        assert_len(self._geoseries[0].exterior.coords, 5)
        exterior_coords = np.array(self._geoseries[0].exterior.coords)[:-1]
        assert_len(exterior_coords, 4)
        return exterior_coords

    def intersection(self, box: _GeoPolygon) -> GeoTrapezoid:
        """Returns the intersection with the given :class:`._GeoPolygon`"""
        return GeoTrapezoid(self._geoseries.intersection(box._geoseries))

    def __post_init__(self):
        """Post-initialization validity checks

        :raise: ValueError if wrapped polygon is not valid
        """
        assert_type(self._geoseries[0], Polygon)


class GeoPoint(_GeoObject):
    """Wrapper for :class:`geopandas.GeoSeries` that constrains it to a 2D Point (a geographical coordinate pair)"""
    def __init__(self, x: float, y: float, crs: str = _GeoObject.DEFAULT_CRS):
        """Creates a geographical coordinate pair

        .. warning::
            X and Y axes are in ENU frame so longitude comes before latitude (easting before northing) even though
            WGS 84 is defined as latitude before longitude.

        :param x: ENU frame X axis coordinate (e.g. longitude in WGS 84)
        :param y: ENU frame Y axis coordinate (e.g. latitude in WGS 84)
        :param crs: Coordinate Reference System (CRS) string (e.g. 'epsg:4326' for WGS 84) for the coordinate pair
        """
        self._geoseries = GeoSeries([Point(x, y)], crs=crs)

    @property
    def latlon(self) -> Tuple[float, float]:
        """(latitude, longitude) tuple in WGS 84

        .. note:
            This returns latitude and longitude in WGS 84 format, i.e. (y, x) in ENU frame
        """
        return self.to_crs('epsg:4326').coords[1::-1]

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
        """Helper property for correcting distance measured in EPSG:3857 pseudo-meters into approximate real meters

        Uses a simple spherical model which is accurate enough for expected use cases
        """
        return np.cos(np.radians(abs(self.lat)))


class GeoSquare(_GeoPolygon):
    """Wrapper for :class:`geopandas.GeoSeries` that constrains it to a (square shaped) bounding box"""
    def __init__(self, center: GeoPoint, radius: float, crs: str = _GeoObject.DEFAULT_CRS):
        """Creates a square bounding box by enveloping a circle of given radius at given center

        :param center: Center of the bounding box
        :param radius: Radius of enclosed circle in meters
        :param crs: Coordinate Reference System (CRS) string (e.g. 'epsg:4326')
        """
        self._geoseries = center.to_crs('epsg:3857')._geoseries.buffer(radius / center.spherical_adjustment)\
            .to_crs(crs).envelope
        assert_type(self._geoseries[0], Polygon)

    @property
    def coords(self) -> np.ndarray:
        """Returns a numpy array of the corners coordinates of the bbox

        Order should be top-left, bottom-left, bottom-right, top-right (same as
        :meth:`python_px4_ros2_map_nav.transform.create_src_corners`).
        """
        corners = box(*self.bounds).exterior.coords
        corners = np.array([
            corners[2],  # tl
            corners[3],  # bl
            corners[0],  # br
            corners[1]   # tr
        ])

        return corners

class GeoTrapezoid(_GeoPolygon):
    """Wrapper for :class:`geopandas.GeoSeries` that constrains it to a (convex isosceles) trapezoid

    Used to represents camera field of view projected to ground plane.
    """
    def __init__(self, corners: np.ndarray, crs: str = _GeoObject.DEFAULT_CRS):
        """Creates a square bounding box by enveloping a circle of given radius inside

        :param corners: Trapezoid corner coordinates
        :param crs: Coordinate Reference System (CRS) string (e.g. 'epsg:4326')
        :raise: :class:`.GeoValueError` if the trapezoid is not valid (convex isosceles trapezoid)
        """
        self._geoseries = GeoSeries([Polygon(corners.squeeze())], crs=crs)

    def __post_init__(self):
        """Post-initialization validity checks"""
        if not (self._geoseries[0].is_valid and self._is_convex_isosceles_trapezoid()):
            raise GeoValueError(f'Not a valid convex isosceles trapezoid: {self._geoseries[0]}')

    def _is_convex_isosceles_trapezoid(self, diagonal_length_tolerance: float = 0.1) -> bool:
        """Returns True if the quadrilateral is a convex isosceles trapezoid

        .. note::
            If the estimated field of view (FOV) is not a convex isosceles trapezoid, it is a sign that (1) the match
            was bad or (2) the gimbal the camera is mounted on has not had enough time to stabilize (camera has
            non-zero roll). Matches where the FOV is not a convex isosceles trapezoid should be rejected assuming we
            can't tell (1) from (2) and that it is better to wait for a good position estimate than to use a bad one.

        .. seealso::
            :func:`.create_src_corners` for the assumed order of the quadrilateral corners.

        :param diagonal_length_tolerance: Tolerance for relative length difference between trapezoid diagonals
        :return: True if the quadrilateral is a convex isosceles trapezoid
        """
        ul, ll, lr, ur = tuple(map(lambda pt: pt.squeeze().tolist(), self.coords))

        # Check convexity (exclude self-crossing trapezoids)
        # Note: inverted y-axis, origin at upper left corner of image
        if not (ul[0] < ur[0] and ul[1] < ll[1] and lr[0] > ll[0] and lr[1] > ur[1]):
            return False

        # Check diagonals same length within tolerance
        ul_lr_length = math.sqrt((ul[0] - lr[0]) ** 2 + (ul[1] - lr[1]) ** 2)
        ll_ur_length = math.sqrt((ll[0] - ur[0]) ** 2 + (ll[1] - ur[1]) ** 2)
        if abs((ul_lr_length / ll_ur_length) - 1) > diagonal_length_tolerance:
            return False

        return True


class GeoValueError(ValueError):
    """Exception returned if a valid GeoObject could not be initialized from provided values."""
    pass