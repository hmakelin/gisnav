""":class:`pyproj.Proj` instance and related methods"""
import math
from functools import lru_cache
from typing import Optional, Union, Tuple, get_args
from pyproj import Geod

from python_px4_ros2_map_nav.assertions import assert_type


class Proj:
    """Class to hold :class:`pyproj.Proj` instance and WGS84 projection related methods

    The intention is to provide a cached static method to provide a reference to a single :class:`pyproj.Proj` and keep
    related projection operations as class methods.
    """

    # Ellipsoid model used by pyproj
    PYPROJ_ELLIPSOID = 'WGS84'

    def __init__(self, ellps=PYPROJ_ELLIPSOID):
        """Initializer

        Do not instantiate directly, use :meth:`~geod_instance` to get a cached instance
        of :class:`~Proj` to prevent having multiple instances.

        :param ellps: Pyproj ellipsoid model to use
        """
        # Used for pyproj transformations
        self._geod = Geod(ellps=ellps)

    def distance(self, latlon1: Union[Tuple[2*(float,)], Tuple[3*(float,)]],
                 latlon2: Union[Tuple[2*(float,)], Tuple[3*(float,)]]) -> float:
        """Returns distance between two points in meters.

        The distance computation is based on latitude and longitude only and ignores altitude.

        :param latlon1: The first point
        :param latlon2: The second point
        :return: The ground distance in meters between the two points
        """
        #assert_type(latlon1, get_args(Union[LatLon, LatLonAlt]))
        #assert_type(latlon2, get_args(Union[LatLon, LatLonAlt]))
        #_, __, dist = self._geod.inv(latlon1.lon, latlon1.lat, latlon2.lon, latlon2.lat)
        _, __, dist = self._geod.inv(latlon1[1], latlon1[0], latlon2[1], latlon2[0])
        return dist

    def move_distance(self, latlon: Union[Tuple[2*(float,)], Tuple[3*(float,)]],
                      azmth_dist: Tuple[Union[int, float], Union[int, float]]) -> Tuple[float, float]:
        """Returns the point that is a given distance in the direction of azimuth from the origin point.

        :param latlon: Origin point
        :param azmth_dist: Tuple containing azimuth in degrees and distance in meters: (azimuth, distance)
        :return: The point that is given meters away in the azimuth direction from origin
        """
        #assert_type(azmth_dist, tuple)
        #assert_type(latlon, get_args(Union[LatLon, LatLonAlt]))
        azmth, dist = azmth_dist  # TODO: silly way of providing these args just to map over a zipped list in _update_map, fix it
        assert_type(azmth, get_args(Union[int, float]))
        assert_type(dist, get_args(Union[int, float]))
        #lon, lat, azmth = self._geod.fwd(latlon.lon, latlon.lat, azmth, dist)
        lon, lat, azmth = self._geod.fwd(latlon[1], latlon[0], azmth, dist)
        return lat, lon

    def get_bbox(self, latlon: Union[Tuple[2*(float,)], Tuple[3*(float,)]], radius_meters: Optional[Union[int, float]] = None) -> Tuple[4*(float,)]:
        """Gets the bounding box containing a circle with given radius centered at given lat-lon fix.

        If the map radius is not provided, a default value is used.

        :param latlon: Center of the bounding box
        :param radius_meters: Radius of the circle in meters enclosed by the bounding box
        :return: The bounding box
        """
        if radius_meters is None:
            radius_meters = self.get_parameter('map_update.map_radius_meters_default')\
                .get_parameter_value().integer_value
        #assert_type(latlon, get_args(Union[LatLon, LatLonAlt]))
        #assert_type(radius_meters, get_args(Union[int, float]))
        corner_distance = math.sqrt(2) * radius_meters  # Distance to corner of square enclosing circle of radius
        ul = self.move_distance(latlon, (-45, corner_distance))
        lr = self.move_distance(latlon, (135, corner_distance))
        return ul[1], lr[0], lr[1], ul[0]  # left, bottom, right, top

    @staticmethod
    @lru_cache(1)
    def instance(ellps=PYPROJ_ELLIPSOID):
        """Returns a cached :class:`pyproj.Proj` instance

        Assumed to be used only for one ellipsoid model so cache size is set to 1, the default 'ellps' argument
        is there to make the lru_cache decorator work.

        :param ellps: Pyproj ellipsoid model to use
        :return: A :class:`~Proj` instance
        """
        proj = Proj(ellps=ellps)
        return proj

