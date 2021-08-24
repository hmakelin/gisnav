from functools import partial
from pyproj import Proj, transform
from shapely.geometry import Point


def get_bbox(latlon, radius_meters):
    """Gets the bounding box containing a circle with given radius centered at given lat-lon fix.

    Uses azimuthal equidistant projection. Based on Mike T's answer at
    https://gis.stackexchange.com/questions/289044/creating-buffer-circle-x-kilometers-from-point-using-python.

    Arguments:
        latlon: The lat-lon tuple (EPSG:4326) for the circle.
        radius_meters: Radius in meters of the circle.

    Returns:
        The bounding box (left, bottom, right, top).
    """
    proj_str = '+proj=aeqd +lat_0={lat} +lon_0={lon} +x_0=0 +y_0=0'
    projection = partial(transform,
                         Proj(proj_str.format(lat=latlon[0], lon=latlon[1])),
                         Proj('+proj=longlat +datum=WGS84'))
    circle = Point(0, 0).buffer(radius_meters)
    circle_transformed = transform(projection, circle).exterior.coords[:]
    lons_lats = list(zip(*circle_transformed))
    return min(lons_lats[0]), min(lons_lats[1]), max(lons_lats[0]), max(lons_lats[1])  # left, bottom, right, top
