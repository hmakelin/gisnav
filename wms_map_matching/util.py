import pyproj
import cv2
import numpy as np

from functools import partial
from shapely.ops import transform
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
    projection = partial(pyproj.transform,
                         pyproj.Proj(proj_str.format(lat=latlon[0], lon=latlon[1])),
                         pyproj.Proj('+proj=longlat +datum=WGS84'))
    circle = Point(0, 0).buffer(radius_meters)
    circle_transformed = transform(projection, circle).exterior.coords[:]
    lons_lats = list(zip(*circle_transformed))
    return min(lons_lats[0]), min(lons_lats[1]), max(lons_lats[0]), max(lons_lats[1])  # left, bottom, right, top

# TODO: method used for both findHomography and findEssentialMat - are the valid input arg spaces the same here or not?
def process_matches(mkp_img, mkp_map, k, reproj_threshold=1.0, prob=0.999, method=cv2.RANSAC, logger=None):
    """Processes matching keypoints from img and map and returns essential, fundamental, and homography matrices & pose.

    Arguments:
        mkp_img - The matching keypoints from image.
        mkp_map - The matching keypoints from map.
        k - The intrinsic camera matrix.
        reproj_threshold - The RANSAC reprojection threshold for homography estimation.
        prob - Prob parameter for findEssentialMat (used by RANSAC and LMedS methods)
        method - Method to use for estimation.
        logger - Optional ROS2 logger for writing log output.
    """
    if len(mkp_img) < 5 or len(mkp_map) < 5:  # TODO: compute homography anyway if 4 keypoints?
        if logger is not None:
            logger.warn('Not enough keypoints for estimating essential matrix.')
        return None, None, None, None
    if logger is not None:
        logger.debug('Estimating homography.')
    h, status = cv2.findHomography(mkp_img, mkp_map, method, reproj_threshold)
    if logger is not None:
        logger.debug('Estimating essential matrix.')
    e, mask = cv2.findEssentialMat(mkp_img, mkp_map, np.eye(3), threshold=reproj_threshold, prob=prob, method=method)
    if logger is not None:
        logger.debug('Recovering pose from essential matrix e=\n{}'.format(e))
    p, r, t, mask = cv2.recoverPose(e, mkp_img, mkp_map, k, mask)
    f = e  # TODO: fundamental matrix computation missing
    if logger is not None:
        logger.debug('Estimation complete,\ne=\n{},\nf=\n{},\nh=\n{},\np=\n{}.\n'.format(e, f, h, p))
    return e, f, h, p