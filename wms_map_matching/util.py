import pyproj
import cv2
import numpy as np
import sys
import os

from functools import partial
from shapely.ops import transform
from shapely.geometry import Point
from math import pi


def get_bbox(latlon, radius_meters=200):
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
def process_matches(mkp_img, mkp_map, k, reproj_threshold=1.0, prob=0.999, method=cv2.RANSAC, logger=None, affine=False):
    """Processes matching keypoints from img and map and returns essential, and homography matrices & pose.

    Arguments:
        mkp_img - The matching keypoints from image.
        mkp_map - The matching keypoints from map.
        k - The intrinsic camera matrix.
        reproj_threshold - The RANSAC reprojection threshold for homography estimation.
        prob - Prob parameter for findEssentialMat (used by RANSAC and LMedS methods)
        method - Method to use for estimation.
        logger - Optional ROS2 logger for writing log output.
        affine - Boolean flag indicating that transformation should be restricted to 2D affine transformation
    """
    if len(mkp_img) < 5 or len(mkp_map) < 5:  # TODO: compute homography anyway if 4 keypoints?
        if logger is not None:
            logger.warn('Not enough keypoints for estimating essential matrix.')
        return None, None, None, None
    if logger is not None:
        logger.debug('Estimating homography.')
    if not affine:
        h, h_mask = cv2.findHomography(mkp_img, mkp_map, method, reproj_threshold)
    else:
        h, h_mask = cv2.estimateAffinePartial2D(mkp_img, mkp_map)
        h = np.vstack((h, np.array([0, 0, 1])))  # Make it into a homography matrix
    if logger is not None:
        logger.debug('Estimating essential matrix.')
    e, mask = cv2.findEssentialMat(mkp_img, mkp_map, np.eye(3), threshold=reproj_threshold, prob=prob, method=method)
    if logger is not None:
        logger.debug('Recovering pose from essential matrix e=\n{}'.format(e))
    p, r, t, mask = cv2.recoverPose(e, mkp_img, mkp_map, k, mask)
    if logger is not None:
        logger.debug('Estimation complete,\ne=\n{},\nh=\n{},\np=\n{}.\n'.format(e, h, p))
    return e, h, p, h_mask

def get_nearest_cv2_rotation(radians):
    """Finds the nearest 90 degree rotation multiple."""
    deg45 = pi/4  # 45 degrees in radians
    deg135 = 3*deg45
    if -deg45 <= radians < deg45:
        return None
    elif deg45 <= radians < deg135:
        return cv2.ROTATE_90_COUNTERCLOCKWISE  # cv2.ROTATE_90_CLOCKWISE
    elif -deg135 <= radians < -deg45:
        return cv2.ROTATE_90_CLOCKWISE  # cv2.ROTATE_90_COUNTERCLOCKWISE
    elif radians < -deg135 or radians >= deg135:
        return cv2.ROTATE_180
    else:
        raise ValueError('Unexpected input value: {}'.format(radians))  # this should not happen


def _make_keypoint(pair, sz=1.0):
    """Converts tuple to a cv2.KeyPoint.

    Helper function used by visualize homography.
    """
    return cv2.KeyPoint(pair[0], pair[1], sz)


def _make_match(x, img_idx=0):
    """Makes a cv2.DMatch from img and map indices.

    Helper function used by visualize homography.
    """
    return cv2.DMatch(x[0], x[1], img_idx)


# TODO: Remove mas from args, it is no longer used
def visualize_homography(img, map, kp_img, kp_map, h_mat, logger=None):
    """Visualizes a homography including keypoint matches and field of view."""
    h, w = img.shape

    # Make a list of matches
    matches = []
    for i in range(0, len(kp_img)):
        matches.append(cv2.DMatch(i, i, 0))  # TODO: implement better, e.g. use _make_match helper
    matches = np.array(matches)

    # Need cv2.KeyPoints for kps (assumed to be numpy arrays)
    kp_img = np.apply_along_axis(_make_keypoint, 1, kp_img)
    kp_map = np.apply_along_axis(_make_keypoint, 1, kp_map)

    src_corners = np.float32([[0, 0], [0, h-1], [w-1, h-1], [w-1, 0]]).reshape(-1, 1, 2)
    dst_corners = cv2.perspectiveTransform(src_corners, h_mat)
    map_with_fov = cv2.polylines(map, [np.int32(dst_corners)], True, 255, 3, cv2.LINE_AA)
    draw_params = dict(matchColor=(0, 255, 0), singlePointColor=None, matchesMask=None, flags=2)
    if logger is not None:
        logger.debug('Drawing matches.')
    out = cv2.drawMatches(img, kp_img, map_with_fov, kp_map, matches, None, **draw_params)
    cv2.imshow('Matches and FoV', out)
    cv2.waitKey(1)

def setup_sys_path():
    """Adds the package share directory to the path so that SuperGlue can be imported."""
    if 'get_package_share_directory' not in sys.modules:
        from ament_index_python.packages import get_package_share_directory
    package_name = 'wms_map_matching'  # TODO: try to read from somewhere (e.g. package.xml)
    share_dir = get_package_share_directory(package_name)
    superglue_dir = os.path.join(share_dir, 'SuperGluePretrainedNetwork')
    sys.path.append(os.path.abspath(superglue_dir))
    return share_dir, superglue_dir
