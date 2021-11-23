import pyproj
import cv2
import numpy as np
import sys
import os
import math
import geojson

from functools import partial
from shapely.ops import transform
from shapely.geometry import Point
from collections import namedtuple

BBox = namedtuple('BBox', 'left bottom right top')
LatLon = namedtuple('LatLon', 'lat lon')
Pixel = namedtuple('Pixel', 'x y')
Dimensions = namedtuple('Dimensions', 'height width')
RPY = namedtuple('RPY', 'roll pitch yaw')

MAP_RADIUS_METERS_DEFAULT = 400


def get_bbox(latlon, radius_meters=MAP_RADIUS_METERS_DEFAULT):
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
                         pyproj.Proj(proj_str.format(lat=latlon.lat, lon=latlon.lon)),
                         pyproj.Proj('+proj=longlat +datum=WGS84'))
    circle = Point(0, 0).buffer(radius_meters)
    circle_transformed = transform(projection, circle).exterior.coords[:]
    lons_lats = list(zip(*circle_transformed))
    return BBox(min(lons_lats[0]), min(lons_lats[1]), max(lons_lats[0]), max(lons_lats[1]))


def fov_to_bbox(fov_wgs84):
    """Returns BBox for WGS84 field of view."""
    left, bottom, right, top = 180, 90, -180, -90
    for pt in fov_wgs84:
        right = pt[1] if pt[1] >= right else right
        left = pt[1] if pt[1] <= left else left
        top = pt[0] if pt[0] >= top else top
        bottom = pt[0] if pt[0] <= bottom else bottom
    return BBox(left, bottom, right, top)


def _make_keypoint(pt, sz=1.0):
    """Converts tuple to a cv2.KeyPoint."""
    return cv2.KeyPoint(pt[0], pt[1], sz)


def visualize_homography(figure_name, img_arr, map_arr, kp_img, kp_map, dst_corners):
    """Visualizes a homography including keypoint matches and field of view."""
    # Make a list of cv2.DMatches that match mkp_img and mkp_map one-to-one
    kp_count = len(kp_img)
    assert kp_count == len(kp_map), 'Keypoint counts for img and map did not match.'
    matches = list(map(lambda i: cv2.DMatch(i, i, 0), range(0, kp_count)))

    # Need cv2.KeyPoints for keypoints
    kp_img = np.apply_along_axis(_make_keypoint, 1, kp_img)
    kp_map = np.apply_along_axis(_make_keypoint, 1, kp_map)

    map_with_fov = cv2.polylines(map_arr, [np.int32(dst_corners)], True, 255, 3, cv2.LINE_AA)
    draw_params = dict(matchColor=(0, 255, 0), singlePointColor=None, matchesMask=None, flags=2)
    out = cv2.drawMatches(img_arr, kp_img, map_with_fov, kp_map, matches, None, **draw_params)
    cv2.imshow(figure_name, out)
    cv2.waitKey(1)

    return out


def move_distance(latlon, azmth_dist):
    """Returns LatLon given distance in the direction of azimuth (degrees) from original point."""
    azmth, dist = azmth_dist  # TODO: silly way of providing these args just to map over a zipped list in _update_map, fix it
    g = pyproj.Geod(ellps='WGS84')
    lon, lat, azmth = g.fwd(latlon.lon, latlon.lat, azmth, dist)
    return LatLon(lat, lon)


def get_fov(img_arr, h_mat):
    """Calculates field of view (FoV) corners from homography and image shape."""
    h, w, _ = img_arr.shape  # height before width in np.array shape
    src_corners = create_src_corners(h, w)
    dst_corners = cv2.perspectiveTransform(src_corners, h_mat)
    return dst_corners


def create_src_corners(h, w):
    return np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(-1, 1, 2)


def setup_sys_path():
    """Adds the package share directory to the path so that SuperGlue can be imported."""
    if 'get_package_share_directory' not in sys.modules:
        from ament_index_python.packages import get_package_share_directory
    package_name = 'wms_map_matching'  # TODO: try to read from somewhere (e.g. package.xml)
    share_dir = get_package_share_directory(package_name)
    superglue_dir = os.path.join(share_dir, 'SuperGluePretrainedNetwork')  # todo: set this stuff up in the superglue adapter module
    sys.path.append(os.path.abspath(superglue_dir))
    return share_dir, superglue_dir


def convert_fov_from_pix_to_wgs84(fov_in_pix, map_raster_padded_dim, map_raster_bbox, map_raster_rotation, img_dim,
                                  uncrop=True):
    """Converts the field of view from pixel coordinates to WGS 84.

    Arguments:
        fov_in_pix - Numpy array of field of view corners in pixel coordinates of rotated map raster.
        map_raster_padded_dim - Size of the padded map raster image.
        map_raster_bbox - The WGS84 bounding box of the padded map raster.
        map_raster_rotation - The rotation that was applied to the map raster before matching in radians.
        img_dim - Size of the image.
        uncrop - Flag to determine whether uncropping will be included.
    """
    if uncrop:
        uncrop = partial(uncrop_pixel_coordinates, img_dim, map_raster_padded_dim)
        fov_in_pix_uncropped = np.apply_along_axis(uncrop, 2, fov_in_pix)
    else:
        fov_in_pix_uncropped = fov_in_pix

    rotate = partial(rotate_point, map_raster_rotation, map_raster_padded_dim)   # why not negative here?
    fov_in_pix_unrotated = np.apply_along_axis(rotate, 2, fov_in_pix_uncropped)

    convert = partial(convert_pix_to_wgs84, map_raster_padded_dim, map_raster_bbox)
    fov_in_wgs84 = np.apply_along_axis(convert, 2, fov_in_pix_unrotated)

    return fov_in_wgs84, fov_in_pix_uncropped, fov_in_pix_unrotated  # TODO: only return wgs84


def rotate_point(radians, img_dim, pt):
    """Rotates point around center of image by radians, counter-clockwise."""
    cx = img_dim[0] / 2
    cy = img_dim[1] / 2  # Should be same as cx (assuming image or map raster is square)
    cos_rads = math.cos(radians)
    sin_rads = math.sin(radians)
    x = cx + cos_rads * (pt[0] - cx) - sin_rads * (pt[1] - cy)
    y = cy + sin_rads * (pt[0] - cx) + cos_rads * (pt[1] - cy)
    return x, y


def convert_pix_to_wgs84(img_dim, bbox, pt):
    """Converts a pixel inside an image to lat lon coordinates based on the image's bounding box.

    In cv2, y is 0 at top and increases downwards. x axis is 'normal' with x=0 at left."""
    lat = bbox.bottom + (bbox.top - bbox.bottom) * (img_dim.height - pt[1]) / img_dim.height  # TODO: use the 'LatLon' named tuple for pt
    lon = bbox.left + (bbox.right - bbox.left) * pt[0] / img_dim.width
    return lat, lon


def write_fov_and_camera_location_to_geojson(fov, location, fov_center, fov_gimbal, filename_fov='field_of_view.json',
                                             filename_location='estimated_location.json',
                                             filename_fov_center='fov_center.json',
                                             filename_gimbal_fov='projected_field_of_view.json',
                                             filename_gimbal_fov_upper_left='projected_field_of_view_ul.json'):
    # TODO: write simulated drone location also!
    """Writes the field of view and lat lon location of drone and center of fov into a geojson file.

    Arguments:
        fov - Estimated camera field of view.
        location - Estimated camera location.
        map_location - Center of the FoV.
    """
    with open(filename_fov, 'w') as f:
        polygon = geojson.Polygon(
            [list(map(lambda x: tuple(reversed(tuple(x))), fov.squeeze()))])  # GeoJSON uses lon-lat
        geojson.dump(polygon, f)

    with open(filename_gimbal_fov, 'w') as f:
        polygon = geojson.Polygon(
            [list(map(lambda x: tuple(reversed(tuple(x))), fov_gimbal.squeeze()))])  # GeoJSON uses lon-lat
        geojson.dump(polygon, f)

    # Can only have 1 geometry per geoJSON - need to dump this Point stuff into another file
    with open(filename_location, 'w') as f2:
        latlon = geojson.Point(tuple(reversed(location[0:2])))
        geojson.dump(latlon, f2)

    with open(filename_fov_center, 'w') as f3:
        fov_latlon = geojson.Point(tuple(reversed(fov_center[0:2])))
        geojson.dump(fov_latlon, f3)

    with open(filename_gimbal_fov_upper_left, 'w') as f4:
        ul = geojson.Point(tuple(reversed(fov_gimbal.squeeze()[0])))
        geojson.dump(ul, f4)

    # TODO: write gimbal-projected FoV into its own file


def get_camera_apparent_altitude(map_radius, map_dimensions, K):
    """Returns camera apparent altitude using the K of the drone's camera and the map's known ground truth size.

    Assumes same K for the hypothetical camera that was used to take a picture of the (ortho-rectified) map raster.

    Arguments:
        map_radius - The radius in meters of the map raster (the raster should be a square enclosing a circle of radius)
        map_dimensions - The image dimensions in pixels of the map raster.
        K - Camera intrinsic matrix (assume same K for map raster as for drone camera).
    """
    focal_length = K[0]
    width_pixels = map_dimensions.width  # [0]
    return map_radius * focal_length / width_pixels


# TODO: use this to replace "get_camera_apparent_altitude"
def get_camera_distance(focal_length, pixels, ground_truth):
    """Calculates distance of camera to object whose ground truth widht is known."""
    return ground_truth * focal_length / pixels


def get_distance_of_fov_center(fov_wgs84):
    """Calculate distance between middle of sides of FoV based on triangle similarity."""
    midleft = ((fov_wgs84[0] + fov_wgs84[1])*0.5).squeeze()
    midright = ((fov_wgs84[2] + fov_wgs84[3])*0.5).squeeze()
    g = pyproj.Geod(ellps='clrk66')  # TODO: this could be stored in Matcher and this could be a private method there
    _, __, dist = g.inv(midleft[1], midleft[0], midright[1], midright[0])
    return dist


def altitude_from_gimbal_pitch(pitch_degrees, distance):
    """Returns camera altitude if gimbal pitch and distance to center of FoV is known."""
    return distance*math.sin(math.radians(pitch_degrees))


def get_bbox_center(bbox):
    """Returns camera lat-lon location assuming it is in the middle of given bbox (nadir facing camera)."""
    return LatLon(bbox.bottom + (bbox.top - bbox.bottom) / 2, bbox.left + (bbox.right - bbox.left) / 2)


def get_x_y(translation_vector, map_dim):
    """Returns the pixel coordinates corresponding to the relative translation from map center."""
    return map_dim.width/2 + translation_vector[0].squeeze()*map_dim.width,\
           map_dim.height/2 + translation_vector[1].squeeze()*map_dim.height  # TODO: should not need squeeze



def get_camera_lat_lon_alt(translation, rotation, dimensions_img, dimensions_map_padded, bbox, rot):
    """Returns camera lat-lon coordinates in WGS84 and altitude in meters."""
    alt = translation[2] * (2 * MAP_RADIUS_METERS_DEFAULT / dimensions_img.width)  # width and height should be same for map raster # TODO: Use actual radius, not default radius

    camera_position = np.matmul(rotation, translation)
    camera_position[0] = camera_position[0]*dimensions_img.width
    camera_position[1] = camera_position[1]*dimensions_img.height
    print('translation:\n' + str(camera_position))
    camera_position = uncrop_pixel_coordinates(dimensions_img, dimensions_map_padded, camera_position)  # Pixel coordinates in original uncropped frame
    print('uncropped translation\n' + str(camera_position))
    translation_rotated = rotate_point(rot, dimensions_map_padded, camera_position[0:2])  # TODO: why/why not -rot?
    print('uncropped, unrotated: ' + str(translation_rotated))
    lat, lon = convert_pix_to_wgs84(dimensions_map_padded, bbox, translation_rotated)  # dimensions --> dimensions_orig

    return float(lat), float(lon), float(alt)  # TODO: get rid of floats here and do it properly above


def rotate_and_crop_map(map, radians, dimensions):
    # TODO: only tested on width>height images.
    """Rotates map counter-clockwise and then crops a dimensions-sized part from the middle.

    Map needs padding so that a circle with diameter of the diagonal of the img_size rectangle is enclosed in map."""
    cv2.imshow('padded', map)
    cv2.waitKey(1)
    assert map.shape[0:2] == padded_map_size(dimensions)
    cx, cy = tuple(np.array(map.shape[0:2]) / 2)
    degrees = math.degrees(radians)
    r = cv2.getRotationMatrix2D((cx, cy), degrees, 1.0)
    map_rotated = cv2.warpAffine(map, r, map.shape[1::-1])
    cv2.imshow('rotated', map_rotated)
    cv2.waitKey(1)
    map_cropped = crop_center(map_rotated, dimensions)
    cv2.imshow('cropped', map_cropped)
    cv2.waitKey(1)
    return map_cropped, map_rotated  # TODO: return cropped only


def crop_center(img, dimensions):
    # TODO: only tested on width>height images.
    """Crops dimensions sized part from center."""
    cx, cy = tuple(np.array(img.shape[0:2]) / 2)  # TODO: could be passed from rotate_and_crop_map instead of computing again
    img_cropped = img[math.floor(cy - dimensions.height / 2):math.floor(cy + dimensions.height / 2),
                      math.floor(cx - dimensions.width / 2):math.floor(cx + dimensions.width / 2)]   # TODO: use floor or smth else?
    assert (img_cropped.shape[0:2] == dimensions.height, dimensions.width), 'Something went wrong when cropping the ' \
                                                                            'map raster. '
    return img_cropped


def uncrop_pixel_coordinates(cropped_dimensions, dimensions, pt):
    """Adjusts the pt x and y coordinates for the original size provided by dimensions."""
    pt[0] = pt[0] + (dimensions.width - cropped_dimensions.width)/2  # TODO: check that 0 -> width and index 1 -> height, could be other way around!
    pt[1] = pt[1] + (dimensions.height - cropped_dimensions.height)/2
    return pt


def padded_map_size(dimensions):
    diagonal = math.ceil(math.sqrt(dimensions.width ** 2 + dimensions.height ** 2))
    return diagonal, diagonal


def get_angle(vec1, vec2, normalize=False):
    """Returns angle in radians between two vectors."""
    if normalize:
        vec1 = vec1 / np.linalg.norm(vec1)
        vec2 = vec2 / np.linalg.norm(vec2)
    dot_product = np.dot(vec1, vec2)
    angle = np.arccos(dot_product)
    return angle
