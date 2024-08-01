# This message represents a stereo image couple composed of an image frame
# from an airborne camera, and an aligned and cropped orthoimage raster of
# approximately the same area.
#
# The "query" and "reference" terminology is used for the image frame and
# and orthoimage raster respectively, as this message is intended to be used for
# pose estimation.
#
# A CRS represented by a proj string is included to convert from the rotated
# and cropped reference pixels back to geographical coordinates.
#
# The precomputed SIFT keypoints and descriptors can be used for query image
# instead of the image itself to improve performance
sensor_msgs/Image query
sensor_msgs/PointCloud2 query_sift  # precomputed SIFT features
sensor_msgs/Image reference  # aligned and cropped orthoimage raster
sensor_msgs/Image dem  # aligned and cropped DEM raster
std_msgs/String crs  # proj string to convert reference pixels to geocoordinates
