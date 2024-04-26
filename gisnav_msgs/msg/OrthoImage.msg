# This message represents an atomic pair of an orthoimage raster and its
# associated digital elevation model (DEM) raster.
# The DEM can improve accuracy by providing z-coordinates when solving
# the PnP problem.
# The CRS is a proj string that converts from raster pixel coordinates to
# geocoordinates.
sensor_msgs/Image image  # orthoimage raster
sensor_msgs/Image dem    # corresponding digital elevation model (DEM)
std_msgs/String crs
