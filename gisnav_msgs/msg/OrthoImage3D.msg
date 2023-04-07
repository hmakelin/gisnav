# High-resolution orthoimage together with its associated elevation raster (DEM) in one atomic message
geographic_msgs/BoundingBox bbox
sensor_msgs/Image img
sensor_msgs/Image dem               # Set to zero array if DEM is not used
