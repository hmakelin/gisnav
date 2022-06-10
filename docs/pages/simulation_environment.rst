.. toctree::
   :maxdepth: 2

Simulation Environment
--------------------------------------------
Docker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
A complete dockerized simulation environment is provided.


PX4 Autopilot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
You can setup your own PX4-Autopilot by following these instructions.


PX4-ROS 2 microRTPS bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
You will need to setup the bridge with the following topic configuration:



WMS Server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The :class:`python_px4_ros2_map_nav_node.nodes.map_nav_node.MapNavNode` class relies on a WMS endpoint to get
map rasters for the estimated location of the vehicle, which will then be used as input for the pose estimation. The
WMS client :class:`python_px4_ros2_map_nav_node.wms.WMSClient` uses OWSLib and runs in its dedicated process.

The example configuration uses mapproxy, which is lightweight and can be configured to both cache tiles and proxy a tile
based endpoint, which are common since they are more efficient, into a WMS endpoint. WMS endpiont is needed since the
GetMap request allows specifying a specific bounding box instead of a premade tile.

If your solution is Internet-connected, you can use any WMS endpoint. Otherwise you may choose to run your own mapproxy,
GeoServer or similar server onboard.



