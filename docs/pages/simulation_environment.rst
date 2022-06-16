Simulation Environment
--------------------------------------------
To use and develop with GISNav, you must setup your simulation environment, which includes setting up ROS 2, PX4 and
the PX4-ROS bridge, and Gazebo. The quickest way is to use the pre-made `Docker`_ script. However, here you will also
find instruction and links to guides to setup everything locally.

.. Docker_

Docker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
A complete dockerized simulation environment is provided.


PX4 Autopilot
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
You can setup your own PX4-Autopilot by following these instructions.


PX4-ROS 2 microRTPS bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
You will need to setup the bridge with the following topic configuration:


WMS Endpoint
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The :class:`.MapNavNode` class relies on a WMS to get map rasters for the estimated location of the vehicle, which will
then be used as input for the pose estimation. The WMS client :class:`.WMSClient` uses OWSLib and runs in a dedicated
thread, although it can also be configured to run in a dedicated process.

The example configuration uses mapproxy, which is lightweight and can be configured to both cache tiles and proxy a tile
based endpoint, which are common since they are more efficient, into a WMS endpoint. WMS endpiont is needed since the
GetMap request allows specifying a specific bounding box instead of a premade tile.

If your solution is Internet-connected, you can use any WMS endpoint. Otherwise you may choose to run your own mapproxy,
GeoServer or similar server onboard.

Own GIS Server
*******************************************
If you want to run your own server or WMS proxy, you may want to consider e.g. these options:

    * `MapProxy <https://mapproxy.org/>`_ (used by the GISNav `Docker`_ example, proxy only)
    * `GeoServer <https://geoserver.org/>`_ (full-fledged `OGC-compliant <https://en.wikipedia.org/wiki/Open_Geospatial_Consortium>` GIS server)

If you do not want to use commercial high-resolution data, you can load your own server with data from public domain
sources such as:

    * US:

        * `Farm Service Agency Aerial Photography Imagery Products and Programs <https://data.nal.usda.gov/dataset/farm-service-agency-aerial-photography-imagery-products-and-programs>`_

You may want to learn `GDAL <https://gdal.org/>`_ to process your downloaded geospatial products to a format that is
understood by your chosen GIS server.