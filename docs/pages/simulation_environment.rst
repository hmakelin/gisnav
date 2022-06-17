Simulation Environment
===================================================
To use and develop with GISNav, you must setup your simulation environment, which includes setting up ROS 2, PX4 and
the PX4-ROS bridge, and Gazebo. The quickest way is to use the pre-made `Docker`_ script. However, here you will also
find instruction and links to guides to setup everything locally.


PX4 Autopilot and ROS 2 & Gazebo
___________________________________________________

You will need to setup the PX4 Autopilot with `ROS 2 and Gazebo <https://docs.px4.io/master/en/simulation/ros_interface.html>`.



.. _Docker:

Option 1  *(recommended)*: Docker
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

A complete dockerized simulation environment is provided in the TODO repository. Follow the `Read Me`_ instructions to
get started with the dockerized environment. Clone and build the docker repo:

.. code-block:: bash
    :caption: Clone the dockerized simulation environment

    cd $HOME && \
        git clone https://gitlab.com/px4-ros2-map-nav/px4-ros2-map-nav-sim.git

You will then need to build the environment by passing it the ``MAPPROXY_TILE_URL`` and ``NVIDIA_DRIVER_MAJOR_VERSION``
arguments. See the `WMS Endpoint`_ section for instruction on how to get an URL for the ``MAPPROXY_TILE_URL`` argument
if you do not have one yet. The example command uses ``nvidia-smi`` to find the major version installed on your system.

.. code-block:: bash
    :caption: Build it

    cd px4-ros2-map-nav-sim && \
        docker-compose build \
            --build-arg MAPPROXY_TILE_URL="https://example.server.com/tiles/%(z)s/%(y)s/%(x)s" \
            --build-arg NVIDIA_DRIVER_MAJOR_VERSION=$(nvidia-smi | grep -oP 'Driver Version: \K[\d{3}]+') \
            .

Once you have your docker container you can run and terminate your
simulation environment by doing:

.. code-block:: bash
    :caption: Run Gazebo example simulation with typhoon_h480 build target and ksql_airport.world

    docker-compose up -d

.. note::
    You should see the Gazebo and QGroundControl windows pop up soon on your screen. If you do not see them, you may
    need configure your ``xhost``:

    .. code-block:: bash

        xhost TODO

Finally, once you are done with your simulation, you can terminate it from your Terminal window:

.. code-block:: bash
    :caption: Terminate example simulation

    docker-compose down


Option 2: Build It Yourself
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. _QGroundControl:

QGroundControl
**************************************************
`Download and install QGroundControl <https://docs.qgroundcontrol.com/master/en/getting_started/quick_start.html>`_ to
get your ground control software up and running. You will need it to control your drone in the Gazebo simulation.


PX4 Autopilot
**************************************************
You can setup your own PX4-Autopilot by following these instructions.

Open a new terminal window and type in the following command:
.. code-block:: bash

    make px4_sitl_rtps gazebo_typhoon_h480__ksql_airport


PX4-ROS 2 microRTPS bridge
**************************************************
You will need to setup the bridge with the following topic configuration:


gscam2
**************************************************

.. _`WMS endpoint`:

WMS Endpoint
___________________________________________________
The :class:`.BaseNode` class relies on a WMS to get map rasters for the estimated location of the vehicle, which will
then be used as input for the pose estimation. The WMS client :class:`.WMSClient` uses OWSLib and runs in a dedicated
thread, although it can also be configured to run in a dedicated process.

The example configuration uses mapproxy, which is lightweight and can be configured to both cache tiles and proxy a tile
based endpoint, which are common since they are more efficient, into a WMS endpoint. WMS endpiont is needed since the
GetMap request allows specifying a specific bounding box instead of a premade tile.

If your solution is Internet-connected, you can use any WMS endpoint. Otherwise you may choose to run your own mapproxy,
GeoServer or similar server onboard.

Own GIS Server
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
If you want to run your own server or WMS proxy, you may want to consider e.g. these options:

    * `MapProxy <https://mapproxy.org/>`_ (used by the GISNav `Docker`_ example, proxy only)
    * `GeoServer <https://geoserver.org/>`_ (full-fledged `OGC-compliant <https://en.wikipedia.org/wiki/Open_Geospatial_Consortium>`_ GIS server)

If you do not want to use commercial high-resolution data, you can load your own server with data from public domain
sources such as:

    * US:

        * `Farm Service Agency Aerial Photography Imagery Products and Programs <https://data.nal.usda.gov/dataset/farm-service-agency-aerial-photography-imagery-products-and-programs>`_

You may want to learn `GDAL <https://gdal.org/>`_ to process your downloaded geospatial products to a format that is
understood by your chosen GIS server.