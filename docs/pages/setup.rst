**************************************************
Setup
**************************************************
To use and develop with GISNav, you must setup your simulation environment, which includes setting up ROS 2, PX4 and
the PX4-ROS bridge, and Gazebo. The quickest way is to use the pre-made `Docker`_ script. However, here you will also
find instruction and links to guides to setup everything locally.

.. ROS 2:

ROS 2
===================================================
TODO

.. _QGroundControl:

QGroundControl
===================================================
`Download and install QGroundControl <https://docs.qgroundcontrol.com/master/en/getting_started/quick_start.html>`_ to
get your ground control software up and running. You will need it to control your drone in the Gazebo simulation.


PX4 Autopilot
===================================================

You will need to setup the PX4 Autopilot with `ROS 2 and Gazebo <https://docs.px4.io/master/en/simulation/ros_interface.html>`.

You can setup your own PX4-Autopilot by following these instructions.

Open a new terminal window and type in the following command:

.. code-block:: bash

    make px4_sitl_rtps gazebo_typhoon_h480__ksql_airport


PX4-ROS 2 microRTPS bridge
---------------------------------------------------
You will need to setup the bridge with the following topic configuration:

See the
`uorb topic configuration guide <https://docs.px4.io/v1.12/en/middleware/micrortps.html#supported-uorb-messages>`_ for
more information.

Once you have the bridge setup, you can run the microRTPS agent locally with:
```bash
micrortps_agent -t UDP
```


gscam
===================================================
As described in the `Video Streaming <https://docs.px4.io/master/en/simulation/gazebo.html#video-streaming>`_ section
of PX4's User Guide, the ``typhoon_h480`` build target for Gazebo SITL supports UDP video streaming. You can use
``gscam`` to pipe the video into ROS, from where it can be subscribed to by GISNav's :class:`.BaseNode`.

Open a new terminal window and source your ROS environment (ROS ``foxy`` in this example):

.. note::
    If you work with your ROS 2 workspace often, you may want to source it in your ``~/.bashrc``:

    .. code-block:: bash

        echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
        echo "source ~/px4_ros_com_ros2/install/setup.bash" >> ~/.bashrc

.. code-block:: bash

    source /opt/ros/foxy/setup.bash
    source install/setup.bash

Then install ``gscam`` and its dependencies from the
`ROS package index <https://index.ros.org/p/gscam/github-ros-drivers-gscam/>`_ for your ROS distribution :

.. code-block:: bash

    sudo apt-get install -y gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl ros-foxy-gscam


Create a ``gscam_prams.yaml`` and ``camera_calibration.yaml`` files like these ones:

.. seealso::
    See the
    `How to Calibrate a Monocular Camera <https://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration>`_
    ROS tutorial on how to create a camera calibration file if you do not want to use the example file

.. code-block:: yaml
    :caption: gscam_params.yaml

    gscam_publisher:
      ros__parameters:
        gscam_config: >
          gst-launch-1.0 udpsrc uri=udp://127.0.0.1:5600 !
          application/x-rtp,media=video,clock-rate=90000,encoding-name=H264 !
          rtph264depay ! h264parse ! avdec_h264 ! videoconvert
        preroll: False
        use_gst_timestamps: True
        frame_id: 'mono'
        image_encoding: 'rgb8'  # Does not support bgr8, handle this downstream

.. code-block:: yaml
    :caption: camera_calibration.yaml

    image_width: 640
    image_height: 360
    camera_name: cgo3
    camera_matrix:
      rows: 3
      cols: 3
      data: [205.46963709898583, 0, 320, 0, 205.46963709898583, 180, 0, 0, 1]
    distortion_model: plumb_bob
    distortion_coefficients:
      rows: 1
      cols: 5
      data: [0, 0, 0, 0, 0]
    rectification_matrix:
      rows: 3
      cols: 3
      data: [1, 0, 0, 0, 1, 0, 0, 0, 1]
    projection_matrix:
      rows: 3
      cols: 4
      data: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0]


And run ``gscam`` with your new configuration when the PX4 Gazebo SITL is running:

.. code-block:: bash

    ros2 run gscam gscam_node --ros-args --params-file src/gisnav/test/assets/gscam_params.yaml \
        -p camera_info_url:=file://$PWD/src/gisnav/test/assets//camera_calibration.yaml


.. _`WMS endpoint`:

WMS Endpoint
===================================================
The :class:`.BaseNode` class relies on a WMS to get map rasters for the estimated location of the vehicle, which will
then be used as input for the pose estimation. The WMS client :class:`.WMSClient` uses OWSLib and runs in a dedicated
thread, although it can also be configured to run in a dedicated process.

The example configuration uses mapproxy, which is lightweight and can be configured to both cache tiles and proxy a tile
based endpoint, which are common since they are more efficient, into a WMS endpoint. WMS endpiont is needed since the
GetMap request allows specifying a specific bounding box instead of a premade tile.

If your solution is Internet-connected, you can use any WMS endpoint. Otherwise you may choose to run your own mapproxy,
GeoServer or similar server onboard.

You can configure the WMS client via the ROS parameter server, or provide a YAML file when spinning up your node:

.. code-block:: yaml
    :caption: Example YAML configuration of wms ROS parameters

    my_node:
      ros__parameters:
        wms:
          url: 'http://localhost:8080/wms'
          version: '1.1.1'
          layers: ['Imagery']
          srs: 'EPSG:4326'  # don't change this setting, internal logic may often implicitly assume EPSG:4326
          request_timeout: 10
          image_format: 'image/jpeg'

.. note::
    The ``wms.url``, ``wms.version`` and ``wms.timeout`` ROS parameters are read-only because currently there is no
    implementation in :class:`.BaseNode` for re-initializing the underlying :class:`.WMSClient` instance with new
    parameters.


Own GIS Server
----------------------------------------------------
The benefit of running your own GIS server is that you can embed it onboard the drone and not rely on an internet
connection. Accessing map tiles or rasters over the internet may be fine for simulation but most likely not for
real-world use.

If you want to run your own server or WMS proxy, you may want to consider e.g. these options:

    * `MapProxy <https://mapproxy.org/>`_ (used by the GISNav `Docker`_ example, proxy only but can cache tiles locally)
    * `GeoServer <https://geoserver.org/>`_ (full-fledged `OGC-compliant <https://en.wikipedia.org/wiki/Open_Geospatial_Consortium>`_ GIS server)

If you do not want to use commercial high-resolution data, you can load your own server with data from public domain
sources such as:

    * `OSM-curated Aerial Imagery <https://wiki.openstreetmap.org/wiki/Aerial_imagery>`_

        * Large list of sources with various licensing terms, see terms of use for each service individually

    * `Farm Service Agency Aerial Photography Imagery Products and Programs <https://data.nal.usda.gov/dataset/farm-service-agency-aerial-photography-imagery-products-and-programs>`_

        * US coverage only

.. note::
    Commercial web-based map services are often
    `tile-based <https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames>`_ (as opposed to WMS) because it is more
    efficient to serve pre-computed tiles than to compute unique rasters for each individual requested bounding box.
    You will need a WMS proxy if you decide to go with a tile-based endpoint.


.. warning::
    Many commercial services explicitly prohibit the caching of map tiles in their licensing terms, especially if their
    business model is based on billing API requests. This is mainly to prevent disintermediation in case their tiles
    are redistributed to a large number of end users.

    While caching tiles onboard your own drone is likely not the kind of misuse targeted by such clauses, you should
    still make sure you understand the Terms of Use of the service you are using and that it fits your planned use case.


.. seealso::
    You may want to learn `GDAL <https://gdal.org/>`_ to process your downloaded geospatial products to a format that is
    understood by your chosen GIS server.

