**************************************************
Setup
**************************************************
This page provides instruction on how to setup a local GISNav development and SITL simulation environment.

`Dockerized environments <https://github.com/hmakelin/gisnav-docker>`_ are also available. Take a look at the
Dockerfiles to see how everything is set up. These instructions should closely mirror what is included in the
Dockerfiles with only minor differences.

Prerequisites
===================================================

* These instructions assume you are running **Ubuntu 20.04 (Focal Fossa)**, although with small changes other releases
  might also work.

* It is strongly recommended that you have an **NVIDIA GPU and CUDA** installed. You can inspect your NVIDIA driver and
  CUDA versions with the ``nvidia-smi`` command line utility. If you don't have it installed, follow the `NVIDIA CUDA
  Installation Guide for Linux <https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html>`_.

PX4 Autopilot
===================================================
PX4 **v1.13** is the only autopilot that is currently supported by GISNav.

Follow the PX4 instructions to setup your `Ubuntu Development Environment
<https://docs.px4.io/master/en/simulation/ros_interface.html>`_ with `Fast DDS
<https://docs.px4.io/master/en/dev_setup/fast-dds-installation.html>`_.

Once you have installed PX4 Autopilot, you can try out the Gazebo simulation to make sure everything is working
correctly. The following command should pop out a Gazebo window with a Typhoon H480 drone sitting somewhere in the
vicinity of San Carlos (KSQL) airport:

.. code-block:: bash

    cd ~/PX4-Autopilot
    make px4_sitl_rtps gazebo_typhoon_h480__ksql_airport

.. note::
    The initial build may take several minutes.

Mock GPS Node Demo Configuration
___________________________________________________
If you are planning to use :class:`.MockGPSNode`, you should adjust the following PX4 parameters to make GISNav work
better either through the PX4 shell, through QGroundControl, or in the
``~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/6011_typhoon_h480`` file before making the build:

.. warning::
    This configuration is intended for simulation use only

.. code-block::

    param set-default NAV_ACC_RAD 20.0
    param set-default MPC_YAWRAUTO_MAX 10.0

    param set-default COM_POS_FS_DELAY 5

    param set-default EKF2_GPS_P_NOISE 10
    param set-default EKF2_GPS_V_NOISE 3

    param set-default SENS_GPS_MASK 2

.. note::
    This is a sample configuration that seems to work, but you may want to experiment with the parameters.

    It is important to make the waypoint turns softer and/or to reduce the yaw rate especially if the camera has some
    pitch (is not completely nadir-facing) to ensure the field of view does not move or rotate* too quickly for GISNav.
    Otherwise GISNav may lose track of position for long enough for the position delay failsafe to trigger before GISNav
    can find the drone again. Increasing the position failsafe delay helps if your GPU is a bit slower or GISNav for some
    reason cannot produce a position estimate for a number of subsequent frames for one reason or another. However as a
    failsafe parameter it should not be made unreasonably large.

    The other parameters are mainly to increase tolerance for variation in the GPS position estimate. GISNav in its
    default configuration seems to be more accurate in estimating vertical position than horizontal position, so the
    example has lower tolerance for vertical position error.

    `*camera yaw rotation speed may be less of an issue if a rotation agnostic neural network is used (not the case by
    default)`

.. _ROS 2 Workspace:

ROS 2 Workspace
===================================================
GISNav requires ROS 2 to communicate with PX4 Autopilot and is therefore structured as a ROS 2 package.

Follow the `PX4 instructions to setup ROS 2 and the PX4-ROS 2 bridge
<https://docs.px4.io/main/en/ros/ros2_comm.html#installation-setup>`_.

Once you have your ROS 2 workspace set up, consider automatically sourcing it in your ``~/.bashrc`` to avoid
manual repetition:

.. code-block:: bash

    echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
    echo "source ~/px4_ros_com_ros2/install/setup.bash" >> ~/.bashrc

.. note::
    The PX4 tutorial uses ``px4_ros_com_ros2`` for the workspace name, while the ``gisnav-docker`` container image uses
    ``colcon_ws``.

.. _PX4-ROS 2 Bridge:

PX4-ROS 2 Bridge
===================================================
The default configuration of the PX4-ROS 2 bridge is not sufficient for GISNav. The bridge must be reconfigured and
the ``micrortps_agent`` re-generated.

To reconfigure the bridge, see the `ROS 2 Offboard Control Example
<https://docs.px4.io/main/en/ros/ros2_offboard_control.html#ros-2-offboard-control-example>`_ on how to edit the
``urtps_bridge_topics.yaml`` file in the ``PX4-Autopilot/msg/tools`` and ``px4_ros_com_ros2/src/px4_ros_com/templates``
folders. You must configure the following send and receive flags for the following topics:

.. list-table:: ``urtps_bridge_topics.yaml``
   :header-rows: 1

   * - PX4-Autopilot/msg/tools
     - px4_ros_com_ros2/src/px4_ros_com/templates
   * - .. code-block:: yaml

            - msg: vehicle_local_position
              send: true
              ...
            - msg: vehicle_global_position
              send: true
              ...
            - msg: vehicle_attitude
              send: true
              ...
            - msg: gimbal_device_set_attitude
              send: true
              ...
            - msg: sensor_gps
              receive: true
     - .. code-block:: yaml

            - msg: VehicleLocalPosition
              send: true
              ...
            - msg: VehicleGlobalPosition
              send: true
              ...
            - msg: VehicleAttitude
              send: true
              ...
            - msg: GimbalDeviceSetAttitude
              send: true
              ...
            - msg: SensorGps
              receive: true

.. note::
    * The ``SensorGps`` topic is used by :class:`.MockGPSNode` and is optional if you are only using :class:`.BaseNode`.
      Remember to add any other topics here that you might be using if you are extending :class:`.BaseNode`.
    * The `Dockerfile for the SITL image
      <https://github.com/hmakelin/gisnav-docker/blob/master/docker/px4-sitl/Dockerfile>`_ uses the
      `configure_urtps_bridge_topics.py
      <https://github.com/hmakelin/gisnav-docker/blob/master/scripts/configure_urtps_bridge_topics.py>`_
      script to automatically configure the above topics before building the PX4 SITL target.

PX4-ROS 2 Bridge Troubleshooting
___________________________________________________

Ensure you have your new workspace sourced before moving on to next steps:

.. code-block:: bash

    cd ~/px4_ros_com_ros2
    source /opt/ros/foxy/setup.bash
    source install/setup.bash

You can check whether your new configuration works by running ``micrortps_agent`` and inspecting the console output:

.. code-block:: bash

    micrortps_agent -t UDP

If your new topics are not listed, you can try cleaning both the ``px4_ros_com_ros2`` workspace and your PX4 build
before rebuilding again:

.. code-block:: bash
    :caption: Clean ROS 2 workspace

    cd ~/px4_ros_com_ros2/scripts
    ./clean_all.bash

.. code-block:: bash
    :caption: Clean PX4 build

    cd ~/PX4-Autopilot
    make clean

.. note::
    *Unverified*:
    When GISNav is running, it will try to exit cleanly when ``Ctrl+C`` is pressed. However, if the combination is
    mashed quickly in succession the clean exit may fail and leave some subscriptions hanging. In this case you may
    want to restart ``micrortps_agent``.

gscam
===================================================

The ``typhoon_h480`` build target for Gazebo SITL supports UDP `video streaming
<https://docs.px4.io/master/en/simulation/gazebo.html#video-streaming>`_ . Here we will use ``gscam`` to publish the
UDP video stream to ROS 2 to make it accessible to GISNav:

Install ``gscam`` and dependencies:

.. code-block:: bash

    sudo apt-get install -y gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl ros-foxy-gscam

The GISNav repository includes a sample camera configuration that we will use. Run ``gscam`` in a dedicated bash shell
with the provided configuration files:

.. code-block:: bash

    cd ~/px4_ros_com_ros2
    ros2 run gscam gscam_node --ros-args --params-file src/gisnav/test/assets/gscam_params.yaml \
        -p camera_info_url:=file://$PWD/src/gisnav/test/assets/camera_calibration.yaml

.. seealso::
    See
    `How to Calibrate a Monocular Camera <https://wiki.ros.org/camera_calibration/Tutorials/MonocularCalibration>`_
    on how to create a custom camera calibration file if you do not want to use the provided example

gscam Troubleshooting
___________________________________________________

.. note::
    *Unverified*:
    When GISNav is running, it will try to exit cleanly when ``Ctrl+C`` is pressed. However, if the combination is
    mashed quickly in succession the clean exit may fail and leave some subscriptions hanging. In this case you may
    want to restart ``gscam``.

.. _QGroundControl:

QGroundControl
===================================================
QGroundControl is a PX4-compatible ground control station software with a graphical user interface. It is needed
for controlling the drone in the SITL (software-in-the-loop) simulation.

Install QGroundControl by following the `official instructions
<https://docs.qgroundcontrol.com/master/en/getting_started/quick_start.html>`_.

You can then run QGroundControl from the directory where you installed it, for example:

.. code-block:: bash

    ~/Applications/QGroundControl.AppImage

QGroundControl Troubleshooting
___________________________________________________

You may need to change the file permissions and/or extract it before running it:

.. code-block:: bash
    :caption: Change file permissions

    cd ~/Applications
    chmod +x QGroundControl.AppImage
    ./QGroundControl.AppImage

.. code-block:: bash
    :caption: Extract and run

    cd ~/Applications
    ./QGroundControl.AppImage --appimage-extract-and-run

.. _`WMS endpoint`:

WMS Endpoint
===================================================
The :class:`.BaseNode` class relies on a WMS to get map rasters for the estimated location of the vehicle, which will
then be used as input for the pose estimation. The WMS client :class:`.WMSClient` uses OWSLib and runs in a dedicated
process, although it can be quite easily changed to run in a separate thread to reduce serialization overhead (no ROS
parameter option exists for this, however).

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

    * `MapProxy <https://mapproxy.org/>`_ (proxy only but can cache tiles locally)
    * `GeoServer <https://geoserver.org/>`_ (full-fledged `OGC-compliant <https://en.wikipedia.org/wiki/Open_Geospatial_Consortium>`_ GIS server)

If you do not want to use commercial high-resolution orthoimagery (from primary sources such as Maxar or secondary
sources like Esri ArcGIS, Google, etc.), you can load your own server with data from public domain or openly licensed
sources such as:

    * `OSM-curated Aerial Imagery <https://wiki.openstreetmap.org/wiki/Aerial_imagery>`_

        * Large list of sources with various licensing terms, see terms of use for each service individually

    * `Open Aerial Map <https://map.openaerialmap.org/>`_

        * Various smaller maps for specific locations, you can also make your own for your area of interest

    * `US Geological Survey <https://www.usgs.gov/programs/national-geospatial-program/imagery>`_

        * US coverage only

    * `UK Environment Agency <https://data.gov.uk/dataset/4921f8a1-d47e-458b-873b-2a489b1c8165/vertical-aerial-photography>`_

        * UK coverage only, ECW format

    * `National Land Survey of Finland <https://www.maanmittauslaitos.fi/en/maps-and-spatial-data/expert-users/product-descriptions/orthophotos>`_

        * Finland coverage only, WMTS, JPEG 2000 format

.. note::
    Commercial web-based map services are often
    `tile-based <https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames>`_ (as opposed to WMS) because it is more
    efficient to serve pre-computed tiles than to compute unique rasters for each individual requested bounding box.
    You will need a WMS proxy if you decide to go with a tile-based endpoint.


.. warning::
    Many commercial services explicitly prohibit the caching of map tiles in their Terms of Use (ToU), especially if
    their business model is based on billing API requests. This is mainly to prevent disintermediation in case their
    tiles are redistributed to a large number of end users.

    While caching tiles onboard your own drone is likely not the kind of misuse targeted by such clauses, you should
    still make sure you understand the ToU of the service you are using and that it fits your planned use case.


.. seealso::
    You may want to learn `GDAL <https://gdal.org/>`_ to process your downloaded geospatial products to a format that is
    understood by your chosen GIS server.

GISNav
===================================================
Install GISNav in your `ROS 2 Workspace`_:

.. code-block:: bash:

    cd ~/px4_ros_com_ros2
    mkdir -p src && cd "$_"
    git clone https://github.com/hmakelin/gisnav.git
    cd gisnav
    pip3 install -r requirements.txt
    pip3 install -r requirements-dev.txt

Download the LoFTR submodule and weights:

.. code-block:: bash

    cd ~/px4_ros_com_ros2/src/gisnav
    git submodule update LoFTR
    pip3 install gdown
    mkdir weights && cd "$_"
    gdown https://drive.google.com/uc?id=1M-VD35-qdB5Iw-AtbDBCKC7hPolFW9UY

.. note::
    The example downloads the dual-softmax (_ds suffix) outdoor weights which are permissively licensed (does not use
    SuperGlue)

Build the GISNav package:

.. code-block:: bash:

    cd ~/px4_ros_com_ros2
    colcon build --packages-select gisnav

Once GISNav is installed, you can run the included :class:`.MockGPSNode` either directly with ``ros2 run``:

.. code-block:: bash

    cd ~/px4_ros_com_ros2
    ros2 run gisnav mock_gps_node --ros-args --log-level info \
        --params-file src/gisnav/config/typhoon_h480__ksql_airport.yaml

Or using the provided launch file:

.. code-block:: bash

    cd ~/px4_ros_com_ros2
    ros2 launch gisnav mock_gps_node.launch.py

