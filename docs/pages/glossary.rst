Glossary
====================================================

.. warning::
    These definitions are constantly revised as the software is developed so
    do not rely on these just yet. They may also not perfectly correspond to
    what is currently written in the source code.

Terminology
____________________________________________________

Here we provide explanations for terms used throughout this documentation that are
more esoteric or represent jargon i.e. have meanings unique to the context of this
documentation, deviating from their more generally understood interpretations.

Some terms may have multiple definitions (denoted by bullet points) and the
correct alternative should be easily inferred from the context they are used in.

.. glossary::

    Altitude
        Altitude of :term:`vehicle` in any vertical datum or reference level.

        .. todo::
            Describe AGL, AMSL, ellipsoid and other flavors of altitude used by GISNav.

    Extension
    Extended functionality
        Functionality beyond GISNav :term:`core` functionality. For example,
        :class:`.MockGPSNode` for integrating GISNav as a :term:`GPS` substitute
        or complement, or :class:`.RVizNode` for integrating GISNav with :term:`RViz`.

    Autopilot
        Autopilot flight control software such as :term:`PX4` or :term:`ArduPilot`.

    Blending
        Blending of multiple :term:`GPS` sensors by the :term:`navigation filter`.

    Bounding box
        A geographical box or rectangle, the coordinates of which are known, that
        bounds an area of interest to be used as a :term:`reference` for pose
        estimation.

        .. seealso:: Learn more
            https://wiki.openstreetmap.org/wiki/Bounding_Box

    Bucket
        An :term:`AWS` S3 bucket

    Camera
        A standard RGB color camera carried :term:`onboard` that is used by GISNav
        for pose estimation.

    Companion
    Companion computer
        The :term:`onboard` companion computer that GISNav runs on. E.g Nvidia
        :term:`Jetson Nano`.

    Container
        A :term:`Docker` container.

    Core
    Core functionality
        Deployment or implementation of GISNav which is enough to implement
        core functionality needed by an :term:`extension`. GISNav is intended
        to be extended by adding more application or integration specific
        :term:`nodes` instead of adding new features to :term:`core` nodes.
        Consists of :class:`.GISNode` and :class:`.CVNode`.

        .. seealso::
            :ref:`Core data flow graph`

    Coverage
        * Code coverage
        * :term:`Onboard` :term:`GIS` embedded orthoimagery coverage over
          flight :term:`mission` area

    Data flow
    Data flow graph
        Distributed :term:`ROS` topology. From which :term:`node` :term:`publisher`
        and via which :term:`topic` is data flowing to which node and
        :term:`subscriber`.

        .. todo::
            Use ROS terminology and call it computation graph?

    Decorator
        A :term:`Python` decorator function.

    Deploy
    Deployment
        A GISNav deployment consisting of various configurations of
        :term:`Docker Compose` services depending on deployment type.

    Elevation
        Elevation of the ground surface or :term:`ground track` in any vertical
        datum or reference level.

        .. seealso::
            :term:`DEM`

        .. todo::
            Describe AGL, AMSL, ellipsoid and other flavors of elevation used by GISNav.

    Firmware
        :term:`Autopilot` software that is loaded onto and executed on
        the :term:`FMU`. More specifically, :term:`PX4` or :term:`ArduPilot`
        software running on the :term:`FMU`, for example in :term:`HIL`
        simulation.

    Frame
        * A spatial coordinate reference frame
        * An :term:`image` frame (i.e. a single frame from a video stream)

    Geopose
        A :term:`pose` containing a :term:`global position` and :term:`orientation`.
        More specifically, a ``geographic_msgs/GeoPose`` or
        ``geographic_msgs/GeoPoseStamped`` type :term:`ROS` :term:`message` .

        .. todo::
            Need separate term for a global orientation (in earth fixed frame)?
            Geopose specifically requires an orientation in earth fixed frame
            and not a relative orientation.

    GetFeatureInfo
        A :term:`WMS` operation for requesting non-:term:`raster` features from
        :term:`GIS` servers.

    GetMap
        A :term:`WMS` operation for requesting :term:`raster` images from
        :term:`GIS` servers.

        .. seealso:: Learn more
            https://opengeospatial.github.io/e-learning/wms/text/operations.html#getmap

    Absolute position
    Global position
        Horizontal and vertical position in a :term:`CRS` that tells the location
        of the :term:`vehicle` relative to Earth.

        .. seealso::
            :term:`Relative position`, :term:`Local position`

    Ground control
    Ground control software
    Ground control station
        Ground control software that controls the :term:`vehicle` through
        a remote radio link, using a protocol such as :term:`MAVLink`.

    Ground track
        The :term:`vehicle` flight path projected to the ground directly below the
        vehicle, in the direction of :term:`nadir`.

    Home
        :term:`Vehicle` local origin or home position as defined by its
        :term:`navigation filter`.

        .. todo::
            This is still poorly defined -  the home and local origin may be
            different.

    Image
        * A :term:`Docker` image
        * A single image frame from the :term:`camera`

        .. seealso::
            :term:`Query image`

        .. warning::
            Not to be confused with :term:`Orthoimage` or :term:`Imagery`

    Launch
    Launch test
        Launching using the :term:`ROS` launch system, ROS launch tests.

    Relative position
    Local position
        Horizontal and vertical position that tells the location of the
        :term:`vehicle` relative to :term:`home`.

        .. note::
            The term "local position" often includes :term:`vehicle` attitude,
            while the term "relative position" only includes its position.
            But this distinction is currently not well established throughout
            the documentation.

        .. seealso::
            :term:`Absolute position`, :term:`Global position`

    Map
        .. todo::
            Available (not defined here). For terms that could all be
            considered "maps", see "orthoimage", "orthophoto", "raster",
            "reference", "DEM", and "stack".

    Message
        A :term:`ROS` message.

    Middleware
        A software application that facilitates communication between other
        software applications (by transmitting data between them). More
        specifically, :term:`MAVROS` or :term:`micro-ROS Agent`.

    Mission
    Mission mode
        * A flight mission, typically a file uploaded to a :term:`GCS` which then
          sends the appropriate commands to the :term:`vehicle` for executing the
          flight mission.
        * :term:`PX4` Mission :term:`mode`

    Mode
        :term:`Autopilot` flight mode

    Model
        * A machine learning model or neural :term:`network`, used for e.g.
          :term:`camera` :term:`pose` estimation
        * A :term:`Gazebo` model, more specifically a :term:`vehicle` model

    Module
        A :term:`Python` module.

    Nadir
        Direction pointing directly down from the :term:`vehicle` (opposed to
        :term:`zenith`). Does not mean down relative to vehicle body but rather
        the direction of the force of gravity.

    Navigation filter
        An algorithm implemented by the :term:`FMU` that is responsible for
        determining :term:`global position` and :term:`local position` based
        on available sensor inputs.

        .. note::
            :term:`EKF` is one commonly used algorithm and is often used
            interchangeably to describe the navigation filter, even if the
            navigation filter does not use EKF.

    Network
        A neural network (a machine learning :term:`model`), such as SuperGlue and
        LoFTR

    Node
        A :term:`ROS` node.

    Notebook
        A :term:`Jupyter notebook`.

    Offboard
        Anything that is not :term:`onboard`. More specifically any computer
        (e.g. running the :term:`GCS`) that is not carried :term:`onboard`
        and does not draw power from the :term:`vehicle` battery.

    Onboard
        Anything carried by the :term:`vehicle` that would draw power from its
        battery, including the :term:`FMU` and the :term:`companion computer`.

    Orientation
        :term:`Vehicle` or :term:`camera` orientation (attitude) in 3D space,
        typically represented by a :term:`quaternion`.

        .. seealso::
            :term:`RPY` for Euler angle representation

    Origin
        .. todo::
            Available

    Imagery
    Orthoimagery
    Orthoimage
    Orthophoto
        * Orthorectified high-resolution geographic imagery stored in :term:`GIS`
        * An orthorectified high-resolution image of a location on Earth for
          which the :term:`bounding box` is known, retrieved from a :term:`GIS`
          system.

        .. seealso:: Learn more
             https://en.wikipedia.org/wiki/Orthophoto

        .. todo::
            The jargon here is still a bit fuzzy: sometimes the aligned and
            stacked :term:`DEM` :term:`raster` is included in the term
            "orthoimage", and the term "orthophoto" is used for the
            high-resolution image only.

    Query
    Query image
        In a pose estimation context, the :term:`image` frame from the
        :term:`camera`, to be compared to the :term:`reference`
        :term:`orthoimage`.

    Package
        * A :term:`ROS 2` (colcon) package
        * A :term:`Python` package

    Parameter
        Most likely one of these:

        * A :term:`ROS 2` parameter
        * A :term:`PX4` parameter
        * An :term:`ArduPilot` parameter

    Path
        A series of :term:`pose`

    Perspective-n-Point
        A problem in computer vision where a camera :term:`pose` is estimated
        from 2D image to 3D :term:`world` coordinate point correspondences.
        :term:`PnP` is used as an acronym.

        .. seealso:: Learn more
            https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html

    Pose
        A spatial pose in three dimensions including :term:`position` and
        :term:`orientation`.

    Position
        * A :term:`global position`
        * A :term:`local position`

    Publish
    Publisher
        A :term:`ROS` publisher, to publish a ROS :term:`message`.

    Quaternion
        A 4-tuple describing or :term:`orientation` in 3D space. Avoids
        the gimbal lock problem that comes when using Euler angles. Should be
        in (x, y, z, w) order unless otherwise defined.

        .. seealso::
            :term:`RPY` for Euler angle representation of orientation

    Raster
        A rasterized image retrieved from a :term:`GIS` system, as opposed
        to a vectorized image. Used exclusively for geographical imagery,
        not e.g. for an :term:`image` from the :term:`camera`.

        .. seealso:: Learn more
            https://carto.com/blog/raster-vs-vector-whats-the-difference-which-is-best

    Reference
    Reference image
    Reference raster
        In a pose estimation context, the :term:`orthoimage` frame from the
        :term:`GIS` server, to be compared to the :term:`query image`.

        .. todo::
            "Raster" should probably be used exclusively here instead of "image"
            to avoid confusing with the query image.

    Rotation
        .. todo::
            Available

    Service
        * A :term:`Docker Compose` service
        * A :term:`ROS` service

    Service orchestration
        Deploying and managing :term:`Docker Compose` services that
        constitute a GISNav deployment. Currently done using Make (Makefiles).

        .. seealso::
            :term:`Service`

    Stack
    Stacked
    Stacked image
    Stacked raster
        The :term:`orthophoto` stacked together with its aligned :term:`DEM`
        :term:`raster`, representing a "3D orthoimage".

        .. todo::
            Consider dropping this terminology completely and fixing the orthoimage
            and orthophoto terms instead. Alternatively, could be extended by
            stacking all kinds of layers from GIS server besides just orthoimagery
            and DEMs.

    Subscribe
    Subscriber
    Subscription
        A :term:`ROS` subscription, to subscribe to a ROS :term:`topic`.

    Test
        Currently the following kinds of tests are recognized:

        * A unit test
        * A :term:`launch test`
        * A simulation (:term:`SITL` or :term:`HIL`) test

    Topic
        A :term:`ROS` topic.

    Vehicle
        The unmanned aircraft that uses GISNav for navigation. Can e.g. be a
        quadcopter of fixed-wing aircraft.

    World
    World coordinates
    World coordinate system
        * In the :term:`PNP` problem context, the coordinate system of the
          :term:`reference` including the z-axis used to represent ground
          :term:`elevation`.
        * A :term:`Gazebo` world.

    Zenith
        Direction pointing directly up from the :term:`vehicle` (opposed to
        :term:`nadir`). Does not mean up relative to vehicle body but rather the
        direction opposite to the force of gravity.

Abbreviations
____________________________________________________

.. glossary::

    BBox
        :term:`Bounding box`

    Dev
        Development

    Coords
        Coordinates

    Qry
        Query

    Ref
        Reference

    Sim
        Simulation

Acronyms
____________________________________________________

.. glossary::

    AGL
        :term:`Altitude` or :term:`Elevation` Above Ground Level

    AMSL
        :term:`Altitude` or :term:`Elevation` Above Mean Sea Level

    API
        Application Programming Interface

    AWS
        Amazon Web Services

    CI
        Continuous Integration

    SRS
    CRS
        Spatial Reference System / Coordinate Reference System

    CV
        Computer Vision

    DEM
        Digital Elevation Model

        .. seealso::
            https://en.wikipedia.org/wiki/Digital_elevation_model

    ENU
        East-North-Up coordinate system

        .. note::
            Up means in the direction of zenith.

    EKF
        Extended Kalman Filter

        .. seealso::
            :term:`Navigation filter`

    FCU
    FMU
        Flight Control Unit / Flight Management Unit

    FOSS
        Free and Open Source Software

        .. seealso::
            https://en.wikipedia.org/wiki/Free_and_open-source_software

    FoV
    FOV
        Field Of View

    FRD
        Front-Right-Down coordinate system.

        .. note::
            Down here means down relative to :term:`vehicle` body, not :term:`nadir`.

    GCS
        :term:`Ground Control Station`

    GIS
        Geographic Information System

    GML
        Geography Markup Language

    GNSS
    GPS
        Global Navigation Satellite System / Global Positioning System

    GPU
        Graphics Processing Unit

    GUI
        Graphical User Interface

    HIL
    HITL
        Hardware In The Loop simulation

    IDE
        Integrated/Interactive Development Environment

    NAIP
        National Agriculture Imagery Program

        .. seealso::
            https://www.usgs.gov/centers/eros/science/usgs-eros-archive-aerial-photography-national-agriculture-imagery-program-naip

    NED
        North-East-Down coordinate system

        .. note::
            Down here means :term:`nadir`.

    OGC
        Open Geospatial Consortium: `ogc.org <https://www.ogc.org/>`_

    OSM
        :term:`OpenStreetMap`

    PnP
    PNP
        :term:`Perspective-n-Point` problem

    PR
        Pull Request

    QGC
        :term:`QGroundControl`

    RDP
        Remote Desktop Protocol

    RPY
        Roll, pitch, yaw

    SITL
        Software In The Loop simulation

    USGS
        United States Geological Survey

    VNC
        Virtual Network Computing

    WMS
    WMTS
        Web Map Service / Web Map Tile Service, two separate :term:`OGC`
        developed communication protocols. WMS allows querying by arbitrary
        :term:`bounding box` while WMTS returns pre-computed tiles in a
        standardized grid.

        .. seealso::
            * https://www.ogc.org/standards/wms

            * https://www.ogc.org/standards/wmts


Proper names
____________________________________________________

This is not an exhaustive list (e.g. does not include many of the specific technologies
used in the project) but should list many of the main ones, especially if they
relate to external interfaces.

.. glossary::

    ArduPilot
        ArduPilot open source autopilot: `ardupilot.org <https://ardupilot.org/>`_

    colcon
        A build automation tool used by :term:`ROS 2`: `colcon.readthedocs.io/en/released/ <https://colcon.readthedocs.io/en/released/>`_

    DDS
    Data Distribution Service
        A :term:`middleware` protocol and standard:
        `dds-foundation.org <https://www.dds-foundation.org/>`_

    Docker
        Software containerization tool: `docker.com <https://www.docker.com/>`_

    Docker Compose
        Tool for defining and running multi-container :term:`Docker` applications:
        `docs.docker.com/compose <https://docs.docker.com/compose/>`_

    Gazebo
        Simulation software: `gazebosim.org <https://gazebosim.org/home>`_

    GDAL
    Geospatial Data Abstraction Library
        Software library for handling geospatial data: `gdal.org <https://gdal.org/>`_

    GSCam
        :term:`ROS` :term:`GStreamer` camera driver: https://github.com/ros-drivers/gscam

    GStreamer
        Open source multimedia framework: `gstreamer.freedesktop.org <https://gstreamer.freedesktop.org/>`_

    Nano
    Jetson Nano
        An Nvidia Jetson Nano computer

    Jupyter
    JupyterLab
    Jupyter notebook
        A web based :term:`IDE`: `jupyter.org <https://jupyter.org/>`_

    Make
        GNU Make, a build automation tool: `gnu.org/software/make/ <https://www.gnu.org/software/make/>`_

    MapServer
        Open source GIS software: `mapserver.org <https://mapserver.org/>`_

    MAVLink
        MAVLink (Micro Air Vehicle Link) protocol: `mavlink.io <https://mavlink.io/en/>`_

    MAVROS
        An open source :term:`MAVLink` to :term:`ROS` :term:`middleware`:
        `wiki.ros.org/mavros <http://wiki.ros.org/mavros>`_

    MAVSDK
        :term:`MAVLink` software development kit: `mavsdk.mavlink.io/main/en/index.html <https://mavsdk.mavlink.io/main/en/index.html>`_

    micro-ROS Agent
        A :term:`ROS` package that wraps the Micro XRCE-DDS Agent :term:`middleware`:
        `github.com/micro-ROS/micro-ROS-Agent <https://github.com/micro-ROS/micro-ROS-Agent>`_

    OpenCV
        Open source computer vision software library: `opencv.org <https://opencv.org/>`_

    OpenStreetMap
        Open source map of the world: `openstreetmap.org <https://www.openstreetmap.org/>`_

    Pixhawk
        Hardware standard for open source autopilots: `pixhawk.org <https://pixhawk.org/>`_

    PX4
        PX4 Autopilot: `px4.io <https://px4.io/>`_

    Python
        A computer programming language: `python.org <https://www.python.org/>`_

    QEMU
        A :term:`FOSS` full-system emulator: `qemu.org <https://www.qemu.org/>`_

    QGroundControl
        :term:`GCS` software: `qgroundcontrol.com <http://qgroundcontrol.com/>`_

    ROS
    ROS 2
        Robot Operating System: `ros.org <https://www.ros.org/>`_

    RViz
        :term:`ROS` 3D visualization software: https://github.com/ros2/rviz

    Torch
        An open source machine learning software library: `torch.ch <http://torch.ch/>`_

    VRT
        :term:`GDAL` Virtual Format (file format)

    YAML
        A data serialization language: `yaml.org <https://yaml.org/>`_

Other
____________________________________________________

.. glossary::

    KSQL
        ICAO airport code for San Carlos Airport in California (used as simulation
        environment in GISNav development and testing).
