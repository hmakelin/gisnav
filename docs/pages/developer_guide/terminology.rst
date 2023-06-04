Terms, abbreviations & acronyms
====================================================

General terminology
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

    Autopilot
        Autopilot flight control software such as :term:`PX4` or :term:`ArduPilot`.

    Bounding box
        A geographical box or rectangle, the coordinates of which are known, that
        bounds an area of interest to be used as a :term:`reference` for pose
        estimation.

    Camera
        A standard RGB color camera carried :term:`onboard` that is used by GISNav
        for pose estimation.

    Companion
    Companion computer
        The :term:`onboard` companion computer that GISNav runs on. E.g Nvidia
        Jetson Nano.

    Container
        A :term:`Docker` container.

    Decorator
        A :term:`Python` decorator function.

    Elevation
        Elevation of the ground surface or :term:`ground track` in any vertical
        datum or reference level.

        .. seealso::
            :term:`DEM`

        .. todo::
            Describe AGL, AMSL, ellipsoid and other flavors of elevation used by GISNav.


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

        .. note::
            This is still poorly defined -  the home and local origin may be
            different.

    Image
        * A :term:`Docker` image
        * A single image frame from the :term:`camera` (also: :term:`query`)

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

    Middleware
        A software application that facilitates communication between other
        software applications (by transmitting data between them).

    Nadir
        Direction pointing directly down from the :term:`vehicle` (opposed to
        zenith). Does not mean down relative to vehicle body but rather the
        direction of the force of gravity.

    Navigation filter
        An algorithm implemented by the :term:`FMU` that is responsible for
        determining :term:`global position` and :term:`local position` based
        on available sensor inputs.

        .. note::
            :term:`EKF` is one commonly used algorithm and is often used
            interchangeably to describe the navigation filter, even if the
            navigation filter does not use EKF.

    Node
        A :term:`ROS` node.

    Offboard
        Anything that is not :term:`onboard`. More specifically any computer
        (e.g. running the :term:`GCS`) that is not carried :term:`onboard`
        and does not draw power from the :term:`vehicle` battery.

    Onboard
        Anything carried by the :term:`vehicle` that would draw power from its
        battery, including the :term:`FMU` and the :term:`companion computer`.

    Orthoimage
    Orthophoto
        An orthorectified high-resolution image of a location on Earth for
        which the :term:`bounding box` is known, retrieved from a :term:`GIS`
        system.

        .. note::
            The jargon here is still a bit fuzzy: sometimes the :term:`DEM`
            :term:`raster` is included in the term "orthoimage", and the term
            "orthophoto" is used for the high-resolution image only.

    Query
        In a pose estimation context, the :term:`image` frame from the
        :term:`camera`, to be compared to the :term:`reference`
        :term:`orthoimage`.

    Parameter
        A :term:`ROS` parameter.

    Raster
        A rasterized image retrieved from a :term:`GIS` system, as opposed
        to a vectorized image. Used exclusively for geographical imagery,
        not e.g. to an :term:`image` from the :term:`camera`.

    Reference
        In a pose estimation context, the :term:`orthoimage` frame from the
        :term:`GIS` server, to be compared to the :term:`query` :term:`image`.

    Service
        * A :term:`Docker Compose` service
        * A :term:`ROS` service

    Vehicle
        The unmanned aircraft that uses GISNav for guidance. Can e.g. be a quadcopter
        of fixed-wing aircraft.

Abbreviations
____________________________________________________

.. glossary::

    BBox
        :term:`Bounding box`

Acronyms
____________________________________________________

.. glossary::

    AGL
        (Altitude) Above Ground Level

    AMSL
        (Altitude) Above Mean Sea Level

    SRS
    CRS
        Spatial Reference System / Coordinate Reference System

    CV
        Computer Vision

    DEM
        Digital Elevation Model

    EKF
        Extended Kalman Filter

        .. seealso::
            :term:`Navigation filter`

    FCU
    FMU
        Flight Control Unit / Flight Management Unit

    FOV
        Field Of View

    FRD
        Front-Right-Down coordinate system.

        .. note::
            Down here means down relative to :term:`vehicle` body, not :term:`nadir`.

    GCS
        :term:`Ground Control Station`

    GIS
        Geographical Information System

    GNSS
    GPS
        Global Navigation Satellite System / Global Positioning System

    NED
        North-East-Down coordinate system

        .. note::
            Down here means :term:`nadir`.

    OGC
        Open Geospatial Consortium: `ogc.org <https://www.ogc.org/>`_

    QGC
        :term:`QGroundControl`

    RPY
        Roll, pitch, yaw

    WMS
        Web Map Service (an :term:`OGC` developed protocol)

Proper names
____________________________________________________

This is not an exhaustive list (e.g. does not include many of the specific technologies
used in the project) but should list many of the main ones.

.. glossary::

    ArduPilot
        ArduPilot open source autopilot: `ardupilot.org <https://ardupilot.org/>`_

    Docker
        Software containerization tool: `docker.com <https://www.docker.com/>`_

    Docker Compose
        Tool for defining and running multi-container :term:`Docker` applications:
        `docs.docker.com/compose <https://docs.docker.com/compose/>`_

    MAVLink
        MAVLink (Micro Air Vehicle Link) protocol: `mavlink.io <https://mavlink.io/en/>`_

    MAVROS
        An open source :term:`MAVLink` to :term:`ROS` :term:`middleware`:
        `wiki.ros.org/mavros <http://wiki.ros.org/mavros>`_

    OpenCV
        Open source computer vision software library: `opencv.org <https://opencv.org/>`_

    Pixhawk
        Hardware standard for open source autopilots: `pixhawk.org <https://pixhawk.org/>`_

    PX4
        PX4 Autopilot: `px4.io <https://px4.io/>`_

    Python
        A computer programming language: `python.org <https://www.python.org/>`_

    QGroundControl
        :term:`GCS` software: `qgroundcontrol.com <http://qgroundcontrol.com/>`_

    ROS
    ROS 2
        Robot Operating System: `ros.org <https://www.ros.org/>`_

    Torch
        An open source machine learning software library: `torch.ch <http://torch.ch/>`_
