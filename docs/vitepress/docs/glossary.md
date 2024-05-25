# Glossary

::: warning Warning: Frequent revision
These definitions are frequently revised as the software is developed and therefore may not always correspond to what is currently written in the source code.

:::

## Terminology

Here we provide explanations for terms used throughout this documentation that are more esoteric or represent jargon i.e. have meanings unique to the context of this documentation, deviating from their more generally understood interpretations.

Some terms may have multiple definitions (e.g. [parameter](#parameter) or [frame](#frame)) and the correct alternative should be easily inferred from the context. Similarly, multiple terms may be used to refer to the same concept depending on the context (e.g. [robot](#robot) vs. [vehicle](#vehicle)).

### Altitude
Altitude of the [vehicle](#vehicle) in any vertical datum or reference level.

::: info See also
[AGL](#agl), [AMSL](#amsl)

:::

### Anchor
A [YAML](#yaml) anchor

### Extension, extended functionality
- Functionality beyond GISNav [core](#core-core-functionality) functionality. For example,
[NMEANode](#nmea-node) for integrating GISNav as a mock [GPS](#gnss-gps) device over
[NMEA](#nmea-nmea-0183)
- A [Docker Compose extension](https://docs.docker.com/compose/compose-file/11-extension/)

### Autopilot
Autopilot flight control software such as [PX4](#px4) or [ArduPilot](#ardupilot).

### Blending
Blending of multiple [GPS](#gnss-gps) sensors by the [navigation filter](#navigation-filter).

### Bounding box
A geographical box or rectangle, the coordinates of which are known, that bounds an
area of interest to be used as a [reference](#reference) for [pose](#pose) estimation.

::: info Learn more
Learn more at [wiki.openstreetmap.org/wiki/Bounding_Box](https://wiki.openstreetmap.org/wiki/Bounding_Box)

:::

### Bucket
An [AWS](#aws) [S3](#s3) bucket

### Camera
A monocular camera carried [onboard](#onboard) that is used by GISNav for [pose](#pose)
estimation.

### Companion, companion computer
The [onboard](#onboard) companion computer that GISNav runs on. E.g., NVIDIA [Jetson Nano](#nano-jetson-nano).

### Container
A [Docker](#docker) container.

### Core, core functionality
GISNav core functionality refers to parts of GISNav code that are assumed to be needed for every kind of deployment. Opposed to [extension](#extension-extended-functionality), which are typically integrations built downstream of core functionality.

### Coverage
- Code coverage
- [Onboard](#onboard) [GIS](#gis) embedded [orthoimagery](#orthoimagery-imagery-orthoimage-orthophoto) coverage over flight [mission](#mission) area

### Decorator
A [Python](#python) decorator function.

### Deploy, deployment
A GISNav deployment consisting of various configurations of [Docker Compose](#docker-compose) services.

### Drone
Currently NOT used by GISNav. [UAV](#uav), [vehicle](#vehicle) or [robot](#robot) are used instead, depending on context.

### Elevation
Elevation of the ground surface or [ground track](#ground-track) in any vertical datum or reference level.

::: info See also
[DEM](#dem), [Altitude](#altitude), [AGL](#agl), [AMSL](#amsl)

:::

### Extra
Python package extras: [packaging.python.org/en/latest/tutorials/installing-packages/#installing-extras](https://packaging.python.org/en/latest/tutorials/installing-packages/#installing-extras).

Used to separate GISNav [core](#core-core-functionality), [extended](#extension-extended-functionality) and development dependencies from each other.

### Firmware
[Autopilot](#autopilot) software that is loaded onto and executed on the [FCU](#fcu-fmu). More specifically, [PX4](#px4) or [ArduPilot](#ardupilot) software running on the FCU, for example during [HIL](#hil-hitl) simulation.

### Frame
- A spatial coordinate reference frame, especially as defined in [REP 103 and REP 105](#rep-rep-103-rep-105)
- An image frame, e.g. a single frame from a video stream from the [onboard](#onboard) [camera](#camera)

### GetFeatureInfo
A [WMS](#wms) operation for requesting non-[raster](#raster) features from [GIS](#gis) servers. Used in earlier versions of GISNav to fetch [DEM](#dem) values for specific points but no longer used.

### GetMap
A [WMS](#wms) operation for requesting [rasters](#raster) from [GIS](#gis) servers. Allows querying by arbitrary [bounding box](#bounding-box), as opposed to tile-based protocols such as [WMTS](#wms-wmts)

::: info Learn more
Learn more at [opengeospatial.github.io/e-learning/wms/text/operations.html#getmap](https://opengeospatial.github.io/e-learning/wms/text/operations.html#getmap)

:::

### Absolute position, global position
Horizontal and vertical position in a [CRS](#srs-crs) that specifies the location of the [vehicle](#vehicle) relative to an Earth-fixed reference frame (as opposed to a local reference frame)

::: info See also
[Relative position, local position](#relative-position)

:::

### Ground control, ground control software, ground control station
Ground control software that controls the [vehicle](#vehicle) through a remote radio link, using a protocol such as [MAVLink](#mavlink).

### Ground track
The [vehicle](#vehicle) flight path projected to the ground directly below the vehicle in the direction of [nadir](#nadir).

### Home
[Vehicle] home position into which it will typically automatically return once it's [flight mission](#mission) is complete. Not necessarily the same as the [EKF](#ekf) local origin.

### Image
- A [Docker](#docker) image
- A single image frame from the [onboard](#onboard) [camera](#camera)

::: warning
Not to be confused with [Orthoimage or imagery](#orthoimagery-imagery-orthoimage-orthophoto)

:::

### Launch, launch test
Launching using the [ROS](#ros-ros-2) launch system, ROS launch tests.

### Relative position, local position
Horizontal and vertical position that specifies the location of the [vehicle](#vehicle) relative to
a local reference frame.

::: info See also
[Absolute position, global position](#absolute-position-global-position)

:::

### Map, `map`
- One of the world-fixed [ROS](#ros-ros-2) coordinate [frames](#frame) that are defined in [REP 105](#rep-rep-103-rep-105). In GISNav the `map` frame is defined by [MAVROS](#mavros).
- A [raster](#raster) retrieved from a [GIS](#gis) server. Generic term that could e.g., mean [orthoimagery](#orthoimagery-imagery-orthoimage-orthophoto) or [DEMs](#dem) depending on context.

### Match, matching
Keypoint matching in the context of camera [pose](#pose) estimation between two images.

::: info See also
[PnP](#pnp-pnp)
:::

### Message
A [ROS](#ros-ros-2) message.

### Middleware
A software application that facilitates communication between other software applications (by transmitting data between them). More specifically, [MAVROS](#mavros), [micro-ROS Agent](#micro-ros-agent) or [GScam](#gscam).

### Mission, mission mode
- A flight mission, typically a file uploaded to a [GCS](#gcs) which then sends the appropriate commands to the [vehicle](#vehicle) to execute.
- [PX4](#px4) Mission [mode](#mode)

### Mode
[Autopilot](#autopilot) flight mode

### Model
- A machine learning model or neural [network](#network). In GISNav used specifically for [camera](#camera) [pose](#pose) estimation.
- A [Gazebo](#gazebo) model, more specifically a [vehicle](#vehicle) model

### Module
A [Python](#python) module.

### Nadir
Direction pointing directly down from the [vehicle](#vehicle) (opposed to [zenith](#zenith)). Does not mean down relative to vehicle body but rather the direction of the force of gravity.

### Navigation filter
An algorithm implemented by the [FMU](#fmu) that is responsible for determining the overall state of the [vehicle](#vehicle) based on fusing available sensor inputs and past states.

::: info
[EKF](#ekf) is one commonly used algorithm and is often used interchangeably to describe the navigation filter, even if the navigation filter does not use EKF.
:::

### Network
A neural network (a machine learning [model](#model)), such as SuperGlue or [LoFTR](#loftr)

### Node
A [ROS](#ros-ros-2) node.

### Notebook
A [Jupyter](#jupyter-jupyterlab) notebook.

### Offboard
Anything that is not [onboard](#onboard). More specifically any computer (e.g., running the [GCS](#GCS)) that is not carried onboard the [vehicle](#vehicle) and therefore does not draw power from the vehicle power source.

::: warning Warning: Conflicting definition in autopilot context
More narrow and conflicting definition that is relevant for autopilots running on [FCU](#fcu-fmu): Any software that is not running on the FCU - even if it is carried "onboard" the vehicle (e.g. "offboard control" when controlling the vehicle from software running on the [companion](#companion) computer).
:::

### Off-vehicle
Term used by at least [PX4](#px4) in a similar way GISNav currently uses the term [offboard](#offboard).

### Onboard
Anything carried by the [vehicle](#vehicle) that would draw power from its battery or power source, including the [FCU](#fcu-fmu) and the [companion computer](#companion-companion-computer).

::: warning Warning: Conflicting definition in autopilot context
More narrow and conflicting definition that is relevant for autopilots running on [FCU](#fcu-fmu): Any software that is running on the FCU.

:::

### On-vehicle
Term used by at least [PX4](#px4) in a similar way GISNav currently uses the term [onboard](#onboard).

### Orientation, attitude
[Vehicle](#vehicle) or [camera](#orientation) orientation (attitude) in 3D space, typically represented by a [quaternion](#quaternion).

::: info See also
[RPY](#rpy) for Euler angle representation
:::

### Origin

Origin of any reference [frame](#frame)

### Orthoimagery, imagery, orthoimage, orthophoto
- Orthorectified high-resolution geographic imagery stored in [GIS](#gis)
- An orthorectified high-resolution image of a location on Earth for which the [bounding box](#bounding-box) is known, retrieved from a GIS server.

::: info Learn more
Learn more at [en.wikipedia.org/wiki/Orthophoto](https://en.wikipedia.org/wiki/Orthophoto)
:::

### Query, query image
In a [pose](#pose) estimation context, the image frame from the [camera](#camera), to be compared to the [reference](#reference) image. The reference image may be an earlier image from the same camera when doing [VO](#vo), or an [orthoimage](#orthoimagery-imagery-orthoimage-orthophoto).

### Package
- A [ROS](#ros-ros-2) (colcon) package
- A [Python](#python) package

### Parameter
Most likely one of these:

- A [ROS 2](#ros-ros-2) parameter
- A [PX4](#px4) parameter
- An [ArduPilot](#ardupilot) parameter

### Perspective-n-Point, PnP
A problem in computer vision where a camera [pose](#pose) is estimated from 2D image to 3D [world](#world) coordinate point correspondences.

::: info Learn more
Learn more at [docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html](https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html)
:::

### Pose
A spatial pose in three dimensions including [position](#position) and [orientation](#orientation).

### Position
- A 3D [global position](#absolute-position-global-position)
- A 3D [local position](#relative-position-local-position)

### Publish, publisher
A [ROS](#ros-ros-2) publisher, to publish a ROS [message](#message).

### Quaternion
A 4-tuple describing [orientation](#orientation-attitude) in 3D space. Avoids the gimbal lock problem that comes when using Euler angles. Should be in (x, y, z, w) order unless otherwise defined.

:::info See also
[RPY](#rpy) for Euler angle representation of orientation
:::

### Raster
A rasterized image retrieved from a [GIS](#gis) server, as opposed to a vectorized image. Used exclusively for geographical imagery, not e.g., for an image from the [camera](#camera).

### Recipe
A [Makefile](#make-make) recipe

### Reference, reference image, reference raster
In a pose estimation context, the [orthoimage](#orthoimagery-imagery-orthoimage-orthophoto) frame from the [GIS](#gis) server, to be compared to the [query image](#query-query-image), or an earlier frame from the same camera if doing [visual odometry](#vo).

### Robot

Used to refer to the [vehicle](#vehicle), typically in a [ROS](#ros-ros-2) context. In GISNav the only kind of robot is the UAV, so this is synonymous with [vehicle](#vehicle).

### Rotation

Most likely rotation of the [reference raster](#reference-reference-image-reference-raster) when aligning it with the [vehicle](#heading). This is done in [pose](#pose) estimation because the neural [networks](#network) are not assumed to be rotation agnostic.


### Service
- A [Docker Compose](#docker-compose) service
- A [ROS](#ros-ros-2) service
- A [systemd](#systemd) service

### Service orchestration
Deploying and managing [Docker Compose](#docker-compose) [services](#service) that constitute a GISNav [deployment](#deploy-deployment). Currently done using [Make](#make-make) (Makefiles).

### Subscribe, subscriber, subscription
A [ROS](#ros-ros-2) subscription, to subscribe to a ROS [topic](#topic).

### Target
A [Makefile](#make-make) target

### Test
Currently the following kinds of tests are recognized:

- A unit test
- A [ROS](#ros-ros-2) launch test
- A simulation ([SITL](#sitl) or [HIL](#hil-hitl)) test

The static analysis in the git pre-commit hooks might also sometimes be referred to as "testing".

### Topic
A [ROS](#ros-ros-2) topic.

### Vehicle
The [UAV](#uav) that uses GISNav for visual navigation. Can e.g., be a quadcopter or fixed-wing aircraft. This term is preferred by autopilot software like [PX4](#px4) and [ArduPilot](#ardupilot), while the more generic term [robot](#robot) is often used in the [ROS](#ros-ros-2) context.

### World, world coordinates, world coordinate system
- In the [PNP](#pnp-pnp) problem context, the coordinate system of the [reference image](#reference-reference-image-reference-raster) including the z-axis used to represent ground [elevation](#elevation).
- A [Gazebo](#gazebo) world.

### Zenith
Direction pointing directly up from the [vehicle](#vehicle) (opposed to [nadir](#nadir)). Does not mean up relative to vehicle body but rather the direction opposite to the force of gravity.

## Abbreviations

### BBox
Bounding box

### `cmp`
Companion computer

### Coords
Coordinates

### Dev
Development

### Diff, `diff`
Difference

Also `git diff` and `diff` command line utilities to show differences in git repositories and files respectively.

### Distro
Distribution, a Linux distribution such as Debian

### `gnc`
GISNav CLI

### Img
Image

### Msg
Message

### NSH, `nsh`
NuttShell, Apache NuttX shell

### Qry
Query

### Ref
Reference

### Sim, `sim`
Simulation

### TTY, `tty`, `pty`
Teletypewriter, pseudo-TTY.

Input devices on Linux.

## Acronyms

### AGL
Altitude or Elevation Above Ground Level

### AMSL
Altitude or Elevation Above Mean Sea Level

### API
Application Programming Interface

### AWS
Amazon Web Services

### CI
Continuous Integration

### SRS, CRS
Spatial Reference System, Coordinate Reference System

### CV
Computer Vision

### DEM
Digital Elevation Model

### DNS
Domain Name System

### ECEF
Earth-Centered, Earth-Fixed (coordinate frame)

E.g. `earth` frame as defined in [REP 105](#rep-rep-103-rep-105)

### EKF
Extended Kalman Filter

::: info See also
[Navigation filter](#navigation-filter)
:::

### ENU
East-North-Up coordinate system

::: info
Up in the direction of [zenith](#zenith).
:::

### EOL
End-of-life, e.g., in context of [ROS](#ros-ros-2) distributions that are no longer officially supported.

### FCU, FMU
Flight Control Unit / Flight Management Unit. For example, [Pixhawk](#pixhawk).

### FOSS
Free and Open Source Software

::: info See also
[Free and open-source software](https://en.wikipedia.org/wiki/Free_and_open-source_software)
:::

### FoV / FOV
Field Of View

### FRD
Front-Right-Down coordinate system.

::: info
Down relative to the vehicle body, not [nadir](#nadir).
:::

### GCS
Ground Control Station

### GHCR
GitHub Container Registry

### GIS
Geographic Information System

### GML
Geography Markup Language

### GNSS, GPS
Global Navigation Satellite System / Global Positioning System

### GPU
Graphics Processing Unit

### GUI
Graphical User Interface

### HIL, HITL
Hardware In The Loop simulation

### IDE
Integrated/Interactive Development Environment

### NAIP
National Agriculture Imagery Program

::: info See also
[USGS NAIP website](https://www.usgs.gov/centers/eros/science/usgs-eros-archive-aerial-photography-national-agriculture-imagery-program-naip)
:::

### NED
North-East-Down coordinate system

::: info
Down in the direction of [nadir](#nadir).
:::

### NMEA, NMEA 0183
Communication protocol for GPS receivers

### OGC
Open Geospatial Consortium: [OGC](https://www.ogc.org/)

### OS
Operating System

### OSM
OpenStreetMap

### PnP, PNP
[Perspective-n-Point](#perspective-n-point-pnp) problem

### PR
Pull Request

### QGC
QGroundControl

### RDP
Remote Desktop Protocol

### REP, REP 103, REP 105
ROS Enhancement Proposal

- [REP 103](https://www.ros.org/reps/rep-0103.html)
- [REP 105](https://www.ros.org/reps/rep-0104.html)

### RPY
Roll, pitch, yaw. Euler angle representation of [attitude](#orientation-attitude) which suffers from gimbal lock, unlike [quaternions](#quaternion)

### SBC
Single Board Computer

### SCP, `scp`
Secure Copy Protocol

### SITL
Software In The Loop simulation

### SQL
Structured Query Language: [SQL](https://en.wikipedia.org/wiki/SQL)

### TCP, TCP/IP, IP
Transmission Control Protocol/Internet Protocol

### ToU / TOU
Terms of Use

### UAV
Unmanned Aerial Vehicle

### UDP
User Datagram Protocol: [User Datagram Protocol](https://en.wikipedia.org/wiki/User_Datagram_Protocol)

### USGS
United States Geological Survey

### VNC
Virtual Network Computing

### VO
Visual Odometry

### WGS, WGS 84
A World Geodetic System coordinate system: [World Geodetic System](https://en.wikipedia.org/wiki/World_Geodetic_System)

### WMS, WMTS
Web Map Service / Web Map Tile Service, two separate OGC developed communication protocols. WMS allows querying by arbitrary bounding box while WMTS returns pre-computed tiles in a standardized grid.

::: info See also
- [WMS Standard](https://www.ogc.org/standards/wms)
- [WMTS Standard](https://www.ogc.org/standards/wmts)

:::

## Proper Names

### ArduPilot
ArduPilot open source autopilot: [ardupilot.org](https://ardupilot.org/)

### `colcon`
A build automation tool used by ROS 2: [colcon.readthedocs.io/en/released](https://colcon.readthedocs.io/en/released/)

### CUDA
NVIDIA parallel computing platform: [developer.nvidia.com/cuda-zone](https://developer.nvidia.com/cuda-zone)

### DDS / Data Distribution Service
A middleware protocol and standard: [dds-foundation.org](https://www.dds-foundation.org/)

### Debian
A Linux distribution: [debian.org/](https://www.debian.org/)

### Docker
Software containerization tool: [docker.com](https://www.docker.com/)

### Docker Compose
Tool for defining and running multi-container Docker applications: [docs.docker.com/compose](https://docs.docker.com/compose/)

### FileGator
A FOSS self-hosted file management application: [docs.filegator.io](https://docs.filegator.io/)

### Gazebo
Simulation software: [gazebosim.org/home](https://gazebosim.org/home)

### GDAL, Geospatial Data Abstraction Library
Software library for handling geospatial data: [gdal.org](https://gdal.org/)

### GSCam, `gscam`
ROS GStreamer camera driver: [github.com/ros-drivers/gscam](https://github.com/ros-drivers/gscam)

### GStreamer
Open source multimedia framework: [gstreamer.freedesktop.org](https://gstreamer.freedesktop.org/)

### Nano, Jetson Nano
An NVIDIA Jetson Nano computer

### Jupyter, JupyterLab
A web-based IDE: [jupyter.org](https://jupyter.org/)

### Make, `make`
GNU Make, a build automation tool: [gnu.org/software/make](https://www.gnu.org/software/make/)

### MapServer
Open source GIS software: [mapserver.org](https://mapserver.org/)

### MAVLink
MAVLink (Micro Air Vehicle Link) protocol: [mavlink.io/en](https://mavlink.io/en/)

### MAVROS
An open source MAVLink to ROS middleware: [wiki.ros.org/mavros](http://wiki.ros.org/mavros)

### MAVSDK
MAVLink software development kit: [mavsdk.mavlink.io/main/en/index.html](https://mavsdk.mavlink.io/main/en/index.html)

### Mermaid, `mermaid.js`
A diagram scripting language: [mermaid.js.org](https://mermaid.js.org/)

### micro-ROS Agent
PX4-specific ROS middleware: [docs.px4.io/main/en/middleware/uxrce_dds.html](https://docs.px4.io/main/en/middleware/uxrce_dds.html)

### OpenCV, `cv2`
Open source computer vision software library: [opencv.org](https://opencv.org/).

`cv2` refers to the OpenCV Python bindings module.

### OpenStreetMap
Open source map of the world: [openstreetmap.org](https://www.openstreetmap.org/)

### Pixhawk
Hardware standard for open source autopilots: [pixhawk.org](https://pixhawk.org/)

### PostGIS
GIS extension for Postgres: [postgis.net](https://postgis.net/)

### Postgres
An SQL server: [postgresql.org](https://www.postgresql.org/)

### PX4
PX4 Autopilot: [px4.io](https://px4.io/)

### Python
A computer programming language: [python.org](https://www.python.org/)

### Raspberry Pi 5
A popular SBC: [raspberrypi.com/products/raspberry-pi-5](https://www.raspberrypi.com/products/raspberry-pi-5/)

### QEMU
A FOSS full-system emulator: [qemu.org](https://www.qemu.org/)

### QGIS
A GIS client (and server): [qgis.org/en/site](https://qgis.org/en/site/)

### QGroundControl
GCS software: [qgroundcontrol.com](http://qgroundcontrol.com/)

### ROS, ROS 2
Robot Operating System: [ros.org](https://www.ros.org/)

GISNav uses ROS 2.

### RViz
ROS 3D visualization software: [github.com/ros2/rviz](https://github.com/ros2/rviz)

### S3
[AWS](#aws) Simple Storage Service (S3): [aws.amazon.com/s3](https://aws.amazon.com/s3/)

### Sphinx
Documentation generation software: [sphinx-doc.org/en/master/](https://www.sphinx-doc.org/en/master/)

### systemd
A system and service manager for Linux: [systemd.io](https://systemd.io/)

### tf2
[ROS 2](#ros-ros-2) transformations library: [wiki.ros.org/tf2](http://wiki.ros.org/tf2)

::: info Not Tensorflow
Tensorflow is often referred to as TF or `tf` in code but GISNav does not use it - GISNav uses
[Torch](#torch-pytorch) instead.

:::

### Torch, PyTorch
An open source machine learning software library: [torch.ch](http://torch.ch/)

Python machine learning library based on Torch: [pytorch.org](https://pytorch.org/)

### Ubuntu
A Linux distribution, only supported OS for GISNav: [ubuntu.com](https://ubuntu.com/)

### VitePress
Static site generator software: [vitepress.dev](https://vitepress.dev/)

### VRT
GDAL Virtual Format (file format)

### X Server
Window system that comes with Ubuntu: [x.org/wiki](https://www.x.org/wiki/)

### YAML
A data serialization language: [yaml.org](https://yaml.org/)

## Other

### KSQL
ICAO airport code for San Carlos Airport in California (used as simulation environment in GISNav development and testing).
