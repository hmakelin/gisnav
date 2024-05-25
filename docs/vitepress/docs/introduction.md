# Introduction to GISNav

<!--@include: ./shared/warning-simulation-use-only.md-->

GISNav provides a simulation environment for map-based GNSS-free navigation for UAVs.

GISNav uses visual comparisons between frames captured by the vehicle's camera along with orthoimagery and digital elevation models retrieved from an onboard GIS server to compute an estimate of the vehicle's global position. This provides complementary offline navigation in environments where traditional GNSS signals or other online navigation may be unavailable or unreliable.

## What GISNav consists of

1. [**Debian distributable**](/install-from-debian-package): GISNav is available as a Debian package to provide a streamlined onboarding experience. This allows for easy installation and management of GISNav on Debian-based systems.

2. [**Containerized simulation environment**](/deploy-with-docker-compose): Under the hood, GISNav consists of a suite of Docker Compose services. This containerized approach ensures that all dependencies are properly isolated, allowing for consistent and reproducible deployments across different environments. The Docker Compose services include all necessary components for GISNav to function, from the GIS server and sample maps to the deep learning model used for keypoint matching.

3. **ROS 2 package**: The core of GISNav is a ROS 2 package called `gisnav`, implementing the main application logic and providing integration with the ROS ecosystem. This ensures compatibility with a wide range of existing robotics applications.

4. [**Command Line Interface**](/gisnav-cli): The GISNav Debian package comes with a simple command line interface (CLI) tool called `gnc` that allows users to easily control the system. The CLI provides commands for setting up the environment, starting and stopping services, and monitoring the system's status.

5. **systemd service**: The Debian package also includes the `gisnav.service` systemd service that ensures all required services are automatically started on system startup.

6. [**Captive admin portal**](/admin-portal): GISNav includes a web-based captive admin portal for managing and configuring the system, including managing the onboard GIS data with no prior knowledge of GIS tools required.

7. **Documentation**: Detailed user and developer documentation is provided to facilitate the setup, usage, and development of GISNav. This includes installation guides, configuration instructions, and development resources to support both end users and contributors.

## How GISNav works

At its core, GISNav utilizes PyTorch and deep learning for keypoint matching and OpenCV for solving the resulting Perspective-n-Point (PnP) problem. This process involves:
- **Uploading orthoimagery**: High-resolution aerial and satellite aerial imagery is [available via both national agencies and commercial providers](/setup-gis-server#orthoimagery-and-dems). The maps are uploaded onto an onboard computer.
- **Matching keypoints**: Frames from the vehicle’s camera are analyzed to detect and match keypoints with the preloaded orthoimagery.
- **Solving PnP problem**: The matched keypoints are then used to solve the PnP problem, determining the vehicle's global pose (geopose) relative to the orthoimage.
- **Sending mock GPS messages**: Once the geopose is estimated, GISNav generates mock GPS navigation messages that can be fed into the vehicle's autopilot software, such as PX4 and ArduPilot. Because GISNav masquerades as an ordinary GPS, no custom autopilot firmware tinkering should be needed for the integration.

## Integration with autopilot software

GISNav integrates with autopilot software including PX4 and ArduPilot. This is achieved through the generation of mock GPS messages that are compatible with the NMEA and PX4's uORB protocols.

::: info ArduPilot support unmaintained
ArduPilot simulation needs significant rework to get working again

:::

## Source code

The source code for GISNav is open source and can be accessed on GitHub at [github.com/hmakelin/gisnav](https://github.com/hmakelin/gisnav). Feedback is welcome to help improve the project.

## Getting started

To get started with GISNav, follow the installation guides provided in the documentation, choose your preferred deployment method (ROS 2 package, Debian distributable, or Docker Compose), and configure your system according to your specific needs.
