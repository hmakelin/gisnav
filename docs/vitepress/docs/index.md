---
# https://vitepress.dev/reference/default-theme-home-page
layout: home

hero:
  name: "GISNav"
  text: "Optical terrain-matching for UAVs"
  tagline: "A ROS 2 package that determines UAV global position by aligning real-time video with maps from an onboard GIS server."
  actions:
    - theme: brand
      text: "Demo"
      link: "/README"
    - theme: alt
      text: "Guide"
      link: "/introduction"
    - theme: alt
      text: "API Reference"
      link: "reference/index"

#features:
#  - title: "GNSS-free navigation"
#    details: "Operates independent of GNSS systems such as GPS, providing secondary navigation in environments where GNSS signals are weak or unavailable."
#  - title: "Offline navigation"
#    details: "Designed to function without any external connections, ensuring continuous operation even in remote or network-restricted areas."
#  - title: "Optical terrain-matching"
#    details: "Provides a precise global position by visually comparing frames from the vehicle's nadir-facing camera to a map of the UAVs approximate global position retrieved from an onboard GIS server."
#  - title: "FOSS with MIT License"
#    details: "Open source under the permissive MIT license, allowing for free use, modification, and distribution."
#  - title: "Monocular Camera Compatibility"
#    details: "Compatible with any standard monocular camera, facilitating easy adoption and integration with existing equipment, without requiring specialized hardware."
#  - title: "MAVLink, NMEA and uORB Protocols"
#    details: "Supports integration with popular autopilot systems like PX4 and ArduPilot through MAVLink, NMEA and uORB protocols."
#  - title: "Secondary GPS Over Serial Port (NMEA)"
#    details: "Functions as a reliable secondary GPS, easily integrating over serial connections without the need for firmware modifications."
#  - title: "Simulation with Gazebo"
#    details: "Includes support for Gazebo simulations, enabling developers to test and refine drone operations in a fully controlled virtual environment."
#  - title: "ROS 2 Integration"
#    details: "Integrates with the ROS 2 ecosystem, providing easy extensibility."

---

<!--@include: ./shared/warning-simulation-use-only.md-->
