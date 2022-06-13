Development Objectives
--------------------------------------------
GISNav demonstrates a map-based visual global positioning for airborne drones that complements and improves on
existing sensor fusion systems. It improves both local and global position and attitude estimate accuracy, and provides
backup global positioning for `GNSS <https://en.wikipedia.org/wiki/Satellite_navigation>`_-denied flight.

.. _Guiding Principles:

Guiding Principles
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The following principles have been used as design guidance when developing GISNav:

* Complement and improve but do not replace

    The natural or primary application of map-based visual global positioning is complementing GNSS, while local
    position and attitude estimation and replacing GNSS completely (GNSS-denied flight) are secondary applications.

* Maintainability and well-defined interfaces over premature optimization

    Proven and established solutions do not yet exist, and the state-of-the-art for deep learning based image matching
    especially is fast-moving.

* Support proven commercial off-the-shelf hardware platforms and `FOSS <https://en.wikipedia.org/wiki/Free_and_open-source_software>`_ software

    Best bet is to work on open platforms with the widest adoption, to ensure development can continue far into the future from a stable foundation.

.. _Constraints:

Constraints
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The `Guiding Principles`_ impose constraints on GISNav, namely:

* Favorable operating terrain is strongly featured urban and semi-urban areas and traffic corridors (roads), not featureless natural terrain
* Monocular stabilized camera should be sufficient
* Drone or UAV size, flight altitude or velocity constrained only to such degree that allows commercial GNSS receivers to work
* Focus on good flight conditions - reasonable assumption for most commercial use cases which is where most develoment effort should be, niche applications will follow
* Open-source software and with permissive licenses only

Development Focus
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Taking the `Constraints`_ into account, development focus should for example be in:

* ROS is baked in, but PX4 could be complemented by other flight control software options such as Ardupilot through Mavlink compatible interface
* Newer pose estimation algorithms to improve accuracy, reliability or performance
* Using elevation or other data from the underlying GIS system to complement ortho-images to improve position and attitude estimates
* Making adoption easier with pre-made configurations for popular hardware platforms
* SITL testing workbench development - have a way to fly premade flight plans in SITL simulation and automatically parse the flight logs and compare against some pre-set thresholds to determine if the software passes the test.
