Development Objectives
--------------------------------------------
GISNav demonstrates a map-based visual global positioning for airborne drones that complements and improves on
existing sensor fusion systems. It improves both local and global position and attitude estimate accuracy, and provides
backup global positioning for `GNSS <https://en.wikipedia.org/wiki/Satellite_navigation>`_-denied flight.

.. _Guiding Principles:
Guiding Principles
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The following principles are used as design guidance in GISNav development:

* Complement and improve - but do not replace - existing local and global position and attitude estimation systems
* The natural or primary application of map-based visual global positioning is complementing GNSS, while local position and attitude estimation or replacing GNSS (GNSS-denied flight) are secondary applications
* Prioritize maintainability and well-defined interfaces over premature optimization
* Support proven commercial off-the-shelf hardware platforms

.. _Constraints:
Constraints
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The `Guiding Principles`_ impose constraints on GISNav, namely:

* Currently GISNav is intended for simulation only
* Favorable operating terrain is strongly featured urban and semi-urban areas and traffic corridors (roads), not featureless natural terrain
* Monocular stabilized camera required
* Drone or UAV size, flight altitude or velocity constrained only to such degree that allows commercial GNSS receivers to work
* Focus on good flight conditions - reasonable assumption for most commercial use cases which is where most develoment effort should be, niche applications will follow

Development Focus
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Taking the `Constraints`_ into account, development focus should for example be in:

* ROS is baked in, but PX4 could be complemented by other flight control software options such as Ardupilot through Mavlink compatible interface
* Newer algorithms to improve accuracy, reliability or performance
* Making adoption easier for different kinds of hardware platforms or configurations
* SITL testing workbench development - have a way to fly premade flight plans in SITL simulation and automatically parse
the flight logs and compare against some thresholds to determine if the software passes the test.
