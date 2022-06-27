**************************************************
Contribute
**************************************************
All kinds of contributions are welcome, from raising issues with the software to software commits and pull requests
(PRs). Please see below for guidance and suggestions on how and where to contribute.


Issues & Improvement Suggestions
==================================================
Please take a look at the issues board on the Github page to see if someone already has raised a similar issue or
suggestion.


Development
==================================================
If you are interested in contributing to GISNav as a developer, you may consider the guidance provided in this section.
These bullet points should be taken more as suggestions on where to find "low-hanging fruit" rather than as
recommendations on what to do.

.. _Project Intention:

Project Intention
--------------------------------------------------
GISNav demonstrates a map-based visual global positioning for airborne drones that complements and improves on
existing sensor fusion systems. It improves both local and global position and attitude estimate accuracy, and provides
backup global positioning for `GNSS <https://en.wikipedia.org/wiki/Satellite_navigation>`_-denied flight.

.. _Guiding Principles:

Guiding Principles
--------------------------------------------------
The following principles have been used as design guidance when developing GISNav and that you can use to evaluate
whether a new feature could fit in:

* Complement and improve but do not replace

    A natural application of map-based visual global positioning is complementing (but not replacing) GPS (GNSS) as
    global position provider.

* Maintainability and well-defined interfaces over premature optimization

    Open source drone autopilot software as well as deep learning based image matching are under active development.
    Autonomous drone delivery services and other use scenarios are not well-developed.

* Build on proven open technology stack

    The future-proof approach is to support open hardware platforms and
    `FOSS <https://en.wikipedia.org/wiki/Free_and_open-source_software>`_ software with existing wide adoption

* Target future `commercial` use cases

    Future autonomous drone services will require precise and reliable navigation, especially within an urban or
    semi-urban environment. Vision is a key enabler when a drone needs to e.g. autonomously land on a small back yard.


.. _Constraints:

Constraints
--------------------------------------------------
The `Guiding Principles`_ impose constraints on GISNav, namely:

* Favorable operating terrain for map-based visual matching is strongly featured urban and semi-urban areas and traffic corridors (roads or similar infrastructure), not featureless natural terrain
* Monocular stabilized camera should be a sufficient hardware setup
* Drone or UAV size, flight altitude or velocity constrained only to such degree that allows commercial GNSS receivers to work
* Emphasis on good flight conditions is a reasonable assumption for most commercial use cases
* Open-source software (and hardware) with permissive licenses only
* ROS is baked in
* Mav

Development Focus
--------------------------------------------------
Taking the `Constraints`_ into account, development focus could for example be in:

* Easier adoption

    * Improving documentation

    * Developing pre-made Docker environments to speed up initial setup

    * Pre-made configurations for popular hardware platforms or autopilot technology stacks

    * Abstracting away current configuration quirks

    * PX4 could be complemented by other autopilot options such as Ardupilot through Mavlink compatible interface

* Improved estimation accuracy, reliability or performance

    * Newer, better pose estimation algorithms or neural networks

    * Using elevation (`DEMs <https://en.wikipedia.org/wiki/Digital_elevation_model>`_) or other data from the underlying GIS system to complement orthoimagery

        For example, a DEM could be used to plug in the z-coordinates of the object points for the
        :func:`cv2.solvePnPRansac` call that is currently used under the hood of :class:`.KeypointPoseEstimator`.

    * Support for stereo camera

* More testing

    * Better test coverage

    * SITL testing workbench development

        Have a way to fly premade flight plans in SITL and automatically parse the flight logs or import them into e.g.
        a Jupyter notebook for further analysis.

* Better customization

    * Have a way to re-initialize the dynamically loaded classes (:class:`.PoseEstimator`) at runtime to make it possible to swap in specialized neural nets for specific terrain via a ROS service

* Better maintainability

    * Move to a more distributed 'ROS native' system in the long term where current modules that are managed by the :class:`.BaseNode` are spun as independent ROS nodes if possible.

Commits & Pull Requests
--------------------------------------------------
Pull requests (PRs) are very much welcome! Please follow the
`feature branch workflow <https://www.atlassian.com/git/tutorials/comparing-workflows/feature-branch-workflow>`_ when
submitting your pull request.

* Take a look at the known issues or create one yourself for your PR before you start working so that others will also be aware of your pending work. You can also use it as an opportunity to get feedback on your idea before you commit to it further.

* If your PR fixes or implements an issue, please link the issue in your pull request

* In your commit messages, please describe not only *what* you have done, but *why* you have done it. This helps the reviewer understand your thought process faster.
