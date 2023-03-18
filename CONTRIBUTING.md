# Contribute

All kinds of contributions are welcome, from raising issues and improvement suggestions with the software to software
commits and pull requests (PRs). Please see below for guidance and suggestions on how to contribute.

## Issues and improvement suggestions

Please take a look at the [open issues][1] to see if someone has already raised a similar issue or suggestion on which
you could add your own comments. If not, use the provided templates to submit a new one.

[1]: https://github.com/hmakelin/gisnav/issues

## Pull requests and software commits

> **Warning**
> By contributing to this repository, you agree that **your contributions will be licensed under the repository's MIT
> license**.

All changes to the software are made through [pull requests][2]. Please pay attention to the following before you
submit your pull request:

* See [Style Guide for Python Code][4] for general code style guidelines.

* If your pull request is related to an existing [open issue][1], please mention it in your pull request description.

* If your pull request addresses a new issue or improvement, consider posting it on the [open issues][1] page before
  you start working on it so that others will also be aware of your pending work.

* Consider creating a [draft pull request][5] to get early feedback on your work before you commit to it further.

* In your pull request, please describe not only *what* you have done, but also *why* you have done it. This helps the
  reviewer understand your thought process faster.

* If your pull request makes use of 3rd party software, please ensure that it is MIT license compatible.

[2]: https://docs.github.com/en/pull-requests

[3]: https://www.conventionalcommits.org/

[4]: https://peps.python.org/pep-0008/

[5]: https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/about-pull-requests#draft-pull-requests

## Project Scope

The following table provides an overview of what GISNav aims to be. You can use it as guidance for determining whether your contributions would fit in the project scope.

| No. | Objective |
| --- | --- |
| 1. | **Precise visual navigation** <br> GISNav should provide a precise global position for drones by visually comparing frames from the drone's nadir-facing camera to a map of the drone's approximate global position retrieved from an onboard GIS system. |
| 2. | **Map-based navigation** <br> GISNav should enable map-based navigation for drones, allowing them to navigate based on visual references to the surrounding environment. |
| 3. | **GNSS-free navigation** <br> GISNav should provide a substitute for GPS/GNSS navigation in cases where GPS/GNSS signals are unavailable or unreliable. |
| 4. | **Reliable performance** <br> GISNav should be reliable and perform well in a variety of environments and conditions. |
| 5. | **User-friendly setup and configuration** <br> GISNav should be easy to setup, configure and integrate with supported autopilot platforms. |
| 6. | **Compatibility with multiple Autopilot systems** <br> GISNav should be compatible with multiple Autopilot systems, including PX4, Ardupilot, and Mavlink. |
| 7. | **Integration with ROS** <br> GISNav should integrate with ROS (Robot Operating System), a popular framework for building robotic systems. |
| 8. | **Dockerization** <br> GISNav should be designed to run in Docker containers to aid in deployment and management. |
| 9. | **Support for Nvidia Jetson** <br> GISNav should be optimized to run on the Nvidia Jetson platform, which is commonly used in drone development. |

In summary, GISNav aims to provide precise visual navigation, map-based navigation, and GNSS-free navigation for drones, and to be reliable, user-friendly, compatible with multiple autopilot systems, and optimized for the Nvidia Jetson platform. It should also integrate with ROS and be designed to run in Docker containers.
