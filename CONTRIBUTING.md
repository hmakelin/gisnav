# Contribute

All kinds of contributions are welcome, from raising issues and improvement suggestions with the software to software 
commits and pull requests (PRs). Please see below for guidance and suggestions on how and where to contribute.

## Issues and improvement suggestions

Please take a look at the [open issues][1] to see if someone has already raised a similar issue or suggestion on which 
you could add your own comments. If not, use the provided templates to submit a new one.

[1]: https://github.com/hmakelin/gisnav/issues

## Pull requests and software commits

> **Note**
> By contributing to this repository, you agree that **your contributions will be licensed under the repository's MIT 
> license**.

All changes to the software are made through [pull requests][2]. Please pay attention to the following before you 
submit your pull request:

* If your pull request is related to an existing [open issue][1], please mention it in your pull request description.

* If your pull request addresses a new issue or improvement, consider posting it on the [open issues][1] page before 
  you start working on it so that others will also be aware of your pending work.

* If your pull request makes use of 3rd party software, please ensure that it is MIT license compatible.

* Consider creating a [draft pull request][3] to get early feedback on your work before you commit to it further.

* In your pull request, please describe not only *what* you have done, but also *why* you have done it. This helps the 
  reviewer understand your thought process faster.

## Improvements and new features

If you are planning to add a new feature or improve an existing feature of the software, please ensure that your 
contribution is aligned with GISNav's **development focus**. Currently the **development focus** is to make the software:

* **Easier to use**, for example by...
  * ...improving documentation
  * ...streamlining the public API
  * ...smoothing out configuration quirks
* **Better tested**, for example by...
  * ...improving unit test coverage
  * ...adding a (better) testing framework
* **Support relevant technology stacks**, for example by...
  * ...adding support for Ardupilot
  * ...adding an adapter for TensorFlow-based neural networks
* **More accurate**, for example by...
  * ...adding a new state-of-the art pose estimation algorithm
  * ...making use of digital-elevation maps (DEM) to retrieve elevation of ground plane
  * ...making use of the [OSM buildings][4] database to remove assumption of planar ground surface

> **Note**
> Please consider the examples in the second level bullet points as *suggestions* on where to find low-hanging fruit rather than 
> as *recommendations* on what to do.

[2]: https://docs.github.com/en/pull-requests

[3]: https://docs.github.com/en/pull-requests/collaborating-with-pull-requests/proposing-changes-to-your-work-with-pull-requests/about-pull-requests#draft-pull-requests

[4]: https://osmbuildings.org/data/