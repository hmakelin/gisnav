Deploy using Makefile
____________________________________________________
The easiest way to deploy GISNav and its :ref:`supporting Docker Compose services
<Deploy with Docker Compose>` is using :term:`Make`. A :ref:`Target naming scheme`
has been devised to describe the different types of available deployments. Read
below how the scheme works and try out some of the example commands to understand
how the GISNav's supporting services are orchestrated.

Prerequisites
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. include:: ../_shared/prerequisites/docker.rst

.. include:: ../_shared/prerequisites/gisnav.rst

Target naming scheme
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The ``docker/Makefile`` defines a number of phony Make targets corresponding to GISNav
deployment configurations. Generally, the make targets are named according to the
following scheme:

.. code-block:: bash
    :caption: Make target naming scheme

    cd ~/colcon_ws/src/gisnav
    make -C docker <build/create/up>-<onboard/offboard>-<sitl/hil>-<middleware>-<dev/test>-<px4/ardupilot>

In the naming scheme, the various qualifiers determine the type of deployment:

.. list-table::
    :widths: 25 75
    :header-rows: 1

    * - Qualifier
      - Description
    * - ``build/create/up``
      - Designates what Docker Compose operation should be applied to the services
    * - ``onboard/offboard``
      - Designates whether the target is intended for :term:`onboard` or :term:`offboard` use
    * - ``sitl/hil``
      - Designates whether the target is for :term:`SITL` or :term:`HIL` simulation
    * - ``middleware``
      - *Optional:* Indicates that the target is an intermediate target consisting of middleware
    * - ``dev/test``
      - *Optional:* Indicates that the target is intended for development or testing
    * - ``px4/ardupilot``
      - Designates whether the target is intended to be used with :term:`PX4` or
        :term:`ArduPilot` firmware

There are some constraints: for example the qualifier ``offboard`` can only occur
together with the ``sitl`` qualifier. For an exhaustive list of supported
combinations of qualifiers, see the source code:

.. dropdown:: See Makefile source code
    :icon: code

    .. literalinclude:: ../../../../docker/Makefile
        :caption: Docker Compose service orchestration with Make
        :language: makefile

.. warning::
    If your :term:`offboard` computer is ``arm64`` based, the ``Makefile`` targets
    and the ``mapserver`` Docker image especially may not work. For now, you
    should look at the :ref:`Deploy with Docker Compose` page if you are
    interested in offboard simulation on ``arm64`` systems.

Building, creating and running services
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The deployments consist of multiple services that can take a long time to build.
Therefore it is recommended to do the building of the images and creating of the
containers separately from the running so that all services start at the same
time:

.. code-block:: bash
    :caption: Build images

    cd ~/colcon_ws/src/gisnav
    make -C docker build-offboard-sitl-test-dev-px4

.. code-block:: bash
    :caption: Create containers

    make -C docker create-offboard-sitl-test-dev-px4

You will need to expose your X server to your GUI containers. The ``expose-xhost``
recipe will expose your X server only to the containers that need access:

.. code-block:: bash
    :caption: Expose X server to containers

    make -C docker expose-xhost

Once you have successfully run a container once, subsequent starts of the same
container will be much faster, especially for the SITL simulation services like
``px4``. You should therefore probably not create new containers with ``up`` and
``down``, but rather restart the existing ones to have all services start very
quickly:

.. code-block:: bash
    :caption: Start containers

    make start

.. code-block:: bash
    :caption: Stop containers

    make stop


Example deployments
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

This section lists some of commonly needed deployments that you can use to get
started.

Mock GPS demo
********************************************
To deploy the :ref:`Get Started` demonstration locally, run the following command.

.. code-block:: bash
    :caption: Deploy GISNav mock GPS demo

    cd ~/colcon_ws/src/gisnav
    make -C docker demo

.. note::
    Building all the constituent services can take a very long time, and some
    services may already start before others have finished building on the
    first attempt. Instead of using the one-liner above, you might want to try
    :ref:`Building, creating and running services` successively.

Local development
********************************************
Use the following command to deploy GISNav supporting services when developing
locally (i.e. running all supporting services but not GISNav itself inside a
Docker container):

.. tab-set::

    .. tab-item:: PX4
        :selected:

        .. code-block:: bash
            :caption: Deploy GISNav supporting services for local development

            cd ~/colcon_ws/src/gisnav
            make -C docker up-offboard-sitl-dev-px4

    .. tab-item:: ArduPilot

        .. code-block:: bash
            :caption: Deploy GISNav supporting services for local development

            cd ~/colcon_ws/src/gisnav
            make -C docker up-offboard-sitl-dev-ardupilot

After the supporting services are running, you will probably want to try
running your :ref:`local installation <Install locally>` of GISNav to
test your latest changes to the source code:

.. include:: ../_shared/launch_gisnav_with_ros2_launch.rst

Offboard SITL
********************************************
Use the following command to run GISNav supporting services :term:`offboard`
in headless mode for automated :term:`SITL` testing.

.. tab-set::

    .. tab-item:: PX4
        :selected:

        .. code-block:: bash
            :caption: Deploy GISNav supporting services for automated SITL testing

            cd ~/colcon_ws/src/gisnav
            make -C docker up-offboard-sitl-test-px4

    .. tab-item:: ArduPilot

        .. code-block:: bash
            :caption: Deploy GISNav supporting services for automated SITL testing

            cd ~/colcon_ws/src/gisnav
            make -C docker up-offboard-sitl-test-ardupilot

.. note::
    In headless mode you will not see a :term:`GCS` :term:`GUI` so you will
    need e.g. :term:`MAVSDK` to control the :term:`vehicle`.
