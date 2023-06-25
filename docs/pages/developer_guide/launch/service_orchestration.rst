Deploy using Makefile
____________________________________________________

The easiest way to deploy GISNav and its
:ref:`supporting Docker Compose services <Docker Compose>` is using :term:`Make`.
For example, to deploy the :ref:`Get Started` demonstration locally, use the
following command:

.. code-block:: bash
    :caption: Deploy GISNav mock GPS demo

    cd ~/colcon_ws/src/gisnav
    make -C docker demo

Make target naming scheme
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The ``Makefile`` defines a number of phony make targets corresponding to GISNav
deployment configurations. Generally, the make targets are named according to the
following scheme:

.. warning::
    If your :term:`offboard` computer is ``arm64`` based, the ``Makefile`` targets
    and the ``mapserver`` Docker image especially may not work.

.. code-block:: bash
    :caption: Make target naming scheme

    cd ~/colcon_ws/src/gisnav
    make -C docker <build>-<onboard/offboard>-<sitl/hil>-<middleware>-<dev/test>-<px4/ardupilot>

In the scheme, the various qualifiers determine the type of deployment:

- ``<build>``:  `(optional)` Designates whether the services should only be built but not run
- ``<onboard/offboard>``: Designates whether the target is intended for :term:`onboard` or :term:`offboard` use
- ``<sitl/hil>``: Determines whether the target is for :term:`SITL` or :term:`HIL` simulation
- ``<middleware>``: `(optional)` Indicates whether the target is an intermediate target consisting of middleware
- ``<dev/test>``: `(optional)` Designates whether the target is for development or testing
- ``<px4/ardupilot>``: Specifies whether the target is intended to be used with :term:`PX4` or :term:`ArduPilot` firmware

There are some constraints: for example the qualifier ``offboard`` can only occur
together with the ``sitl`` qualifier.

Local development
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Use the following command to run GISNav supporting services when developing
locally (i.e. running all supporting services but not GISNav itself inside a
:term:`Docker` container):

.. note::
    If the :term:`Gazebo` and :term:`QGroundControl` windows do not appear on
    your screen you may need to expose your ``xhost`` to your :term:`Docker`
    containers (see e.g. `ROS GUI Tutorial <http://wiki.ros.org/docker/Tutorials/GUI>`_):

    .. code-block:: bash
        :caption: Expose xhost

        for containerId in $(docker ps -f name=docker -aq); do
            xhost +local:$(docker inspect --format='{{ .Config.Hostname }}' $containerId)
        done

.. tab-set::

    .. tab-item:: PX4
        :selected:

        .. code-block:: bash
            :caption: Deploy GISNav supporting services for local development with PX4

            cd ~/colcon_ws/src/gisnav
            make -C docker offboard-sitl-dev-px4

    .. tab-item:: ArduPilot

        .. code-block:: bash
            :caption: Deploy GISNav supporting services for local development with ArduPilot

            cd ~/colcon_ws/src/gisnav
            make -C docker offboard-sitl-dev-ardupilot

Offboard SITL testing
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Use the following command to run GISNav supporting services :term:`offboard`
in headless mode for automated :term:`SITL` testing:

.. note::
    In headless mode you will not see a :term:`GCS` :term:`GUI` so you will
    need e.g. :term:`MAVSDK` to control the :term:`vehicle`.

.. tab-set::

    .. tab-item:: PX4
        :selected:

        .. code-block:: bash
            :caption: Deploy GISNav supporting services for automated SITL testing with PX4

            cd ~/colcon_ws/src/gisnav
            make -C docker offboard-sitl-test-px4

    .. tab-item:: ArduPilot

        .. code-block:: bash
            :caption: Deploy GISNav supporting services for automated SITL testing with ArduPilot

            cd ~/colcon_ws/src/gisnav
            make -C docker offboard-sitl-test-ardupilot

Full list of make targets
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

To see the full list of supported make targets you can inspect the source code:

.. dropdown:: See Makefile source code
    :icon: code

    .. literalinclude:: ../../../../docker/Makefile
        :caption: Docker Compose service orchestration with Make
        :language: makefile
