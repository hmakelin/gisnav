Deploy using Makefile
____________________________________________________

The easiest way to deploy GISNav and its
:ref:`supporting Docker Compose services <Deploy with Docker Compose>` is using
:term:`Make`. For example, to deploy the :ref:`Get Started` demonstration locally,
ensure you have the :ref:`prerequisites <Prerequisites>` installed and then use
the following command:

.. code-block:: bash
    :caption: Deploy GISNav mock GPS demo

    cd ~/colcon_ws/src/gisnav
    make -C docker demo

Prerequisites
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. include:: _prerequisites_docker.rst

.. include:: _prerequisites_gisnav.rst

Make target naming scheme
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The ``docker/Makefile`` defines a number of phony make targets corresponding to GISNav
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
    and the ``mapserver`` Docker image especially may not work.

Deploy for local development
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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

.. include:: _launch_gisnav_with_ros2_launch.rst

Deploy for offboard SITL testing
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Use the following command to run GISNav supporting services :term:`offboard`
in headless mode for automated :term:`SITL` testing:

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
