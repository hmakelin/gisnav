Deploy with Docker Compose
____________________________________________________

GISNav uses :term:`Docker Compose` to define the services that constitute its
different deployment configurations. The services are
:ref:`orchestrated by a Makefile <Deploy using Makefile>` for convenience and
for increased ease of adoption.

This page describes how these services are built and deployed individually to
help you customize GISNav's deployments beyond what the Makefile offers.

Prerequisites
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. include:: ../_shared/prerequisites/docker.rst

.. include:: ../_shared/prerequisites/gisnav.rst

.. include:: ../_shared/prerequisites/compose_project_name_env_variable.rst

Overview of services
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The `docker-compose.yaml
<https://github.com/hmakelin/gisnav/blob/master/docker/docker-compose.yaml>`_
file defines all services used to support GISNav deployments. The diagram below
describes the system architecture through the external interfaces between the
Docker Compose services. The GISNav service is outlined in red.

The Docker bridge networks have in-built DNS which means the container names
depicted in the diagram resolve to their respective IP addresses.

.. dropdown:: See YAML source code
    :icon: code

    .. literalinclude:: ../../../../docker/docker-compose.yaml
        :caption: Docker Compose services
        :language: yaml

.. todo::
    Split mavlink network into mavlink and ROS networks. For ROS the intention
    is to use the shared memory device instead of serializing and going through
    the network stack since we will be passing a lot of images around.

.. note::
    The application services have access to both networks and are not actually
    duplicated.

.. tab-set::

    .. tab-item:: SITL simulation & GISNav on host
        :selected:

        .. warning::
            The ``px4`` service must be started at the same time as or after
            the ``qgc``, ``mavros`` and ``micro-ros-agent`` services are started.
            Otherwise the ``px4`` service will not know their IP addresses in the
            ``gisnav_mavlink`` network. See the `px4/entrypoint.sh
            <https://github.com/hmakelin/gisnav/blob/master/docker/px4/entrypoint.sh>`_
            file for details.

        .. raw:: html
            :file: ../../../_build/external_interfaces.html

    .. tab-item:: SITL simulation & GISNav on companion computer
        :selected:

            .. todo::
                Add diagrams for different deployment configurations, e.g.
                including use Docker Compose overrides.

    .. tab-item:: HIL simulation
        :selected:

            .. todo::
                Add diagrams for different deployment configurations, e.g.
                including use Docker Compose overrides.

Example deployments
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The interdependencies between different services are hard-coded into the
`docker-compose.yaml
<https://github.com/hmakelin/gisnav/blob/master/docker/docker-compose.yaml>`_
file and typically you will only need to explicitly start one service unless
you also want to start optional services.

Mock GPS demo
********************************************

To deploy the mock :term:`GPS` demonstration introduced on the :ref:`Get Started`
page locally, ensure your development system satisfies all the :ref:`prerequisites
<Prerequisites>` and then follow the below steps to create, start, and shutdown
the required containers.


.. code-block:: bash
    :caption: Build images and create containers

    cd ~/colcon_ws/src/gisnav/docker
    docker compose -p gisnav create --build gisnav qgc

.. include:: ../_shared/expose_xhost.rst

.. code-block:: bash
    :caption: Start containers

    cd ~/colcon_ws/src/gisnav/docker
    docker compose -p gisnav start gisnav qgc

.. include:: ../_shared/docker_compose_shutdown.rst

Local development
********************************************

When deploying for local development, the difference to
:ref:`deploying the Get Started demonstration <Deploy demonstration>` is that
we do not include the ``gisnav`` service which is assumed to be
:ref:`launched separately from a local development version <Launch GISNav>`:

.. code-block:: bash
    :caption: Build images and create containers

    cd ~/colcon_ws/src/gisnav/docker
    docker compose create --build \
        qgc \
        rviz \
        px4 \
        qgis \
        mapserver \
        postgres

.. include:: ../_shared/expose_xhost.rst

.. code-block:: bash
    :caption: Start containers

    cd ~/colcon_ws/src/gisnav/docker
    docker compose start \
        qgc \
        rviz \
        px4 \
        qgis \
        mapserver \
        postgres

After you have your supporting services deployed you would typically
:ref:`use the ROS 2 launch system <Use ROS 2 launch system>` to launch your
locally installed development version of GISNav:

.. include:: ../_shared/launch_gisnav_with_ros2_launch.rst


.. include:: ../_shared/docker_compose_shutdown.rst
