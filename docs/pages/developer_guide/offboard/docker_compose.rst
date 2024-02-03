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

.. dropdown:: See YAML source code
    :icon: code

    .. literalinclude:: ../../../../docker/docker-compose.yaml
        :caption: Docker Compose services
        :language: yaml

.. todo::
    Split mavlink network into mavlink and ROS networks

.. note::
    The application services have access to both networks and are not actually
    duplicated.

.. raw:: html
    :file: ../../../_build/external_interfaces.html

Example deployments
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Mock GPS demo
********************************************

To deploy the mock :term:`GPS` demonstration introduced on the :ref:`Get Started`
page locally, ensure your development system satisfies all the :ref:`prerequisites
<Prerequisites>` and then follow the below steps to create, start, and shutdown
the required containers.


.. code-block:: bash
    :caption: Build images and create containers

    cd ~/colcon_ws/src/gisnav/docker
    docker compose -p gisnav create --build \
        mapserver \
        micro-ros-agent \
        mavros \
        qgc \
        px4 \
        gisnav


.. include:: ../_shared/expose_xhost.rst

.. code-block:: bash
    :caption: Start containers

    cd ~/colcon_ws/src/gisnav/docker
    docker compose -p gisnav start \
        mapserver \
        micro-ros-agent \
        mavros \
        qgc \
        px4 \
        gisnav


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
        mapserver \
        micro-ros-agent \
        mavros \
        qgc \
        rviz \
        px4 \
        qgis \
        postgres

.. include:: ../_shared/expose_xhost.rst

.. code-block:: bash
    :caption: Start containers

    cd ~/colcon_ws/src/gisnav/docker
    docker compose start \
        mapserver \
        micro-ros-agent \
        mavros \
        qgc \
        rviz \
        px4 \
        qgis \
        postgres

After you have your supporting services deployed you would typically
:ref:`use the ROS 2 launch system <Use ROS 2 launch system>` to launch your
locally installed development version of GISNav:

.. include:: ../_shared/launch_gisnav_with_ros2_launch.rst


.. include:: ../_shared/docker_compose_shutdown.rst
