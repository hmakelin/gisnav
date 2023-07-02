Troubleshooting
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Windows or :term:`GUI` not appearing
************************

Expose xhost
________________________

If the :term:`Gazebo`, :term:`QGroundControl` or :term:`RViz` windows do not
appear on your screen soon after
:ref:`deploying your Docker Compose services <Deploy with Docker Compose>`, you
may need to expose your ``xhost`` to your containers.

.. include:: _expose_xhost.rst

Simulation is slow
************************

Headless mode
________________________

When developing on a lower performance system or when doing automated testing
(e.g. with :term:`MAVSDK`) you may want to run the :term:`Gazebo` simulation
in headless mode to increase performance:

.. note::
    See :ref:`Deploy with Docker Compose` for more information on how to build
    the Docker Compose services used in the below example.

.. tab-set::

    .. tab-item:: PX4
        :selected:

        .. code-block:: bash
            :caption: Run Gazebo in headless mode

            cd ~/colcon_ws/src/gisnav/docker
            docker compose -f docker-compose.headless.yaml up px4

    .. tab-item:: ArduPilot

        .. code-block:: bash
            :caption: Run Gazebo in headless mode

            cd ~/colcon_ws/src/gisnav/docker
            docker compose -f docker-compose.headless.yaml up ardupilot

GPU temperature
________________________

Check your :term:`GPU` temperature to ensure it's not overheating and not
being throttled.

Match visualization not appearing
************************

Here we assume the :term:`SITL` or :term:`HIL` simulation is working but
GISNav itself does not appear to be working since the match visualization
does not appear.

Disable SharedMemory for Fast DDS
________________________

    [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7412:
    open_and_lock_file failed -> Function open_port_internal

If you are not able to establish :term:`ROS` communication between the ``mavros`` or
``micro-ros-agent`` containers and the host, or receive the above error when
using the ``--network host`` option, try disabling SharedMemory for Fast DDS
**on your host**. You can do so by creating an XML configuration (e.g.,
``disable_shared_memory.xml``) as described in `this comment`_
or discussion `here`_ and restarting the ROS daemon with the new configuration:

.. _this comment: https://github.com/eProsima/Fast-DDS/issues/1698#issuecomment-778039676
.. _here: https://stackoverflow.com/questions/65900201/troubles-communicating-with-ros2-node-in-docker-container

.. code-block:: bash

    export FASTRTPS_DEFAULT_PROFILES_FILE=disable_fastrtps.xml
    ros2 daemon stop
    ros2 daemon start

ArduPilot simulation not working
************************

Disable AppArmor
________________________

.. warning::
    Consider the security implications to your system before trying this out.

Possibly needed if using ``--network host``: If QGroundControl or Gazebo do
not seem to be starting when running the containers, you may need to run them
image with ``--security-opt apparmor:unconfined`` or ``--privileged`` options.

General debugging
************************

Run shell inside container
________________________

If you need to do debugging on your
:ref:`Docker Compose images <Deploy with Docker Compose>` with :term:`GUI`
applications (e.g. :term:`Gazebo` inside the ``px4`` service), run bash inside your
service container using the following command:

.. code-block:: bash
    :caption: Run bash inside px4 service container

    cd ~/colcon_ws/src/gisnav/docker
    docker compose run px4 bash
