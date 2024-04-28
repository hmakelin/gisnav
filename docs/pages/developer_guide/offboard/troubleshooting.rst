Troubleshooting
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Windows or GUI not appearing
************************************************

Expose xhost
________________________________________________

If the :term:`Gazebo`, :term:`QGroundControl`, :term:`RViz` or :term:`QGIS`
windows do not appear on your screen soon after
:ref:`deploying your Docker Compose services <Deploy with Docker Compose>`, you
may need to expose your ``xhost`` to your containers.

.. include:: ../_shared/expose_xhost.rst

Simulation is slow
************************************************

Headless mode
________________________________________________

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

GPU drivers not available
________________________________________________

Your system might not be using the GPU. Check that CUDA is available:

.. code-block::
    :caption: Check CUDA availability

    hmakelin@hmakelin-MS-7D48:~/colcon_ws/src/gisnav$ python3
    Python 3.10.12 (main, Nov 20 2023, 15:14:05) [GCC 11.4.0] on linux
    Type "help", "copyright", "credits" or "license" for more information.
    >>> import torch
    >>> torch.cuda.is_available()
    True

Sometimes this command will not return ``True`` and possibly raises an error. Try
updating your drivers and/or restarting your computer.

GPU temperature
________________________________________________

Check your :term:`GPU` temperature to ensure it's not overheating and not
being throttled.

Match visualization not appearing
************************************************

Here we assume the :term:`SITL` or :term:`HIL` simulation is working but
GISNav itself does not appear to be working since the match visualization
does not appear.

Disable SharedMemory for Fast DDS
________________________________________________

    [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7412:
    open_and_lock_file failed -> Function open_port_internal

If you are not able to establish :term:`ROS` communication between the ``mavros`` container
and the host, or receive the above error when using the ``--network host`` option, try
disabling SharedMemory for Fast DDS **on your host**. You can do so by creating an XML
configuration (e.g., ``disable_shared_memory.xml``) as described in `this comment`_
or discussion `here`_ and restarting the ROS daemon with the new configuration:

.. _this comment: https://github.com/eProsima/Fast-DDS/issues/1698#issuecomment-778039676
.. _here: https://stackoverflow.com/questions/65900201/troubles-communicating-with-ros2-node-in-docker-container

.. code-block:: bash

    export FASTRTPS_DEFAULT_PROFILES_FILE=disable_fastrtps.xml
    ros2 daemon stop
    ros2 daemon start

ArduPilot simulation not working
************************************************

.. todo::
    Currently ArduPilot support is broken and the simulation is not expected
    to work. Use PX4 instead.

Disable AppArmor
________________________________________________

.. warning::
    Consider the security implications to your system before trying this out.

Possibly needed if using ``--network host``: If QGroundControl or Gazebo do
not seem to be starting when running the containers, you may need to run them
image with ``--security-opt apparmor:unconfined`` or ``--privileged`` options.

General debugging
************************************************

Run shell inside container
________________________________________________

If you need to do debugging on your
:ref:`Docker Compose images <Deploy with Docker Compose>` with :term:`GUI`
applications (e.g. :term:`Gazebo` inside the ``px4`` service), run bash inside your
service container using the following command:

.. code-block:: bash
    :caption: Run bash inside px4 service container

    cd ~/colcon_ws/src/gisnav/docker
    docker compose -p gisnav run px4 bash

You will need to use ``docker compose`` here instead of ``docker`` to for the
GUI applications to work properly.
