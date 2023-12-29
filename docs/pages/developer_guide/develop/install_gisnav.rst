Install locally
____________________________________________________

Here we describe how to install GISNav locally on your development computer
so that it is easy to make changes and re-test the software without having
to rebuild a :term:`Docker` image.

If you are only interested in using but not necessarily developing GISNav, you
will probably want to use the :ref:`Docker Compose builds <Deploy with Docker Compose>`
instead of installing locally.

Prerequisites
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. include:: ../launch/_prerequisites_ros.rst

.. include:: ../launch/_prerequisites_gisnav.rst

.. include:: ../launch/_prerequisites_source_workspace.rst

Install system dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Install GISNav system dependencies with the following commands:

.. code-block:: bash
    :caption: Install GISNav system dependencies

    cd ~/colcon_ws/src

    git clone \
        https://github.com/px4/px4_msgs.git
    git clone \
        --branch gimbal-protocol-v2-plugin \
        https://github.com/adinkra-labs/mavros_feature_gimbal-protocol-v2-plugin.git mavros

    rosdep update
    rosdep install --from-paths . -y --ignore-src


Install Python dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You must install at least the :term:`core` dependencies.

If you know you are not e.g. going to use the :class:`.MockGPSNode` :term:`extension`,
you can skip installing the extended dependencies.

The development dependencies are required for e.g. :ref:`generating documentation
<Generate documentation>` and running :ref:`Launch tests`. They are highly recommended
if you are doing development work on GISNav.

You may optionally use a ``virtualenv`` for managing your Python dependencies
inside your colcon workspace:

.. code-block:: bash
    :caption: Manage Python dependencies with virtualenv (optional)

    cd ~/colcon_ws
    python3 -m virtualenv venv
    source venv/bin/activate

Install the required and optional dependencies with the following commands:

.. tab-set::

    .. tab-item:: Core dependencies
        :selected:

        .. code-block:: bash
            :caption: Install GISNav core Python dependencies

            cd ~/colcon_ws/src/gisnav/gisnav
            pip3 install .

    .. tab-item:: Extended dependencies (optional)

        .. code-block:: bash
            :caption: Install GISNav extension Python dependencies

            cd ~/colcon_ws/src/gisnav/gisnav
            pip3 install .[mock_gps_node]
            pip3 install .[qgis_node]

    .. tab-item:: Development dependencies (optional)

        .. code-block:: bash
            :caption: Install GISNav development Python dependencies

            cd ~/colcon_ws/src/gisnav/gisnav
            pip3 install .[dev]

Build colcon workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Build the GISNav package along with other dependencies you have in your colcon
workspace. If you have already built the other dependencies (such as ``px4_msgs``
for PX4 configuration) earlier you may want to skip rebuilding them and build
GISNav only to save time:

.. include:: ../launch/_build_colcon_workspace.rst

Once GISNav is installed, you can for example try :ref:`launching with the ROS 2
launch system <Use ROS 2 launch system>`.
