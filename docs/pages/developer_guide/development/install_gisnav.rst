Install locally
____________________________________________________

This page describes how to install GISNav locally on your development computer.
With a local installation of GISNav it is easy to make changes and re-test the
software without having to rebuild a :term:`Docker` image.

If you are only interested in running but not necessarily developing GISNav, you
will probably want to use the :ref:`Docker Compose services
<Deploy with Docker Compose>` instead of installing locally.

Prerequisites
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. include:: ../_shared/prerequisites/ros.rst

.. include:: ../_shared/prerequisites/source_workspace.rst

Install system dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Install GISNav system dependencies with the following commands:

.. code-block:: bash
    :caption: Install GISNav system dependencies
    :substitutions:

    cd ~/colcon_ws/src
    git clone --branch |vversion| https://github.com/hmakelin/gisnav.git
    git clone \
        --branch gimbal-protocol-v2-plugin \
        https://github.com/adinkra-labs/mavros_feature_gimbal-protocol-v2-plugin.git \
        mavros
    rosdep update
    rosdep install --from-paths . -y -r --ignore-src

.. note::
    If you want to use a ROS distribution that has reached end-of-life (EOL)
    like Foxy, you can add the ``--include-eol-distros`` option to
    ``rosdep update``.

Install Python dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You must install at least the :term:`core` dependencies. If you know you are not
e.g. going to use the :class:`.MockGPSNode` :term:`extension`, you can skip
installing the extended dependencies.

The development dependencies are required for e.g. :ref:`generating documentation
<Generate documentation>` and running :ref:`Launch tests`. They are highly recommended
if you are doing development work on GISNav.

.. note::
    If you want to use a Python virtual environment for your GISNav Python
    dependencies, see `this issue`_ first. We do not provide instructions here
    as the workspace might not play nicely with the Python virtual environment.

    .. _this issue: https://github.com/ros2/ros2/issues/1094

Install the required and optional Python dependencies with the following commands:

.. tab-set::

    .. tab-item:: Core
        :selected:

        .. code-block:: bash
            :caption: Install GISNav core Python dependencies

            cd ~/colcon_ws/src/gisnav
            pip3 install ./gisnav

    .. tab-item:: Extended (optional)

        .. code-block:: bash
            :caption: Install GISNav extension Python dependencies

            cd ~/colcon_ws/src/gisnav
            pip3 install ./gisnav[mock_gps_node]
            pip3 install ./gisnav[qgis_node]

        You can install the dependencies for any :term:`extension` node
        by using its name as a package extra.

    .. tab-item:: Development (optional)

        .. code-block:: bash
            :caption: Install GISNav development Python dependencies

            cd ~/colcon_ws/src/gisnav
            pip3 install ./gisnav[dev]

Build colcon workspace
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Build the GISNav package along with other dependencies you have in your colcon
workspace. If you have already built the other dependencies earlier you may
want to skip rebuilding them and build GISNav only to save time:

.. include:: ../_shared/build_colcon_workspace.rst

Once GISNav is installed, you can for example try :ref:`launching with the ROS 2
launch system <Use ROS 2 launch system>`.
