Run individual ROS nodes
____________________________________________________

You can run individual :term:`ROS` nodes and optionally provide values for
the ROS parameters from the :term:`YAML` files in the ``launch/params/``
folder with the example commands presented here.

The commands here are useful for debugging and development only. For complete
functional deployments you should look into deploying GISNav
:ref:`with Make <Deploy using Makefile>`, :ref:`with Docker Compose
<Deploy with Docker Compose>`, or :ref:`using the ROS 2 launch system
<Use ROS 2 launch system>`.

Prerequisites
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

.. include:: _prerequisites_install_locally.rst

Run node with custom log level
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You can change the log level with the ``--log-level`` argument:

.. code-block:: bash
    :caption: Run :class:`.GISNode` with ``info`` log level

    cd ~/colcon_ws
    ros2 run gisnav gis_node --ros-args \
        --log-level info

Run node with custom parameter values
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

You can provide your custom ROS parameter values at launch in a :term:`YAML`
file using the ``--params-file`` argument:

.. code-block:: bash
    :caption: Run :class:`.CVNode` with ROS parameter file

    cd ~/colcon_ws
    ros2 run gisnav cv_node --ros-args \
        --params-file src/gisnav/gisnav/launch/params/cv_node.yaml
