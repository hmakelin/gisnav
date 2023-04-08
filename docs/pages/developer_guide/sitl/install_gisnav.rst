GISNav ROS package
____________________________________________________
Install ROS 2
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
If you followed the :ref:`Autopilots` SITL install instructions you should already have ROS 2 installed. See
`ROS 2 foxy install instructions`_  if you do not yet have ROS 2 on your machine. You can check your
ROS version with the ``whereis ros2`` command:

.. code-block:: text
    :caption: Check ROS version example output

    hmakelin@hmakelin-Nitro-AN515-54:~$ whereis ros2
    ros2: /opt/ros/foxy/bin/ros2

.. _ROS 2 foxy install instructions: https://docs.ros.org/en/foxy/Installation.html

.. note::
    Once you have your `ROS 2 workspace`_ set up, consider automatically sourcing it in your ``~/.bashrc`` to avoid
    manual repetition:

    .. _ROS 2 Workspace: https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html

    .. code-block:: bash
        :caption: Source ROS 2 workspace in ~/.bashrc

        echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
        echo "source ~/colcon_ws/install/setup.bash" >> ~/.bashrc

Install GDAL
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
``GDAL`` is required to install the ``geopandas`` Python dependency:

.. code-block:: text
    :caption: Install GDAL

    sudo apt-get -y install software-properties-common
    sudo add-apt-repository ppa:ubuntugis/ppa
    sudo apt-get update
    sudo apt-get -y install gdal-bin libgdal-dev

Install GISNav
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Install GISNav in your ROS 2 Workspace:

.. code-block:: bash
    :caption: Install GISNav in colcon workspace

    cd ~/colcon_ws
    mkdir -p src && cd "$_"
    git clone https://github.com/hmakelin/gisnav.git
    git clone --branch gimbal-protocol-v2-plugin \
      https://github.com/adinkra-labs/mavros_feature_gimbal-protocol-v2-plugin.git mavros && \
    git clone https://github.com/px4/px4_msgs.git
    rosdep update
    rosdep install --from-paths . -y --ignore-src
    cd gisnav/gisnav
    pip3 install -r requirements.txt

.. note::
    If you have followed the `PX4 ROS 2 User Guide`_, your workspace may be called ``px4_ros_com_ros2`` instead of
    ``colcon_ws``

    .. _PX4 ROS 2 User Guide: https://docs.px4.io/main/en/ros/ros2_comm.html

Build the GISNav package along with other dependencies you may have in your colcon workspace. If you have already built
the other dependencies (such as ``px4_msgs`` for PX4 configuration) earlier you may want to skip
rebuilding them and build GISNav only to save time:

.. tab-set::

    .. tab-item:: With dependencies
        :selected:

        .. code-block:: bash
            :caption: Build colcon workspace

            cd ~/colcon_ws
            colcon build

    .. tab-item:: GISNav only

        .. code-block:: bash
            :caption: Build GISNav package only

            cd ~/colcon_ws
            colcon build --packages-select gisnav

Once GISNav is installed, you can try to :ref:`Launch from ROS launch file`.

Development dependencies
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
The development dependencies are required to :ref:`Generate documentation` and run :ref:`Launch tests`. Install them
with the following commands:

.. code-block:: bash
    :caption: Install Python development dependencies

    cd ~/colcon_ws/src/gisnav/gisnav
    python3 -m pip install -r requirements-dev.txt
