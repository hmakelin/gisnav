* You must source your colcon workspace, eg. via ``~/.bashrc``. You will need to
  know your ROS 2 version (e.g., "foxy", "humble", etc.) for the following command:

.. code-block:: bash
    :caption: Source ROS 2 workspace in ~/.bashrc

    echo "source /opt/ros/<your-ros2-version>/setup.bash" >> ~/.bashrc
    echo "source ~/colcon_ws/install/setup.bash" >> ~/.bashrc
    source ~/.bashrc
