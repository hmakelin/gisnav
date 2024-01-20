* Setup and source a ROS 2 colcon workspace. To setup the workspace run the
  below command:

  .. code-block:: bash
      :caption: Create a workspace

      mkdir -p ~/colcon_ws/src/gisnav

  For convenience you might want to source your workspace in your bash profile
  if you do a lot of development:

  .. code-block:: bash
      :caption: Source workspace in bash profile
      :substitutions:

      echo "source /opt/ros/|ros_version|/setup.bash" >> ~/.bashrc
      echo "source ~/colcon_ws/install/setup.bash" >> ~/.bashrc
      source ~/.bashrc

  If you already have setup a workspace, you can simply source it in your current
  shell:

  .. code-block:: bash
      :caption: Source workspace in current shell
      :substitutions:

      source /opt/ros/|ros_version|/setup.bash
      source ~/colcon_ws/install/setup.bash

  Test that you have sourced your workspace by listing available packages:

  .. code-block:: bash
      :caption: List packages in workspace

      ros2 pkg list

  If you see a list of packages, your workspace is probably active.