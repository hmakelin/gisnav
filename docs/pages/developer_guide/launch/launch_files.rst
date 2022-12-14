Launch from ROS launch file
____________________________________________________
The easiest way to launch GISNav is to use the premade launch files in the ``launch/`` folder.

Assuming you have already installed the ``gisnav`` colcon package, you can launch with a single command:

.. tab-set::

    .. tab-item:: PX4
        :selected:

        .. code-block:: bash
            :caption: Launch GISNav for PX4

            ros2 launch gisnav px4.launch.py

        Your shell should then start showing log messages generated by GISNav's nodes:

        .. code-block:: text
            :caption: Example output when launching GISNav for PX4

            hmakelin@hmakelin-Nitro-AN515-54:~/workspace/px4_ros_com_ros2$ ros2 launch gisnav px4.launch.py
            [INFO] [launch]: All log files can be found below /home/hmakelin/.ros/log/2022-12-14-13-55-31-751707-hmakelin-Nitro-AN515-54-1269819
            [INFO] [launch]: Default logging verbosity is set to INFO
            [INFO] [px4_node-1]: process started with pid [1269837]
            [INFO] [mock_gps_node-2]: process started with pid [1269839]
            [INFO] [map_node-3]: process started with pid [1269841]
            [INFO] [bbox_node-4]: process started with pid [1269843]
            [INFO] [pose_estimation_node-5]: process started with pid [1269845]
            [pose_estimation_node-5] [INFO] [1671026134.317154046] [pose_estimation_node]: ROS parameter "max_pitch" already declared with value "30".
            [pose_estimation_node-5] [INFO] [1671026134.317957668] [pose_estimation_node]: ROS parameter "min_match_altitude" already declared with value "50".
            [pose_estimation_node-5] [INFO] [1671026134.318823560] [pose_estimation_node]: ROS parameter "attitude_deviation_threshold" already declared with value "10".
            [pose_estimation_node-5] [INFO] [1671026134.319698017] [pose_estimation_node]: ROS parameter "export_position" already declared with value "".
            [pose_estimation_node-5] [INFO] [1671026134.326348362] [pose_estimation_node]: Loaded params:
            ...

    .. tab-item:: ArduPilot

        .. code-block:: bash
            :caption: Launch GISNav for ArduPilot

            ros2 launch gisnav ardupilot.launch.py

        Your shell should then start showing log messages generated by GISNav's nodes:

        .. code-block:: text
            :caption: Example output when launching GISNav for ArduPilot

            hmakelin@hmakelin-Nitro-AN515-54:~/workspace/px4_ros_com_ros2$ ros2 launch gisnav ardupilot.launch.py
            [INFO] [launch]: All log files can be found below /home/hmakelin/.ros/log/2022-12-14-20-15-13-434747-hmakelin-Nitro-AN515-54-1299156
            [INFO] [launch]: Default logging verbosity is set to INFO
            [INFO] [ardupilot_node-1]: process started with pid [1299173]
            [INFO] [mock_gps_node-2]: process started with pid [1299175]
            [INFO] [map_node-3]: process started with pid [1299177]
            [INFO] [bbox_node-4]: process started with pid [1299179]
            [INFO] [pose_estimation_node-5]: process started with pid [1299181]
            [pose_estimation_node-5] [INFO] [1671048916.388314406] [pose_estimation_node]: ROS parameter "max_pitch" already declared with value "30".
            [pose_estimation_node-5] [INFO] [1671048916.389742861] [pose_estimation_node]: ROS parameter "min_match_altitude" already declared with value "50".
            [pose_estimation_node-5] [INFO] [1671048916.391036526] [pose_estimation_node]: ROS parameter "attitude_deviation_threshold" already declared with value "10".
            [pose_estimation_node-5] [INFO] [1671048916.392134461] [pose_estimation_node]: ROS parameter "export_position" already declared with value "".
            [pose_estimation_node-5] [INFO] [1671048916.398298326] [pose_estimation_node]: Loaded params:
            ...

.. seealso::
    See the `Creating Launch Files`_ tutorial for more information on the ROS 2 launch system

    .. _Creating Launch Files: https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Creating-Launch-Files.html