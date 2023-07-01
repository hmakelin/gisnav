**************************************************
User Guide
**************************************************
This section provides instruction on how you can integrate GISNav with your project as well as configure and extend
it to match your use case.

.. warning::
    Do not use this software for real drone flights. GISNav is untested and has only been demonstrated in a
    software-in-the-loop (SITL) simulation environment.

.. toctree::
    :caption: Use GISNav

    launch/service_orchestration
    launch/docker_compose
    launch/ros2_launch_system
    launch/run_individual_node
    launch/modify_params
    launch/troubleshooting

.. toctree::
    :caption: Develop GISNav

    develop/system_requirements
    develop/install_gisnav
    develop/test_gisnav

.. toctree::
    :caption: Onboard SITL & HIL

    sitl/companion_computer
    sitl/autopilot
    hil/jetson_pixhawk

.. toctree::
    :caption: Integrate with your own project

    sitl/gis_server
    integrate/ros_messaging
    integrate/mock_gps
    integrate/pose_estimators
