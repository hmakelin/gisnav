**************************************************
Developer Guide
**************************************************
This section provides instruction on how you can integrate GISNav with your project as well as configure and extend
it to match your use case.

.. warning::
    Do not use this software for real drone flights. GISNav is untested and has only been demonstrated in a
    software-in-the-loop (SITL) simulation environment.

.. toctree::
    :caption: Run GISNav

    launch/service_orchestration
    launch/docker_compose
    launch/ros2_launch_system
    launch/run_individual_node
    launch/modify_params
    launch/troubleshooting

.. toctree::
    :caption: Setup SITL

    sitl/prerequisites
    sitl/autopilot
    sitl/qgroundcontrol
    sitl/gis_server
    sitl/install_gisnav
    sitl/companion_computer

.. toctree::
    :caption: Setup HIL

    hil/jetson_pixhawk

.. toctree::
    :caption: Run tests

    test/test_prerequisites
    test/launch_tests
    test/code_coverage
    test/sitl_tests

.. toctree::
    :caption: Integrate with your own project

    integrate/ros_messaging
    integrate/mock_gps
    integrate/pose_estimators
