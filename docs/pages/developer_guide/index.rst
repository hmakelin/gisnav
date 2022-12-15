**************************************************
Developer Guide
**************************************************
This section provides instruction on how you can integrate GISNav with your project as well as configure and extend
it to match your use case.

.. warning::
    Do not use this software for real drone flights. GISNav is untested and has only been demonstrated in a
    software-in-the-loop (SITL) simulation environment.

.. toctree::
    :caption: Setup SITL environment

    sitl_environment/docker
    sitl_environment/prerequisites
    sitl_environment/autopilot
    sitl_environment/qgroundcontrol
    sitl_environment/gis_server
    sitl_environment/install_gisnav

.. toctree::
    :caption: Launch GISNav

    launch/launch_files
    launch/run_individual_nodes
    launch/modify_params

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
