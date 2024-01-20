**************************************************
User guide
**************************************************
This section provides instruction on how you can integrate GISNav with your project as well as configure and extend
it to match your use case.

.. warning::
    Do not use this software for real drone flights. GISNav is untested and has only been demonstrated in a
    software-in-the-loop (SITL) simulation environment.

.. toctree::
    :caption: Development

    development/system_requirements
    development/install_gisnav
    development/test_gisnav

.. toctree::
    :caption: Offboard simulation

    offboard/service_orchestration
    offboard/docker_compose
    offboard/ros2_launch_system
    offboard/run_individual_node
    offboard/troubleshooting

.. toctree::
    :caption: Onboard simulation

    onboard/companion_computer
    onboard/jetson_pixhawk

.. toctree::
    :caption: Integration and configuration

    integration/gis_server
    integration/modify_params
    integration/ros_messaging
    integration/mock_gps
