# Run individual ROS nodes

This page provides examples for running individual ROS nodes with custom options and custom values for the ROS parameters from the YAML files in the `launch/params/` folder.

The commands here are mostly useful for debugging and development, while [Docker Compose](/deploy-with-docker-compose) and the [Makefile and ROS launch system](/deploy-for-development) are recommended for launching larger configurations of nodes.

## Prerequisites

<!--@include: ./shared/require-install-locally.md-->

## Custom log level

Change the log level with the `--log-level` argument:

```bash
cd ~/colcon_ws
ros2 run gisnav gis_node --ros-args \
    --log-level info
```

## Custom parameter values

Provide custom ROS parameter values at launch in a YAML file using the `--params-file` argument:

```bash
cd ~/colcon_ws
ros2 run gisnav gis_node --ros-args \
    --params-file src/gisnav/gisnav/launch/params/gis_node.yaml
```

::: tip Configure via admin portal
The [Admin portal](/admin-portal) home page has a link to a captive FileGator file server that allows you to edit these ROS parameter YAML files via a web browser.

:::
