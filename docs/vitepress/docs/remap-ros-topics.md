# Remap ROS topics

The `gisnav` ROS 2 package depends on ROS for both external and internal communication, and ROS is also the natural way to integrate GISNav with other systems. To integrate GISNav with another ROS system, some topic name remapping may be required.

::: tip Remapping GISNav output
The most interesting ROS message is most likely the filtered camera pose and twist published as `nav_msgs/Odometry` by the `ekf_localization_node` from the `robot_localization` package (a dependency of `gisnav`), or the raw camera pose published as `geometry_msgs/PoseWithCovarianceStamped`  by `pose_node` from the `gisnav` package.

The `gisnav_msgs` package includes message definitions that GISNav uses internally which are probably not interesting from an integration perspective.

See the [ROS topography](/system-architecture#ros-topography) diagram for more context.

:::

## Prerequisites

<!--@include: ./shared/require-install-locally.md-->

## Example commands

See the examples below on how to launch and run GISNav ROS nodes with remapped topic names.

### ros2 launch

The below Python launch file diff shows an example remapping of the camera topics for `StereoNode`:

```python
ld.add_action(
        Node(
            package="gisnav",
            name="stereo_node",
            namespace="gisnav",
            executable="stereo_node",
            parameters=[
                os.path.join(package_share_dir, "launch/params/stereo_node.yaml")
            ],
            remappings=[ # [!code ++]
                ("camera/camera_info", "camera_info"), # [!code ++]
                ("camera/image_raw", "image"), # [!code ++]
            ], # [!code ++]
        )
    )
```

### ros2 run

The below example shows a remapping of camera topics for `StereoNode` on the command line:

```bash
cd ~/colcon_ws
ros2 run gisnav stereo_node --ros-args \
    --params-file src/gisnav/launch/params/stereo_node.yaml \
    -r camera/camera_info:=camera_info \
    -r camera/image_raw:=image
```
