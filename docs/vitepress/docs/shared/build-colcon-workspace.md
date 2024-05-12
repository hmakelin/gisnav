Build the colcon workspace. If you have previously built your workspace and only made changes to GISNav, you may want to only re-build GISNav:

::: code-group

```bash [Entire workspace]
cd ~/colcon_ws
colcon build
```

```bash [GISNav only]
cd ~/colcon_ws
colcon build --packages-select gisnav gisnav_msgs
```

:::

If it's your first time building the workspace, remember to also source the `install/setup.bash` script to ensure all your newly built executables and libraries are on the path:

```bash
cd ~/colcon_ws
source install/setup.bash
```
