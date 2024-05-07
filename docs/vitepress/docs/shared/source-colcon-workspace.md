For convenience, source your workspace in your bash profile if you do a lot of development work:

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source ~/colcon_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

Alternatively, simply source it in your current shell:

```bash
source /opt/ros/humble/setup.bash
source ~/colcon_ws/install/setup.bash
```

Test that you have sourced your workspace by listing available packages:

```bash
ros2 pkg list
```

If you see a list of packages, your workspace is probably active.
