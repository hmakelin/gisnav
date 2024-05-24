To improve maintainability, the Docker Compose configuration is split into multiple files.

A script that comes with the Debian package and is included in the source code helps you define the required configuration overlays for your specific system. Use the below examples to source the `GISNAV_COMPOSE_FILES` environment variable by directly invoking the script in the source code repository (you do not have to install the Debian package):

::: code-group

```bash [Persistent <Badge type="tip" text="Recommended"/>]
source ~/colcon_ws/src/gisnav/debian/gisnav/usr/lib/gisnav/export_compose_files.sh ~/colcon_ws/src/gisnav/docker
echo "export GISNAV_COMPOSE_FILES=$GISNAV_COMPOSE_FILES" >> ~/.bashrc
```

```bash [Current session]
source ~/colcon_ws/src/gisnav/debian/gisnav/usr/lib/gisnav/export_compose_files.sh ~/colcon_ws/src/gisnav/docker
```

:::

You can then check that you are correctly parsing the Compose file stack by inspecting the resolved configuration in canonical format:

```bash
cd ~/colcon_ws/src/gisnav/docker
docker compose -p gisnav $GISNAV_COMPOSE_FILES config
```
