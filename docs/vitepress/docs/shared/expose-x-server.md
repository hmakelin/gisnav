This method is more secure as it only exposes the X server to containers with names containing "gisnav":

```bash
for containerId in $(docker ps -f name=gisnav -aq); do
    xhost +local:$(docker inspect --format='{{ .Config.Hostname }}' $containerId)
done
```

A recipe to expose the X server is also included in the Makefile `expose-xhost` target:

```bash
cd ~/colcon_ws/src/gisnav/docker
make expose-xhost
```

This method is easier but less secure as it exposes the X server to any client:

```bash
xhost +
```
