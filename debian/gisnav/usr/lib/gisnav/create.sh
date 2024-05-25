#!/bin/bash
# Custom script handling the creation of containers. We expose X server to
# specific containers to enable some the GUI apps to work.

gisnav_docker_home=/etc/gisnav/docker

/usr/lib/gisnav/compose.sh "$@"

# Expose X server to new containers
make -C $gisnav_docker_home expose-xhost
