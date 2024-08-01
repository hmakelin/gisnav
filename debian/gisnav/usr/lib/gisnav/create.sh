#!/bin/bash
# Custom script handling the creation of containers. We expose X server to
# specific containers to enable some the GUI apps to work.

gisnav_docker_home=/etc/gisnav/docker

/usr/lib/gisnav/compose.sh "$@"

# Expose X server to new containers if X display server is available
if xdpyinfo >/dev/null 2>&1; then
    make -C $gisnav_docker_home expose-xhost
fi
