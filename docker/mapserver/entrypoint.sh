#!/bin/bash
set -e

export MS_MAPFILE=/etc/mapserver/mapserver.map

exec "$@"