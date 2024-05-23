#!/bin/bash

# Define common variables
project_name="gisnav"

# Parse command-line arguments for services
if [ "$#" -lt 1 ]; then
  echo "Usage: $0 <service1> <service2> ... <serviceN>"
  exit 1
fi

GISNAV_SERVICES="$@"

# Figure out which Docker Compose overrides to use based on GPU type
source /usr/lib/gisnav/export_compose_files.sh

# Launch Docker Compose with the determined options
docker compose -p $project_name $GISNAV_COMPOSE_FILES up -d $GISNAV_SERVICES
