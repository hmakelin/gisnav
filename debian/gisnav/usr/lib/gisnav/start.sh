#!/bin/bash

# Define common variables
project_name="gisnav"

# Load Docker Compose services to deploy
source /usr/lib/gisnav/export_services.sh

# Figure out which Docker Compose overrides to use based on GPU type
source /usr/lib/gisnav/export_compose_files.sh

# Launch Docker Compose with the determined options
docker compose -p $project_name $GISNAV_COMPOSE_FILES up -d $GISNAV_SERVICES
