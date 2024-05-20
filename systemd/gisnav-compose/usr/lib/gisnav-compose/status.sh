#!/bin/bash

# Define common variables
project_name="gisnav"
services="gisnav nginx"

# Figure out which Docker Compose overrides to use based on GPU type
source /usr/lib/gisnav-compose/export_compose_files.sh

# Launch Docker Compose with the determined options
docker compose -p $project_name ps
