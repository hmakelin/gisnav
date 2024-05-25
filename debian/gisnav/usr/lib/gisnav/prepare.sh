#!/bin/bash

# Define common variables
project_name="gisnav"

GISNAV_SERVICES="$@"

gisnav_docker_home=/etc/gisnav/docker

# Figure out which Docker Compose overrides to use based on GPU type
source /usr/lib/gisnav/export_compose_files.sh $gisnav_docker_home

REQUIRED_SWAP=4  # Required swap size in GB
TEMP_SWAPFILE="/tmp/temp_swapfile"
TEMP_SWAPSIZE="4G"

# Function to create a temporary swap file
# For example on Raspberry Pi 5, 4GB of memory does not seem to be sufficient
# to build mavros.
create_temp_swapfile() {
    # Check existing swap space
    existing_swap=$(free -g | awk '/Swap:/ {print $2}')

    if [ "$existing_swap" -lt "$REQUIRED_SWAP" ]; then
        echo "Insufficient swap space. Creating temporary swap file..."
        sudo fallocate -l $TEMP_SWAPSIZE $TEMP_SWAPFILE
        sudo chmod 600 $TEMP_SWAPFILE
        sudo mkswap $TEMP_SWAPFILE
        sudo swapon $TEMP_SWAPFILE
        temp_swap_created=true
    else
        echo "Sufficient swap space available: ${existing_swap}GB"
        temp_swap_created=false
    fi
}

# Function to remove the temporary swap file
remove_temp_swapfile() {
    if [ "$temp_swap_created" = true ]; then
        echo "Removing temporary swap file..."
        sudo swapoff $TEMP_SWAPFILE
        sudo rm $TEMP_SWAPFILE
    fi
}

# Create a temporary swap file if needed
create_temp_swapfile

# Pull or build the Docker images including dependencies and create container
# Pulling disabled since we might not have the right CUDA versions or other
# environment specific installed on the gisnav development image in GHCR
#docker compose $GISNAV_COMPOSE_FILES -p $project_name pull --include-deps $GISNAV_SERVICES
docker compose $GISNAV_COMPOSE_FILES -p $project_name build --with-dependencies $GISNAV_SERVICES
docker compose $GISNAV_COMPOSE_FILES -p $project_name create $GISNAV_SERVICES

# Remove the temporary swap file after build is complete
remove_temp_swapfile

# Expose X server to new containers
make -C $gisnav_docker_home expose-xhost
