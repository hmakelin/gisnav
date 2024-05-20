#!/bin/bash

# Define common variables
project_name="gisnav"
services="gisnav nginx"

# Load the GPU type from the export_gpu_type.sh script
source /usr/lib/gisnav-compose/export_gpu_type.sh

# Define docker compose command options
case $GISNAV_GPU_TYPE in
    "nvidia")
        echo "Launching Docker Compose with Nvidia override."
        compose_files="-f docker-compose.yaml -f docker-compose.nvidia.yaml"
        ;;
    "broadcom")
        echo "Launching Docker Compose with Broadcom override."
        compose_files="-f docker-compose.yaml -f docker-compose.broadcom.yaml"
        ;;
    "none")
        echo "No GPU detected. Launching Docker Compose without GPU-specific overrides."
        compose_files="-f docker-compose.yaml -f docker-compose.no-gpu.yaml"
        ;;
    *)
        echo "Unknown GPU type detected. Launching Docker Compose without GPU-specific overrides."
        compose_files="-f docker-compose.yaml -f docker-compose.no-gpu.yaml"
        ;;
esac

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

# Pull or build the Docker images including dependencies and create containers
docker compose $compose_files $project_name pull --include-deps $services
docker compose $compose_files $project_name build --with-dependencies $services
docker compose $compose_files $project_name create $services

# Remove the temporary swap file after build is complete
remove_temp_swapfile
