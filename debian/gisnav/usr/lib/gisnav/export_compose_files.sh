#!/bin/bash
# Determines what Docker Compose overrides should be used based on environment
# (most likely based on GPU type).

# Load the GPU type from the export_gpu_type.sh script
source /usr/lib/gisnav/export_gpu_type.sh

gisnav_docker_home=${1:-/etc/gisnav/docker}

# Print the path for verification
echo "Using GISNav Docker home at: $gisnav_docker_home"

# Base compose files, excluding GPU-specific ones
compose_files="-f $gisnav_docker_home/docker-compose.yaml \
               -f $gisnav_docker_home/docker-compose.commands.yaml \
               -f $gisnav_docker_home/docker-compose.dependencies.yaml \
               -f $gisnav_docker_home/docker-compose.labels.yaml \
               -f $gisnav_docker_home/docker-compose.networking.yaml \
               -f $gisnav_docker_home/docker-compose.ros.yaml \
               -f $gisnav_docker_home/docker-compose.socket.yaml \
               -f $gisnav_docker_home/docker-compose.tty.yaml \
               -f $gisnav_docker_home/docker-compose.volumes.yaml \
               -f $gisnav_docker_home/docker-compose.x11.yaml"

# Determine the GPU override
case $GISNAV_GPU_TYPE in
    "nvidia")
        echo "Using Docker Compose with Nvidia override."
        compose_files="$compose_files -f $gisnav_docker_home/docker-compose.gpu.nvidia.yaml"
        ;;
    "broadcom")
        echo "Using Docker Compose with Broadcom override."
        compose_files="$compose_files -f $gisnav_docker_home/docker-compose.gpu.broadcom.yaml"
        ;;
    "none")
        echo "No GPU detected. Using Docker Compose without GPU-specific overrides."
        ;;
    *)
        echo "Unknown GPU type detected. Using Docker Compose without GPU-specific overrides."
        ;;
esac

# Export the Docker Compose overrides as an environment variable
export GISNAV_COMPOSE_FILES=$compose_files
