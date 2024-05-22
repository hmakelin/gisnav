#!/bin/bash
# Determines what Docker Compose overrides should be used based on environment
# (most likely based on GPU type).
#
# Requires export_gpu_type.sh and GISNAV_GPU_TYPE env variable.

gisnav_docker_home=/etc/gisnav/docker

case $GISNAV_GPU_TYPE in
    "nvidia")
        echo "Using Docker Compose with Nvidia override."
        compose_files="-f $gisnav_docker_home/docker-compose.yaml -f $gisnav_docker_home/docker-compose.nvidia.yaml"
        ;;
    "broadcom")
        echo "Using Docker Compose with Broadcom override."
        compose_files="-f $gisnav_docker_home/docker-compose.yaml -f $gisnav_docker_home/docker-compose.broadcom.yaml"
        ;;
    "none")
        echo "No GPU detected. Using Docker Compose without GPU-specific overrides."
        compose_files="-f $gisnav_docker_home/docker-compose.yaml"
        ;;
    *)
        echo "Unknown GPU type detected. Using Docker Compose without GPU-specific overrides."
        compose_files="-f $gisnav_docker_home/docker-compose.yaml"
        ;;
esac

# Export the Docker Compose overrides as an environment variable
export GISNAV_COMPOSE_FILES=$compose_files
