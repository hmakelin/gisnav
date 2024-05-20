#!/bin/bash
# Determines what Docker Compose overrides should be used based on environment
# (most likely based on GPU type).
#
# Requires export_gpu_type.sh and GISNAV_GPU_TYPE env variable.

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

# Export the Docker Compose overrides as an environment variable
export GISNAV_COMPOSE_FILES=$compose_files
