#!/bin/bash

# Define common variables
project_name="gisnav"
services="gisnav nginx"

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

# Launch Docker Compose with the determined options
docker compose -p $project_name $compose_files up -d $services
