#!/bin/bash
# Determines what Docker Compose overrides should be used based on environment
# (most likely based on GPU type). Exports GISNAV_COMPOSE_FILES.

# Check for verbose flag
verbose=0
for arg in "$@"; do
    if [[ "$arg" == "-v" || "$arg" == "--verbose" ]]; then
        verbose=1
        break
    fi
done

gisnav_docker_home=${1:-/etc/gisnav/docker}

# Print the path for verification if verbose flag is set
if [[ $verbose -eq 1 ]]; then
    echo "Using GISNav Docker home at: $gisnav_docker_home"
fi

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

# Load the GPU type from the export_gpu_type.sh script, pass verbose flag if set
if [[ $verbose -eq 1 ]]; then
    source /usr/lib/gisnav/export_gpu_type.sh --verbose
else
    source /usr/lib/gisnav/export_gpu_type.sh
fi

# Determine the GPU override
case $GISNAV_GPU_TYPE in
    "nvidia")
        if [[ $verbose -eq 1 ]]; then
            echo "Using Docker Compose with Nvidia override."
        fi
        compose_files="$compose_files -f $gisnav_docker_home/docker-compose.gpu.nvidia.yaml"
        ;;
    "broadcom")
        if [[ $verbose -eq 1 ]]; then
            echo "Using Docker Compose with Broadcom override."
        fi
        compose_files="$compose_files -f $gisnav_docker_home/docker-compose.gpu.broadcom.yaml"
        ;;
    "none")
        if [[ $verbose -eq 1 ]]; then
            echo "No GPU detected. Using Docker Compose without GPU-specific overrides."
        fi
        ;;
    *)
        if [[ $verbose -eq 1 ]]; then
            echo "Unknown GPU type detected. Using Docker Compose without GPU-specific overrides."
        fi
        ;;
esac

# Export the Docker Compose overrides as an environment variable
export GISNAV_COMPOSE_FILES="--env-file $gisnav_docker_home/.env $compose_files"
