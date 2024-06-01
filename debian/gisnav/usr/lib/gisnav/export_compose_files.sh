#!/bin/bash
# Determines what Docker Compose overrides should be used based on environment
# (most likely based on GPU type). Exports GISNAV_COMPOSE_FILES.
set -e

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
               -f $gisnav_docker_home/docker-compose.healthcheck.yaml \
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

# Add HIL layer if we are in HIL mode
if [[ "${GISNAV_MODE:?empty or not set}" == "hil" ]]; then
    # TODO: support devices that do not have "PX4" in here - this is brittle
    # Find the Pixhawk device path
    gisnav_serial_device_name=$(ls /dev/serial/by-id/ | grep PX4)

    # Check if a device was found
    if [ -z "$gisnav_serial_device_name" ]; then
        echo "No PX4 device found"
        exit 1
    else
        echo "Found PX4 device $gisnav_serial_device_name"
    fi

    gisnav_serial_device="/dev/serial/by-id/$gisnav_serial_device_name"

    # Create the /tmp/.gnc.env file
    temp_env=/tmp/.gnc.env
    cp $gisnav_docker_home/.env $temp_env

    # Append the Pixhawk device path to the /tmp/.gnc.env file
    echo "GISNAV_SERIAL_DEVICE=$gisnav_serial_device" >> $temp_env

    compose_files="$compose_files -f $gisnav_docker_home/docker-compose.hil.yaml"
    compose_files="$compose_files -f $gisnav_docker_home/docker-compose.commands.hil.yaml"

    compose_files="--env-file $temp_env $compose_files"
else
    compose_files="--env-file $gisnav_docker_home/.env $compose_files"
fi

# Export the Docker Compose overrides as an environment variable
export GISNAV_COMPOSE_FILES=$compose_files
