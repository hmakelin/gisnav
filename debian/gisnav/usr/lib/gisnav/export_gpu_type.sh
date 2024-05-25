#!/bin/bash
# Detects GPU manufacturer ("nvidia", "broadcom", or "none") and exports it in
# a GISNAV_GPU_TYPE environment variable. If both Nvidia and Broadcom are detected,
# Nvidia will be exported.

# Check for verbose flag
verbose=0
for arg in "$@"; do
    if [[ "$arg" == "-v" || "$arg" == "--verbose" ]]; then
        verbose=1
        break
    fi
done

# Initialize GPU type variable
gpu_type="none"

# Check for VideoCore GPU
if vcgencmd version &> /dev/null; then
    if [[ $verbose -eq 1 ]]; then
        echo "VideoCore (Broadcom) GPU detected."
    fi
    gpu_type="broadcom"
else
    if [[ $verbose -eq 1 ]]; then
        echo "VideoCore (Broadcom) GPU not detected."
    fi
fi

# Check for Nvidia GPU using nvidia-smi, tegrastats, and lspci
if command -v nvidia-smi &> /dev/null; then
    if [[ $verbose -eq 1 ]]; then
        echo "Nvidia GPU detected."
    fi
    gpu_type="nvidia"
elif command -v tegrastats &> /dev/null; then
    if [[ $verbose -eq 1 ]]; then
        echo "Nvidia GPU detected (via tegrastats)."
    fi
    gpu_type="nvidia"
elif lspci | grep -i nvidia &> /dev/null; then
    if [[ $verbose -eq 1 ]]; then
        echo "Nvidia GPU detected (via lspci)."
    fi
    gpu_type="nvidia"
else
    if [[ $verbose -eq 1 ]]; then
        echo "Nvidia GPU not detected."
    fi
fi

# Output the GPU type
if [[ $verbose -eq 1 ]]; then
    echo "Detected GPU type: $gpu_type"
fi

# Export the GPU type as an environment variable
export GISNAV_GPU_TYPE=$gpu_type
