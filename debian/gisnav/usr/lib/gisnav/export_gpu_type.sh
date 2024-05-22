#!/bin/bash
# Detects GPU manufacturer ("nvidia", "broadcom", or "none"). and exports it in
# a GISNAV_GPU_TYPE environment variable. If both Nvidia and Broadcom are detected,
# Nvidia will be exported.

# Initialize GPU type variable
gpu_type="none"

# Check for VideoCore GPU
if vcgencmd version &> /dev/null; then
    echo "VideoCore (Broadcom) GPU detected."
    gpu_type="broadcom"
else
    echo "VideoCore (Broadcom) GPU not detected."
fi

# Check for Nvidia GPU using nvidia-smi, tegrastats, and lspci
if command -v nvidia-smi &> /dev/null; then
    echo "Nvidia GPU detected."
    gpu_type="nvidia"
elif command -v tegrastats &> /dev/null; then
    echo "Nvidia GPU detected (via tegrastats)."
    gpu_type="nvidia"
elif lspci | grep -i nvidia &> /dev/null; then
    echo "Nvidia GPU detected (via lspci)."
    gpu_type="nvidia"
else
    echo "Nvidia GPU not detected."
fi

# Output the GPU type
echo "Detected GPU type: $gpu_type"

# Export the GPU type as an environment variable
export GISNAV_GPU_TYPE=$gpu_type
