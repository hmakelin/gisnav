#!/bin/bash
set -e

source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/opt/colcon_ws/install/setup.bash" --

# Ensure we have npm to build docs
source ~/.nvm/nvm.sh

# Needed for pip installed dev tools like pre-commit and sphinx-build
export PATH=/usr/lib/gisnav:$PATH

# Move .yaml param files to shared volume so that they can be edited via
# file manager:
# Source directory where the original .yaml files are located
SOURCE_DIR="/opt/colcon_ws/install/gisnav/share/gisnav/launch/params"

# Target directory where the .yaml files should be moved to (shared volume)
TARGET_DIR="/etc/gisnav/"

if [ ! -d "$TARGET_DIR" ]; then
    echo "WARNING: Target directory $TARGET_DIR does not exist. Please use Docker Compose to create the image to ensure ROS launch parameters are moved to a shared volume."
fi

# TODO: fix mounting of config files - currently the exposed ones are not used
# Check if the source directory exists and has yaml files
if [ -d "$SOURCE_DIR" ] && [ -d "$TARGET_DIR" ] && [ "$(ls -A $SOURCE_DIR/*.yaml 2>/dev/null)" ]; then
    # Iterate over each .yaml file in the source directory
    for file in $SOURCE_DIR/*.yaml; do
        # Extract filename from the path
        filename=$(basename "$file")

        # Check if the file already exists in the target directory
        if [ -e "$TARGET_DIR/$filename" ]; then
            echo "WARNING: $filename already exists in $TARGET_DIR. Skipping move."
        else
            # Move the file to the target directory
            mv "$file" "$TARGET_DIR"

            # Check if move was successful before creating a symlink
            if [ $? -eq 0 ]; then
                echo "INFO: Moved $filename to $TARGET_DIR"
                # Create a symbolic link in the source directory pointing to the new location
                ln -s "${TARGET_DIR}/${filename}" "${SOURCE_DIR}/${filename}"
            else
                echo "ERROR: Failed to move $filename"
            fi
        fi
    done
else
    echo "INFO: Launch parameter files or target volume not found on container - likely already moved to shared volume or shared volume is not mounted."
fi

exec "$@"
