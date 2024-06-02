#!/bin/bash
set -e

# Remove any stale PID or socket files before starting Apache
rm -f /var/run/apache2/apache2.pid
rm -f /var/run/apache2/cgisock.*

# Script settings and monitored directory configurations.
IMAGERY_DIR="/etc/mapserver/maps/imagery"
DEM_DIR="/etc/mapserver/maps/dem"
IMAGERY_VRT_FILE="imagery.vrt"
DEM_VRT_FILE="dem.vrt"

# GDAL supported raster formats
SUPPORTED_FORMATS=("tif" "tiff" "jp2" "ecw" "img")

mkdir -p $IMAGERY_DIR
mkdir -p $DEM_DIR

wait_for_file_completion() {
    local file=$1
    local prev_size=-1
    local size=0

    while true; do
        if [[ -f "$file" ]]; then
            size=$(stat -c%s "$file")
            if [[ $size -eq $prev_size ]]; then
                echo "File $file size is stable at $size bytes."
                return 0
            else
                prev_size=$size
                echo "Waiting for file $file to stabilize. Current size: $size bytes."
            fi
        else
            echo "File $file does not exist."
            return 0
        fi
        sleep 2
    done
}

unzip_with_retry() {
    local file=$1
    local dir_path=$2
    local max_retries=3
    local attempt=1

    wait_for_file_completion $file

    while (( attempt <= max_retries )); do
        echo "Unzipping attempt $attempt for $file..."
        if unzip -o "$file" -d "$dir_path"; then
            echo "Unzipped successfully: $file"
            rm "$file"
            return 0
        else
            echo "Unzip attempt $attempt failed for $file. Retrying..."
            sleep 2.0
        fi
        ((attempt++))
    done

    echo "Failed to unzip $file after $max_retries attempts. Will retry on next trigger."
    return 1
}

generate_vrt() {
    DIR_PATH=$1
    VRT_FILE=$2

    # Initialize an empty array to hold the file paths
    FILE_LIST=()

    # Add file paths to the list if they exist
    for ext in "${SUPPORTED_FORMATS[@]}"; do
        files=("$DIR_PATH"/*.$ext)
        for file in "${files[@]}"; do
            if [ -e "$file" ]; then
                wait_for_file_completion "$file"
                echo "$ext found: $file"
                FILE_LIST+=("$file")
            fi
        done
    done

    # Check if any supported files were found
    if [ ${#FILE_LIST[@]} -gt 0 ]; then
        echo "Generating VRT file: $VRT_FILE"
        cd /etc/mapserver && gdalbuildvrt "$VRT_FILE" "${FILE_LIST[@]}"
    else
        echo "No supported files found in $DIR_PATH to generate VRT"
    fi
}

process_directory_change() {
    DIR_PATH=$1
    VRT_FILE=$2
    FILE=$3
    EVENT=$4

    echo "Detected $EVENT event for file: $FILE"
    BASENAME=$(basename "$FILE")

    # Convert EVENT to lowercase
    EVENT=$(echo "$EVENT" | tr '[:upper:]' '[:lower:]')

    # Skip temporary download files
    if [[ "$BASENAME" =~ \.part$|\.download$ ]]; then
        echo "Ignoring partial download file: $FILE"
    elif [[ "$BASENAME" =~ \.zip$ ]]; then
        if [[ "$EVENT" != "delete" ]]; then
            echo "Scheduling unzip of $FILE..."
            unzip_with_retry $FILE $DIR_PATH
        fi
    else
        # Check if the file matches any of the supported formats
        for ext in "${SUPPORTED_FORMATS[@]}"; do
            if [[ "$BASENAME" =~ \.$ext$ ]]; then
                echo "Raster file detected, regenerating VRT."
                generate_vrt $DIR_PATH $VRT_FILE
                break
            fi
        done
    fi
    return
}

# Monitor IMAGERY_DIR
inotifywait -m -e create -e moved_to -e delete "$IMAGERY_DIR" --format '%e %w%f' | while read EVENT FILE
do
    process_directory_change "$IMAGERY_DIR" "$IMAGERY_VRT_FILE" "$FILE" "$EVENT"
done &

# Monitor DEM_DIR
inotifywait -m -e create -e moved_to -e delete "$DEM_DIR" --format '%e %w%f' | while read EVENT FILE
do
    process_directory_change "$DEM_DIR" "$DEM_VRT_FILE" "$FILE" "$EVENT"
done &

# Move demo imagery and DEM over to shared volume if still on image
# This should trigger the inotify script
# Check and move NAIP imagery if it exists
if [ -f /etc/mapserver/$NAIP_ZIP_FILENAME ]; then
    mv /etc/mapserver/$NAIP_ZIP_FILENAME $IMAGERY_DIR
else
    echo "NAIP imagery not found on container - likely already moved to shared volume"
fi

# Check and move USGS DEM if it exists
if [ -f /etc/mapserver/$DEM_FILENAME ]; then
    mv /etc/mapserver/$DEM_FILENAME $DEM_DIR
else
    echo "USGS DEM not found on container - likely already moved to shared volume"
fi

# Regenerate .vrt files on startup in case they have been destroyed for some reason
generate_vrt "$DEM_DIR" "$DEM_VRT_FILE"
generate_vrt "$IMAGERY_DIR" "$IMAGERY_VRT_FILE"

exec "$@"
