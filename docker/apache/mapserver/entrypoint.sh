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

    echo "Generating VRT file: $VRT_FILE"
    cd /etc/mapserver && gdalbuildvrt "$VRT_FILE" "$DIR_PATH"/*.tif "$DIR_PATH"/*.tiff "$DIR_PATH"/*.jp2 "$DIR_PATH"/*.ecw "$DIR_PATH"/*.img
}

process_directory_change() {
    DIR_PATH=$1
    VRT_FILE=$2
    FILE=$3

    echo "Detected change in file: $FILE"
    BASENAME=$(basename "$FILE")

    # Skip temporary download files
    if [[ "$BASENAME" =~ \.part$|\.download$ ]]; then
        echo "Ignoring partial download file: $FILE"
    elif [[ "$BASENAME" =~ \.zip$ ]]; then
        echo "Scheduling unzip of $FILE..."
        unzip_with_retry $FILE $DIR_PATH
    elif [[ "$BASENAME" =~ \.(tif|tiff|jp2|ecw|img)$ ]]; then
        echo "Raster file detected, regenerating VRT."
        generate_vrt $DIR_PATH $VRT_FILE
    fi
    return
}

# Monitor IMAGERY_DIR
inotifywait -m -e create -e moved_to -e delete "$IMAGERY_DIR" --format '%w%f' | while read FILE
do
    process_directory_change "$IMAGERY_DIR" "$IMAGERY_VRT_FILE" "$FILE"
done &

# Monitor DEM_DIR
inotifywait -m -e create -e moved_to -e delete "$DEM_DIR" --format '%w%f' | while read FILE
do
    process_directory_change "$DEM_DIR" "$DEM_VRT_FILE" "$FILE"
done &

# Move demo imagery and DEM over to shared volume if still on image
# This should trigger the inotify script
mv /etc/mapserver/$NAIP_ZIP_FILENAME $IMAGERY_DIR || echo "NAIP imagery not found on container - likely already moved to shared volume"
mv /etc/mapserver/$DEM_FILENAME $DEM_DIR || echo "USGS DEM not found on container - likely already moved to shared volume"

# Regenerate .vrt files on startup in case they have been destroyed for some reason
generate_vrt "$DEM_DIR" "$DEM_VRT_FILE"
generate_vrt "$IMAGERY_DIR" "$IMAGERY_VRT_FILE"

exec "$@"
