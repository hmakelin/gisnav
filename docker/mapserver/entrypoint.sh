#!/bin/bash
# This script automatically manages raster files (such as TIFF, JP2, ECW, and IMG formats)
# within a specified download directory (/etc/mapserver) for use with MapServer. It listens
# for file creation, movement, and deletion events in the download directory. When a new ZIP
# file is added to the directory, the script unzips its contents into the same directory and
# then removes the original ZIP file. If a raster file is added directly or extracted from a
# ZIP file, the script regenerates a Virtual Raster (VRT) file to include all raster files
# present in the directory. After updating the VRT file, it copies both the VRT and all
# raster files to a target directory (/etc/mapserver/imagery) for use by MapServer.
# Subsequently, it cleans up by removing the original raster files from the download directory.
# This process ensures that MapServer always serves the latest raster data available in the
# target directory, while keeping the download directory clean and free of processed files.
set -e

# Remove any stale PID or socket files before starting Apache
rm -f /var/run/apache2/apache2.pid
rm -f /var/run/apache2/cgisock.*

# Script settings and monitored directory configurations.
DOWNLOAD_DIR="/etc/mapserver/maps"
TARGET_DIR="/etc/mapserver/maps"
VRT_FILE="naip.vrt"  # TODO: make name more generic, e.g. default.vrt

mkdir -p $DOWNLOAD_DIR
mkdir -p $TARGET_DIR

# Start monitoring the download directory for new files, deletions, and movements.
inotifywait -m -e create -e moved_to -e delete "$DOWNLOAD_DIR" --format '%w%f' | while read FILE
do
    echo "Detected change in file: $FILE"
    BASENAME=$(basename "$FILE")

    # Skip temporary download files
    if [[ "$BASENAME" =~ \.part$|\.download$ ]]; then
        echo "Ignoring partial download file: $FILE"
        continue
    fi

    if [[ "$BASENAME" =~ \.zip$ ]]; then
        # Attempt to unzip the file
        echo "Attempting to unzip $FILE..."
        if ! unzip -o "$FILE" -d "$DOWNLOAD_DIR" > /dev/null 2>&1; then
            echo "Unzip failed, possibly due to incomplete download: $FILE. Will retry on next trigger."
            continue
        else
            echo "Unzipped successfully: $FILE"
            # Optionally, delete the ZIP file after successful extraction
            rm "$FILE"
        fi
    fi

    # Regenerate the VRT file if a raster file is detected or after unzipping
    if [[ "$BASENAME" =~ \.(tif|tiff|jp2|ecw|img)$ ]]; then
        # Move all raster files to TARGET_DIR
        echo "Moving raster files to $TARGET_DIR..."
        find "$DOWNLOAD_DIR" -regex ".*\.\(tif\|tiff\|jp2\|ecw\|img\)$" -exec mv {} "$TARGET_DIR" \;

        echo "Raster file detected, regenerating VRT. You can safely ignore errors or warnings relating to missing files below unless you expect specific files to be present. Pay attention to the difference between the two supported TIFF file extensions: .tif and .tiff."
        # Update the VRT file to include all raster files in DOWNLOAD_DIR
        # VRT file not in imagery folder which should make it hidden from
        # external users (not on shared volume)
        cd /etc/mapserver && gdalbuildvrt "$VRT_FILE" "$DOWNLOAD_DIR"/*.tif "$DOWNLOAD_DIR"/*.tiff "$DOWNLOAD_DIR"/*.jp2 "$DOWNLOAD_DIR"/*.ecw "$DOWNLOAD_DIR"/*.img

        # Move the VRT file to TARGET_DIR *after* moving the raster files
        # as this overwrites the potential old one already there
        #echo "Moving VRT file to $TARGET_DIR..."
        #mv "$VRT_FILE" "$TARGET_DIR/$VRT_FILE"
    fi
done &

exec "$@"
