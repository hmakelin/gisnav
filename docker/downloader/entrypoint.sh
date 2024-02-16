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

# Script settings and monitored directory configurations.
DOWNLOAD_DIR="/etc/mapserver"
TARGET_DIR="/etc/mapserver/imagery"
VRT_FILE="downloaded-orthoimagery.vrt"

# Start monitoring the download directory for new files, deletions, and movements.
inotifywait -m -e create -e moved_to -e delete "$DOWNLOAD_DIR" --format '%w%f' | while read FILE
do
    echo "Detected change in file: $FILE"
    BASENAME=$(basename "$FILE")

    if [[ "$BASENAME" =~ \.zip$ ]]; then
        # If the file is a ZIP file, unzip it to the DOWNLOAD_DIR
        echo "Unzipping $FILE..."
        unzip -o "$FILE" -d "$DOWNLOAD_DIR"
        # Remove the ZIP file after unzipping if desired
        rm "$FILE"
    fi

    # Regenerate the VRT file if a raster file is detected or after unzipping
    if [[ "$BASENAME" =~ \.(tif|tiff|jp2|ecw|img)$ ]]; then
        echo "Raster file detected, regenerating VRT..."
        # Update the VRT file to include all raster files in DOWNLOAD_DIR
        gdalbuildvrt "$VRT_FILE" "$DOWNLOAD_DIR"/*.tif "$DOWNLOAD_DIR"/*.tiff "$DOWNLOAD_DIR"/*.jp2 "$DOWNLOAD_DIR"/*.ecw "$DOWNLOAD_DIR"/*.img

        # Copy all raster files to TARGET_DIR
        echo "Moving raster files to $TARGET_DIR..."
        find "$DOWNLOAD_DIR" -regex ".*\.\(tif\|tiff\|jp2\|ecw\|img\)$" -exec mv {} "$TARGET_DIR/" \;

        # Copy the VRT file to TARGET_DIR *after* moving the raster files
        # as this overwrites the potential old one already there
        echo "Moving VRT file to $TARGET_DIR..."
        cp "$VRT_FILE" "$TARGET_DIR/$VRT_FILE"
    fi
done

exec "$@"
