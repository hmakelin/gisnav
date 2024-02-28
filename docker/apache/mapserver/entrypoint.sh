#!/bin/bash
# This script automatically manages raster files (such as TIFF, JP2, ECW, and IMG formats)
# within a specified download directory (/etc/mapserver) for use with MapServer. It listens
# for file creation, movement, and deletion events in the imagery and dem directories. When
# a new ZIP file is added to either directory, the script unzips its contents into the same
# directory and then removes the original ZIP file. If a raster file is added directly or
# extracted from a ZIP file, the script regenerates a Virtual Raster (VRT) file to include
# all raster files present in the directory.The script cleans up by removing the original
# potential .zip files from the directories after extraction. This process ensures that
# MapServer always serves the latest raster data available in the directories.
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

process_directory_change() {
    DIR_PATH=$1
    VRT_FILE=$2
    FILE=$3

    echo "Detected change in file: $FILE"
    BASENAME=$(basename "$FILE")

    # Skip temporary download files
    if [[ "$BASENAME" =~ \.part$|\.download$ ]]; then
        echo "Ignoring partial download file: $FILE"
        return
    fi

    if [[ "$BASENAME" =~ \.zip$ ]]; then
        echo "Attempting to unzip $FILE..."
        sleep 0.5  # completed .zip downloads might not be processed correctly without this sleep
        if ! unzip -o "$FILE" -d "$DIR_PATH" > /dev/null 2>&1; then
            echo "Unzip failed, possibly due to incomplete download: $FILE. Will retry on next trigger."
            return
        else
            echo "Unzipped successfully: $FILE"
            rm "$FILE"
        fi
    fi

    sleep 0.5

    if [[ "$BASENAME" =~ \.(tif|tiff|jp2|ecw|img)$ ]]; then
        echo "Raster file detected, regenerating VRT."
        cd /etc/mapserver && gdalbuildvrt "$VRT_FILE" "$DIR_PATH"/*.tif "$DIR_PATH"/*.tiff "$DIR_PATH"/*.jp2 "$DIR_PATH"/*.ecw "$DIR_PATH"/*.img
    fi
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

exec "$@"
