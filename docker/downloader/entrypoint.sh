#!/bin/bash
set -e

DOWNLOAD_DIR="/etc/mapserver"
TARGET_DIR="/etc/mapserver/imagery"

# watch for new files in the download dir
inotifywait -m -e create -e moved_to --format '%f' "${DOWNLOAD_DIR}" | while read FILENAME
do
    if [[ "${FILENAME}" =~ \.zip$ ]]; then
        # unzip automatically into target dir
        mkdir -p "${TARGET_DIR}"
        unzip -o "${DOWNLOAD_DIR}/${FILENAME}" -d "${TARGET_DIR}"
        # Remove the zip file after extraction
        rm "${DOWNLOAD_DIR}/${FILENAME}"
    fi
    # rebuild .VRT file used for imagery layer
    # todo: gdalbuildvrt
done

exec "$@"
