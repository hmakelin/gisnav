#!/bin/bash
set -e
DOWNLOAD_DIR="/etc/mapserver"

# watch for new files in the mapped volume and unzip automatically
inotifywait -m -e create -e moved_to --format '%f' "${DOWNLOAD_DIR}" | while read FILENAME
do
    if [[ "${FILENAME}" =~ \.zip$ ]]; then
        unzip -o "${DOWNLOAD_DIR}/${FILENAME}" -d "${DOWNLOAD_DIR}"
        # Remove the zip file after extraction
        rm "${DOWNLOAD_DIR}/${FILENAME}"
    fi
done

exec "$@"

