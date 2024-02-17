# Use an Ubuntu base image
FROM ubuntu:jammy

# Avoid prompts from apt and messages from debconf
ENV DEBIAN_FRONTEND=noninteractive

# Install dependencies that are common across both architectures
RUN apt-get update && apt-get install -y \
    gdal-bin \
    inotify-tools \
    unzip \
    wget \
    libasound2 \
    libgtk-3-0 \
    libdbus-glib-1-2 \
    libx11-xcb1 \
    bzip2 \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

# Install Firefox (not via Snap)
# Detect architecture using dpkg (more reliable in Debian-based containers)
RUN ARCH=$(dpkg --print-architecture) && \
    FIREFOX_VERSION=102.0 && \
    if [ "$ARCH" = "amd64" ]; then \
        FIREFOX_URL="https://ftp.mozilla.org/pub/firefox/releases/${FIREFOX_VERSION}/linux-x86_64/en-US/firefox-${FIREFOX_VERSION}.tar.bz2"; \
    elif [ "$ARCH" = "arm64" ]; then \
        FIREFOX_URL="https://ftp.mozilla.org/pub/firefox/releases/${FIREFOX_VERSION}/linux-aarch64/en-US/firefox-${FIREFOX_VERSION}.tar.bz2"; \
    else \
        echo "Unsupported architecture"; exit 1; \
    fi && \
    wget -q ${FIREFOX_URL} -O /tmp/firefox.tar.bz2 && \
    tar xjf /tmp/firefox.tar.bz2 -C /opt/ && \
    ln -s /opt/firefox/firefox /usr/local/bin/firefox && \
    rm /tmp/firefox.tar.bz2

COPY static /srv/downloader
COPY entrypoint.sh /

RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
