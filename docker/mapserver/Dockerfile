FROM ubuntu:jammy AS apache

# Update apt repository and install necessary packages
RUN apt-get update && apt-get install -y \
    cgi-mapserver \
    mapserver-bin \
    apache2 \
    inotify-tools \
    libapache2-mod-fcgid \
    gdal-bin \
    libgdal-dev \
    unzip \
    wget \
    && rm -rf /var/lib/apt/lists/*

FROM apache AS mapserver

ARG NAIP_GDOWN_ID=16M_kbsLpF3t87KC2n9YgqGEBA5h0lG7U
ARG OSM_GDOWN_ID=1snGYjWxs71m6I-qxKsmUzch_bAfQtrEW
ENV NAIP_GDOWN_ID=${NAIP_GDOWN_ID}
ENV OSM_GDOWN_ID=${OSM_GDOWN_ID}

# These do not have to match the uploaded file name in Google Drive
ARG NAIP_ZIP_FILENAME="usda-fsa-naip-san-mateo-ca-2020.zip"
ARG OSM_ZIP_FILENAME="osm-buildings-ksql-airport.zip"
ARG VRT_FILENAME="naip.vrt"
ARG OSM_TIF_FILENAME="osm-buildings-ksql-airport.tif"
ARG DEM_FILENAME="USGS_13_n38w123_20220810.tif"
ARG DEM_4326_FILENAME="USGS_13_n38w123_20220810__EPSG_4326.tif"
ARG COMPOUND_FILENAME="osm-buildings-dem.tif"

ENV DEBIAN_FRONTEND=noninteractive

# Install tools to download and unzip maps from Google Drive
RUN apt-get update && \
    apt-get -y install python3-pip && \
    pip3 install gdown

WORKDIR /etc/mapserver

# Download NAIP raster imagery
RUN gdown $NAIP_GDOWN_ID -O $NAIP_ZIP_FILENAME && \
    unzip $NAIP_ZIP_FILENAME && \
    rm $NAIP_ZIP_FILENAME

# Download OSM Buildings vector data
RUN gdown $OSM_GDOWN_ID -O $OSM_ZIP_FILENAME && \
    unzip $OSM_ZIP_FILENAME && \
    rm $OSM_ZIP_FILENAME

# Create VRT file from NAIP GeoTIFFs
# VRT file name should match with what is configured in Mapfile
RUN gdalbuildvrt $VRT_FILENAME *.tif

# Rasterize OSM Buildings vectors into a VRT file
RUN gdal_rasterize \
    -a height \
    -ts $(gdalinfo $VRT_FILENAME |grep "Size is" |cut -d\  -f3-4 |sed "s/,//") \
    osm-buildings-ksql-airport.geojson $OSM_TIF_FILENAME

# Download USGS DEM: https://www.sciencebase.gov/catalog/item/62f5de86d34eacf53973ab2a
# Attribution: "Map services and data available from U.S. Geological Survey, National Geospatial Program."
# Reproject to WGS 84 using gdalwarp
RUN wget "https://prd-tnm.s3.amazonaws.com/StagedProducts/Elevation/13/TIFF/historical/n38w123/USGS_13_n38w123_20220810.tif" && \
    cp $OSM_TIF_FILENAME $DEM_4326_FILENAME && \
    gdalwarp $DEM_FILENAME $DEM_4326_FILENAME # -s_srs EPSG:4269 -t_srs EPSG:4326

# Calculate compound elevation layer as sum of OSM Buildings and USGS DEM (units are meters)
# Assume DEM is low resolution enough to not model the buildings already included in OSM Buildings
RUN gdal_calc.py -A $DEM_4326_FILENAME -B $OSM_TIF_FILENAME --outfile=$COMPOUND_FILENAME --calc="A+B" --extent=intersect

COPY mapfiles/ /etc/mapserver/
COPY templates/ /etc/mapserver/

# Enable Apache modules
RUN a2enmod rewrite && a2enmod fcgid && a2enmod cgi

# Configure Apache to serve the MapServer CGI
# Add .cgi suffix to the executable to make Apache correctly handle the
# requests
RUN echo "<Directory /usr/lib/cgi-bin/>" > /etc/apache2/conf-available/mapserver.conf \
    && echo "    Options +ExecCGI -MultiViews +SymLinksIfOwnerMatch" >> /etc/apache2/conf-available/mapserver.conf \
    && echo "    AddHandler fcgid-script .fcgi" >> /etc/apache2/conf-available/mapserver.conf \
    && echo "    AddHandler cgi-script .cgi .pl" >> /etc/apache2/conf-available/mapserver.conf \
    && echo "    Require all granted" >> /etc/apache2/conf-available/mapserver.conf \
    && echo "</Directory>" >> /etc/apache2/conf-available/mapserver.conf \
    && echo "ScriptAlias /cgi-bin/ /usr/lib/cgi-bin/" >> /etc/apache2/conf-available/mapserver.conf \
    && a2enconf mapserver \
    && mv /usr/lib/cgi-bin/mapserv /usr/lib/cgi-bin/mapserv.cgi \
    && chmod +x /usr/lib/cgi-bin/mapserv.cgi

# Setup complete, start MapServer with default mapfile
COPY entrypoint.sh /

RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

FROM apache AS fileserver

# Avoid prompts from apt and messages from debconf
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y \
    php \
    libapache2-mod-php \
    php-zip \
    php-mbstring \
    php-dom \
    php-xml \
    libasound2 \
    libgtk-3-0 \
    libdbus-glib-1-2 \
    libx11-xcb1 \
    bzip2 && \
    rm -rf /var/lib/apt/lists/*

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

RUN mkdir -p /var/www/ && \
    cd /var/www && \
    wget https://github.com/filegator/static/raw/master/builds/filegator_latest.zip && \
    unzip filegator_latest.zip && \
    rm filegator_latest.zip && \
    chown -R www-data:www-data filegator && \
    chmod -R 775 filegator

RUN echo "<VirtualHost *:80>" > /etc/apache2/sites-available/filegator.conf && \
    echo "  DocumentRoot /var/www/filegator/dist" >> /etc/apache2/sites-available/filegator.conf && \
    echo "</VirtualHost>" >> /etc/apache2/sites-available/filegator.conf

RUN a2dissite 000-default.conf && a2ensite filegator.conf

# Setup complete, start MapServer with default mapfile
COPY fs_entrypoint.sh /

RUN chmod +x fs_entrypoint.sh

ENTRYPOINT ["/fs_entrypoint.sh"]
