FROM ubuntu:jammy AS apache
ARG TINYOWS_MAPFILE

# Update apt repository and install necessary packages
RUN apt-get update && apt-get install -y \
    apache2 \
    inotify-tools \
    gdal-bin \
    libgdal-dev \
    unzip \
    wget \
    && rm -rf /var/lib/apt/lists/*

FROM apache AS mapserver

# Install TinyOWS dependencies
RUN apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y \
      build-essential \
      autoconf \
      automake \
      libtool \
      libxml2-dev \
      libgeos-dev \
      postgresql-server-dev-all \
      postgis \
      libfcgi-dev \
      libfcgi0ldbl \
      libpq-dev \
      flex \
      git \
      cgi-mapserver \
      mapserver-bin \
      mapcache-cgi \
      libapache2-mod-mapcache \
      libapache2-mod-fcgid \
    && rm -rf /var/lib/apt/lists/*

# Clone, build, and install TinyOWS
# Need to copy postgis3.2 contrib to new location
RUN git clone https://github.com/MapServer/tinyows.git \
    && cd tinyows \
    && autoconf \
    && ./configure \
    && make \
    && make install \
    && cp tinyows /usr/lib/cgi-bin

ARG NAIP_GDOWN_ID=16M_kbsLpF3t87KC2n9YgqGEBA5h0lG7U
ARG OSM_GDOWN_ID=1snGYjWxs71m6I-qxKsmUzch_bAfQtrEW
ENV NAIP_GDOWN_ID=${NAIP_GDOWN_ID}

# These do not have to match the uploaded file name in Google Drive
ARG NAIP_ZIP_FILENAME="usda-fsa-naip-san-mateo-ca-2020.zip"
ARG DEM_FILENAME="USGS_13_n38w123_20220810.tif"

ENV DEBIAN_FRONTEND=noninteractive
ENV NAIP_ZIP_FILENAME=${NAIP_ZIP_FILENAME}
ENV DEM_FILENAME=${DEM_FILENAME}

# Install tools to download and unzip maps from Google Drive
RUN apt-get update && \
    apt-get -y install python3-pip && \
    pip3 install gdown

WORKDIR /etc/mapserver

# Download NAIP raster imagery
RUN gdown $NAIP_GDOWN_ID -O $NAIP_ZIP_FILENAME

# Download USGS DEM: https://www.sciencebase.gov/catalog/item/62f5de86d34eacf53973ab2a
# Attribution: "Map services and data available from U.S. Geological Survey, National Geospatial Program."
# Reproject to WGS 84 using gdalwarp
RUN wget "https://prd-tnm.s3.amazonaws.com/StagedProducts/Elevation/13/TIFF/historical/n38w123/USGS_13_n38w123_20220810.tif"
    #cp $OSM_TIF_FILENAME $DEM_4326_FILENAME && \
    #gdalwarp $DEM_FILENAME $DEM_4326_FILENAME # -s_srs EPSG:4269 -t_srs EPSG:4326

COPY mapserver/mapfiles/ /etc/mapserver/

RUN mkdir -p /var/cache/mapcache

# Enable Apache modules
RUN a2enmod rewrite && a2enmod fcgid && a2enmod cgi

# Configure Apache to serve the MapServer CGI and TinyOWS
RUN echo "<Directory /usr/lib/cgi-bin/>" > /etc/apache2/conf-available/mapserver.conf \
    && echo "    Options +ExecCGI -MultiViews +SymLinksIfOwnerMatch" >> /etc/apache2/conf-available/mapserver.conf \
    && echo "    AddHandler fcgid-script .fcgi" >> /etc/apache2/conf-available/mapserver.conf \
    && echo "    AddHandler cgi-script .cgi .pl" >> /etc/apache2/conf-available/mapserver.conf \
    && echo "    Require all granted" >> /etc/apache2/conf-available/mapserver.conf \
    #&& echo "    SetEnv TINYOWS_MAPFILE $TINYOWS_MAPFILE" >> /etc/apache2/conf-available/mapserver.conf \
    && echo "</Directory>" >> /etc/apache2/conf-available/mapserver.conf \
    && echo "ScriptAlias /cgi-bin/ /usr/lib/cgi-bin/" >> /etc/apache2/conf-available/mapserver.conf \
    && a2enconf mapserver \
    && mv /usr/lib/cgi-bin/mapserv /usr/lib/cgi-bin/mapserv.cgi \
    && mv /usr/lib/cgi-bin/tinyows /usr/lib/cgi-bin/tinyows.cgi \
    && ln -s /usr/lib/cgi-bin/tinyows.cgi /usr/lib/cgi-bin/tinyows \
    && chmod +x /usr/lib/cgi-bin/mapserv.cgi \
    && chmod +x /usr/lib/cgi-bin/tinyows.cgi

# Setup complete, start MapServer with default mapfile
COPY mapserver/entrypoint.sh /
COPY mapserver/etc/ /etc/

RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]

FROM apache AS fileserver

# Avoid prompts from apt and messages from debconf
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && \
    apt-get install -y \
    php \
    jq \
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

# Setup filegator. Granting guest user permissions here will skip the login
# screen on launch.
# TODO clone specific version and build with less brittle customizations
RUN mkdir -p /var/www/ && \
    cd /var/www && \
    wget https://github.com/filegator/static/raw/master/builds/filegator_latest.zip && \
    unzip filegator_latest.zip && \
    rm filegator_latest.zip && \
    chown -R www-data:www-data filegator && \
    chmod -R 775 filegator && \
    cd filegator && \
    cp configuration_sample.php configuration.php && \
    sed -i "/'editable' => \[/ s/]/, '.yml', '.yaml'&/" configuration.php && \
    sed -i 's/100 \* 1024 \* 1024/2 \* 1024 \* 1024 \* 1024/g' configuration.php && \
    jq '.["2"].permissions = "read|write|upload|download|zip"' private/users.json.blank > temp.json && mv temp.json private/users.json


RUN echo "<VirtualHost *:80>" > /etc/apache2/sites-available/filegator.conf && \
    echo "  DocumentRoot /var/www/filegator/dist" >> /etc/apache2/sites-available/filegator.conf && \
    echo "</VirtualHost>" >> /etc/apache2/sites-available/filegator.conf

RUN a2dissite 000-default.conf && a2ensite filegator.conf

# Setup complete, start MapServer with default mapfile
COPY fileserver/entrypoint.sh /

RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
