ARG DOCKER_TAG=latest

FROM camptocamp/mapserver:${DOCKER_TAG}

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

COPY mapfiles/ /etc/mapserver/
COPY templates/ /etc/mapserver/

WORKDIR /etc/mapserver

# Install tools to download and unzip maps from Google Drive
RUN apt-get update && \
    apt-get -y install unzip python3-pip wget && \
    pip3 install gdown

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

# Setup complete, start MapServer with default mapfile
COPY entrypoint.sh /

RUN chmod 755 /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
