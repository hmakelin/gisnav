FROM postgres:14

# Install PostGIS
RUN apt-get update \
    && apt-get -y install postgresql-14-postgis-3 \
    && rm -rf /var/lib/apt/lists/* \
    && apt clean

# Copy the initialization scripts to the entrypoint directory
COPY init-gisnav-db.sh /docker-entrypoint-initdb.d/
COPY clear-feature-position.sql /docker-entrypoint-initdb.d/
