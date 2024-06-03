FROM postgres:14

# Install PostGIS
RUN apt-get update \
    && apt-get -y install postgresql-14-postgis-3 \
    && rm -rf /var/lib/apt/lists/* \
    && apt clean

# Copy the initialization script to the entrypoint directory
COPY init-gisnav-db.sh /docker-entrypoint-initdb.d/
