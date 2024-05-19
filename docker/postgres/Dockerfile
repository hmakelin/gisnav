FROM postgres:14

RUN apt-get update \
    && apt-get -y install postgresql-14-postgis-3 \
    && rm -rf /var/lib/apt/lists/* \
    && apt clean

COPY init-postgis.sh /

# Script to create a PostGIS-enabled database
RUN echo "#!/bin/bash" > /init-postgis.sh \
    && echo "psql --username \$POSTGRES_USER --dbname \$POSTGRES_DB <<-EOSQL" >> /init-postgis.sh \
    && echo "CREATE EXTENSION IF NOT EXISTS postgis;" >> /init-postgis.sh \
    && echo "EOSQL" >> /init-postgis.sh \
    && chmod +x /init-postgis.sh
