FROM ubuntu:20.04

ARG CONFIG_FILE=yaml/mapproxy.yaml
ARG HOST=localhost
ARG PORT=80
ARG MAPPROXY_TILE_URL

ENV CONFIG_FILE=${CONFIG_FILE}
ENV HOST=${HOST}
ENV PORT=${PORT}
ENV MAPPROXY_TILE_URL=${MAPPROXY_TILE_URL}

# Check that the tile endpoint url was provided as build argument
RUN if [ -z "$MAPPROXY_TILE_URL" ]; \
        then \
            tput setaf 1; \
            echo -e "\$MAPPROXY_TILE_URL is empty! Provide it with --build-arg MAPPROXY_TILE_URL=\"url\"."; \
            tput sgr0; \
            false; \
        else \
            echo "Using ${MAPPROXY_TILE_URL} for tile endpoint."; \
    fi

# Install MapProxy along with required & optional dependencies
RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y python3-pip python3-pil python3-yaml libproj-dev libgeos-dev \
        python3-lxml libgdal-dev python3-shapely && \
    pip3 install MapProxy

# Copy configuration file over to $HOME
# MapProxy config files can't use environment variables so do tile URL substitution manually
COPY ${CONFIG_FILE} /
RUN mv $(basename $CONFIG_FILE) $HOME/$(basename $CONFIG_FILE) && \
    sed -i -e "s#\$MAPPROXY_TILE_URL#$MAPPROXY_TILE_URL#g" $HOME/$(basename $CONFIG_FILE)

# Expose port for incoming WMS requests
EXPOSE $PORT/tcp
