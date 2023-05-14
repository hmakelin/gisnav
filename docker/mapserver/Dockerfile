ARG DOCKER_TAG=latest

FROM camptocamp/mapserver:${DOCKER_TAG}

COPY setup_mapserver.sh /etc/mapserver/setup_mapserver.sh

ARG NAIP_GDOWN_ID=16M_kbsLpF3t87KC2n9YgqGEBA5h0lG7U
ARG OSM_GDOWN_ID=1snGYjWxs71m6I-qxKsmUzch_bAfQtrEW
ENV NAIP_GDOWN_ID=${NAIP_GDOWN_ID}
ENV OSM_GDOWN_ID=${OSM_GDOWN_ID}

COPY mapfiles/ /etc/mapserver/
COPY templates/ /etc/mapserver/

RUN /etc/mapserver/setup_mapserver.sh

# Setup complete, start MapServer with default mapfile
COPY entrypoint.sh /

RUN chmod 755 /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
