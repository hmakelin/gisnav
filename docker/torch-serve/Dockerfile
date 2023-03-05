ARG BASE_IMAGE

FROM ${BASE_IMAGE}

LABEL maintainer="Harri Makelin <hmakelin@protonmail.com>"

COPY --chmod=755 create_pth.py setup.sh $HOME
COPY loftr_handler.py ts_config /home/model-server/

RUN $HOME/setup.sh

CMD ["torchserve", \
    "--start", \
    "--ncs", \
    "--model-store", "model_store", \
    "--models", "loftr=loftr.mar", \
    "--ts-config", "$HOME/ts_config"]