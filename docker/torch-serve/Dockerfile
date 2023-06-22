ARG BASE_IMAGE
FROM ${BASE_IMAGE}

LABEL maintainer="Harri Makelin <hmakelin@protonmail.com>"

COPY --chown=755 loftr_handler.py ts_config /home/model-server/

USER root
RUN apt -y install libsm6

USER model-server
RUN pip3 install kornia==0.6.10 importlib-metadata opencv-python torch-model-archiver

RUN python3 -c "import torch; \
  from kornia.feature import LoFTR; \
  model = LoFTR(pretrained='outdoor'); \
  torch.save(model.state_dict(), 'loftr.pth'); \
  "

# Create LoFTR inference model in torch-serve expected .mar format
# Need to preserve directory structure in .mar for loftr.py imports to work
# Use this workaround: https://github.com/pytorch/serve/issues/566#issuecomment-964340770
RUN TEMP_DIR=$(mktemp -d) && \
  LOFTR_DIR=$(python3 -c "import os;  \
                          from kornia.feature import loftr; \
                          print(os.path.dirname(os.path.abspath(loftr.__file__)))") && \
  ln -s "$LOFTR_DIR" $TEMP_DIR && \
  ln -s "$LOFTR_DIR/backbone" $TEMP_DIR && \
  ln -s "$LOFTR_DIR/utils" $TEMP_DIR && \
  ln -s "$LOFTR_DIR/loftr_module" $TEMP_DIR && \
  torch-model-archiver \
    --model-name loftr \
    --version $(python3 -c "import kornia; \
                            print(kornia.__version__)") \
    --model-file $LOFTR_DIR/loftr.py \
    --serialized-file loftr.pth \
    --export-path model-store \
    --extra-files $TEMP_DIR \
    --handler $HOME/loftr_handler.py && \
    rm -rf $TEMP_DIR

CMD ["torchserve", \
    "--start", \
    "--ncs", \
    "--model-store", "model-store", \
    "--models", "loftr=loftr.mar", \
    "--ts-config", "ts_config"]
