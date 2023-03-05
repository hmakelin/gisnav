# Install OpenCV
apt-get update && \
apt-get -y install sudo && \
sudo DEBIAN_FRONTEND="noninteractive" apt-get -y install python3-opencv

# Install kornia which comes with LoFTR + dependencies
pip3 install kornia importlib-metadata

# Create model directories and .pth file
cd /home/model-server && \
mkdir model_defs && \
mkdir model_store && \
python3 create_pth.py && \
cp loftr.pth model_defs

# Create LoFTR inference model in torch-serve expected .mar format
# Need to preserve directory structure in .mar for loftr.py imports to work
# Use this workaround: https://github.com/pytorch/serve/issues/566#issuecomment-964340770
pip install torch-model-archiver && \
TEMP_DIR=$(mktemp -d) && \
LOFTR_DIR=$(python3 -c "import os; from kornia.feature import loftr; \
            print(os.path.dirname(os.path.abspath(loftr.__file__)))") && \
  ln -s "$LOFTR_DIR" $TEMP_DIR && \
  ln -s "$LOFTR_DIR/backbone" $TEMP_DIR && \
  ln -s "$LOFTR_DIR/utils" $TEMP_DIR && \
  ln -s "$LOFTR_DIR/loftr_module" $TEMP_DIR && \
  torch-model-archiver \
    --model-name loftr \
    --version $(python3 -c "import kornia; print(kornia.__version__)") \
    --model-file $LOFTR_DIR/loftr.py \
    --serialized-file model_defs/loftr.pth \
    --export-path model_store \
    --extra-files $TEMP_DIR \
    --handler /home/model-server/loftr_handler.py && \
    rm -rf $TEMP_DIR
