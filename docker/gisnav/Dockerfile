FROM ros:foxy

ARG USERNAME=gisnav
ARG UID=1000
ARG GID=$UID

RUN apt-get update --fix-missing && \
    apt-get install sudo && \
    groupadd --gid $GID $USERNAME && useradd --uid $UID --gid $GID -m $USERNAME && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME
USER $USERNAME

# Install GISNav & colcon dependencies
RUN mkdir -p $HOME/colcon_ws/src && \
    cd $HOME/colcon_ws/src && \
    git clone --branch hmakelin-update-px4-v1.14 https://github.com/hmakelin/gisnav.git && \
    git clone https://github.com/hmakelin/gisnav_msgs.git && \
    git clone https://github.com/px4/px4_msgs.git && \
    rosdep update && \
    rosdep install --from-paths . -y --ignore-src

# Install LoFTR
RUN cd $HOME/colcon_ws/src/gisnav && \
    git submodule update --init gisnav/pose_estimators/third_party/LoFTR && \
    sudo apt-get -y install python3-pip && \
    pip3 install gdown && \
    mkdir weights && \
    cd weights && \
    $HOME/.local/lib/$USERNAME/gdown https://drive.google.com/uc?id=1M-VD35-qdB5Iw-AtbDBCKC7hPolFW9UY

# Python dependencies
RUN cd $HOME/colcon_ws/src/gisnav && \
    pip3 install -r requirements.txt  && \
    pip3 install --upgrade numpy python-dateutil

# GeographicLib
RUN sudo apt-get -y install ros-foxy-mavros ros-foxy-mavros-extras && \
    cd $HOME && \
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    sudo bash ./install_geographiclib_datasets.sh

# Build
SHELL ["/bin/bash", "-c"]
RUN cd $HOME/colcon_ws && \
    source /opt/ros/foxy/setup.bash && \
    colcon build

# Clean
RUN sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/*

COPY * /
RUN sudo chmod 755 /disable_shared_memory.xml
