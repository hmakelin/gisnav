FROM ros:foxy

LABEL maintainer="Harri Makelin <hmakelin@protonmail.com>"

ARG USERNAME=gisnav
ARG UID=1000
ARG GID=$UID

RUN apt-get update --fix-missing && \
    apt-get install sudo && \
    groupadd --gid $GID $USERNAME && useradd --uid $UID --gid $GID -m $USERNAME && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME
USER $USERNAME

RUN sudo apt-get -y install ros-foxy-mavros ros-foxy-mavros-extras && \
    cd $HOME && \
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    sudo bash ./install_geographiclib_datasets.sh

COPY * /
RUN sudo chmod 755 /disable_shared_memory.xml

CMD ros2 run mavros mavros_node --ros-args --param fcu_url:=udp-b://127.0.0.1:14551@14555
#CMD ros2 run mavros mavros_node --ros-args --param fcu_url:=udp-b://127.0.0.1:14540@14550  # px4
