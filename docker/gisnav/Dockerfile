FROM ros:foxy

ARG USERNAME=gisnav
ARG UID=1000
ARG GID=$UID

RUN apt-get update --fix-missing && \
    apt-get -y dist-upgrade && \
    apt-get -y install sudo && \
    groupadd --gid $GID $USERNAME && useradd --uid $UID --gid $GID -m $USERNAME && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME

USER $USERNAME

WORKDIR /home/$USERNAME/colcon_ws/src/gisnav
COPY --chown=$USERNAME . .

RUN sudo chmod +x docker/gisnav/entrypoint.sh

# Install GISNav dependencies
RUN rosdep update && \
    rosdep install --from-paths . -y -r --ignore-src && \
    sudo apt-get -y install python3-pip && \
    pip install -r requirements.txt

# Install GDAL
RUN sudo apt-get -y install software-properties-common && \
    sudo add-apt-repository ppa:ubuntugis/ppa && \
    sudo apt-get -y upgrade && \
    sudo apt-get update && \
    sudo apt-get -y install gdal-bin libgdal-dev

# Install GeographicLib
RUN sudo apt-get -y install ros-foxy-mavros ros-foxy-mavros-extras && \
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh && \
    sudo bash ./install_geographiclib_datasets.sh

WORKDIR /home/$USERNAME/colcon_ws/src

# Custom ROS messages
RUN git clone https://github.com/hmakelin/gisnav_msgs.git && \
    git clone https://github.com/px4/px4_msgs.git

WORKDIR /home/$USERNAME/colcon_ws

# Build
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/foxy/setup.bash && \
    colcon build

# Clean
RUN sudo apt-get clean && \
    sudo rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["src/gisnav/docker/gisnav/entrypoint.sh"]
