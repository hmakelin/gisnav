FROM ros:foxy

LABEL maintainer="Harri Makelin <hmakelin@protonmail.com>"

ARG USERNAME=gisnav
ARG UID=1000
ARG GID=$UID

# Set 'gisnav' as default user and add it to sudo'ers
RUN apt-get update --fix-missing && \
    apt-get install -y sudo wget && \
    groupadd --gid $GID $USERNAME && useradd --uid $UID --gid $GID -m $USERNAME && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME
USER $USERNAME

# Install required tools
RUN sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main"  \
      > /etc/apt/sources.list.d/gazebo-stable.list' && \
    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - && \
    sudo apt-get update && \
    sudo DEBIAN_FRONTEND=noninteractive apt-get -y install git gitk git-gui python3-pip python-is-python3 gazebo11 \
      gazebo11-common libgazebo11-dev libgazebo11 ros-foxy-gazebo-ros-pkgs ros-foxy-gazebo-plugins ros-foxy-gazebo-dev \
      ros-foxy-gazebo-ros && \
    pip install --upgrade pymavlink MAVProxy pexpect

# Clone ArduPilot and update submodules
RUN cd $HOME && \
    git clone https://github.com/ArduPilot/ardupilot && \
    cd ardupilot && \
    git submodule update --init --recursive

# Install ArduPilot prerequisites
# The script requires the USER env variable be set to ${USERNAME}
SHELL ["/bin/bash", "-c"]
RUN cd $HOME/ardupilot/ && \
    export USER=$USERNAME && \
    sudo DEBIAN_FRONTEND=noninteractive apt-get install -y keyboard-configuration && \
    Tools/environment_install/install-prereqs-ubuntu.sh -y

# Update PATH
ENV PATH="/usr/lib/ccache:/home/$USERNAME/.local/bin:${PATH}:/home/$USERNAME/ardupilot/Tools/autotest"

# Install Gazebo plugin
RUN cd $HOME && \
    git clone https://github.com/hmakelin/ardupilot_gazebo && \
    cd ardupilot_gazebo && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j4 && \
    sudo make install && \
    echo 'source /usr/share/gazebo/setup.sh' >> ~/.bashrc && \
    echo 'export GAZEBO_MODEL_PATH=~/ardupilot_gazebo/models' >> ~/.bashrc && \
    echo 'export GAZEBO_RESOURCE_PATH=~/ardupilot_gazebo/worlds:${GAZEBO_RESOURCE_PATH}' >> ~/.bashrc

# Make initial build
RUN cd $HOME/ardupilot && \
    source ~/.profile && \
    $(sim_vehicle.py -v ArduCopter -f gazebo-iris -L KSQL_Airport | grep -m 1 "BUILD SUMMARY") || echo "Gazebo built."

# Copy and apply configuration files, add SITL simulation demo location
COPY * /
RUN sudo mv -t $HOME/ /gazebo-iris.parm /Makefile && \
    cat $HOME/gazebo-iris.parm >> $HOME/ardupilot/Tools/autotest/default_params/gazebo-iris.parm && \
    echo "KSQL_Airport=37.523640,-122.255122,1.7,90" >> $HOME/ardupilot/Tools/autotest/locations.txt

CMD cd $HOME && make sim_vehicle
