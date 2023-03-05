FROM px4io/px4-dev-ros2-foxy

LABEL maintainer="Harri Makelin <hmakelin@protonmail.com>"

ARG USERNAME=gisnav
ARG UID=1000
ARG GID=$UID

ARG ROS_DOMAIN_ID=0
ENV ROS_DOMAIN_ID=${ROS_DOMAIN_ID}

# Set 'gisnav' as default user and add it to sudo'ers
# QGroundControl will not run as 'root'
RUN apt-get update --fix-missing && \
    apt-get install sudo && \
    groupadd --gid $GID $USERNAME && useradd --uid $UID --gid $GID -m $USERNAME && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME
USER $USERNAME

# gstreamer and Gazebo plugins for getting video stream out of gazebo
RUN sudo apt-get -y install ros-foxy-gazebo-ros-pkgs gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl

# Install PX4-Autopilot, pull latest commit from PX4-SITL_gazebo submodule fork
RUN sudo apt-get -y install -o Dpkg::Options::="--force-overwrite" \
        libignition-common3-core-dev libignition-common3-av-dev expect && \
    git clone --branch hmakelin-v1.14.0-beta1 \
        https://github.com/hmakelin/PX4-Autopilot.git $HOME/PX4-Autopilot --recursive && \
    cd $HOME/PX4-Autopilot && \
    git fetch --all --tags && \
    git checkout tags/v1.14.0-1.0.1-beta1 -b gisnav && \
    git submodule update --init --recursive && \
    cd Tools/simulation/gazebo/sitl_gazebo && \
    git pull origin hmakelin-v1.14.0-beta1 && \
    cd $HOME/PX4-Autopilot && \
    bash Tools/setup/ubuntu.sh --no-nuttx

# Make initial PX4 build (for faster startup in the future)
RUN cd $HOME/PX4-Autopilot && \
    yes | DONT_RUN=1 make px4_sitl gazebo_typhoon_h480__ksql_airport

# Apply configuration files
COPY * /
RUN sudo mv -t $HOME/ /6011_typhoon_h480 && \
    cat $HOME/6011_typhoon_h480 >> $HOME/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/6011_typhoon_h480

# Run SITL simulation - answer 'y' to prompt about custom commit in submodule (temp fix until 1.14 stable comes out)
CMD cd ${HOME}/PX4-Autopilot && \
	expect -c \
		"set timeout -1; \
		spawn make px4_sitl gazebo_typhoon_h480__ksql_airport; \
		expect \"Hit 'y' and <ENTER> to continue the build with this version. Hit <ENTER> to resolve manually.\"; \
		send \"y\n\"; \
		interact"
