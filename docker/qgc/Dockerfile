FROM ubuntu:focal

ARG USERNAME=qgroundcontrol
ARG UID=1000
ARG GID=$UID

RUN apt-get update --fix-missing && \
    apt-get install sudo && \
    groupadd --gid $GID $USERNAME && useradd --uid $UID --gid $GID -m $USERNAME && \
    echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME
USER $USERNAME

RUN sudo apt-get update --fix-missing && \
    sudo usermod -a -G dialout $USERNAME && \
    sudo apt-get remove modemmanager -y && \
    sudo apt-get -y install gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl && \
    sudo apt-get -y install libqt5gui5 && \
    sudo apt-get -y install libfuse2 && \
    sudo apt-get -y install libpulse-mainloop-glib0  && \
    sudo apt-get -y install wget

RUN wget -P $HOME https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage && \
    sudo chmod +x $HOME/QGroundControl.AppImage

COPY * /
RUN sudo mv -t $HOME/ /ksql_airport_px4.plan /ksql_airport_ardupilot.plan

ENTRYPOINT ["/bin/bash", "-c", "cd $HOME && ./QGroundControl.AppImage --appimage-extract-and-run >/dev/null"]
