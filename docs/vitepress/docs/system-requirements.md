# System requirements

This is the supported system i.e. development and testing is done on these specs. Other systems may also work but are not tested.

## Simulation host

### Hardware

- **NVIDIA GPU and CUDA drivers** <Badge type="tip" text="Strongly recommended"/>
- **Memory:** 16GB <Badge type="tip" text="Recommended"/>, 8GB should also work but not tested
- **Storage:** 50GB free
    ::: info Storage
    Storage mostly needed for storing Docker images that may include deep learning model weights and map rasters. Current suite of images is quite unoptimized from a storage perspective and improvement could be done here.
    :::

### Operating System

- **Ubuntu 22.04 LTS (Jammy Jellyfish)** <Badge type="tip" text="Recommended"/>
    ::: info Debian
    Most Debian derivatives may work with some modifications, but this guide assumes you are running Ubuntu Jammy.
    :::

## Companion computer

- **Raspberry Pi 5**
    - **Memory**: 8GB
    - **Storage**: 128GB SD card  <Badge type="tip" text="Recommended"/>, 64GB should also work
    - **OS**: Debian and its derivatives <Badge type="tip" text="Recommended"/>
    ::: tip GPU recommended
    The goal is to make GISNav work without a GPU so it is developed and tested on a Raspberry Pi. However, you will likely get better results using e.g. an Nvidia board. There should be no hard dependency on an NVIDIA GPU, but PyTorch will by default upload the model into main (CPU) memory since CUDA is not available.
    :::

    ::: warning Warning: 4GB of memory not enough
    The deep learning model may be too large for the 4GB model. Attempts to make GISNav work on 4GB of memory have been made and may be made in the future.
    :::

## Autopilot FMU (HIL)

- **Pixhawk FMUv4**
    ::: info Tested on NXP RDDRONE-FMUK66 (FMUv4)
    See the HIL instructions [here](/hil-pixhawk).
    :::
    ::: tip GPS 2 serial port
    You may want to look for Pixhawk boards with a dedicated serial port for a secondary GPS (GPS 2).
    :::
