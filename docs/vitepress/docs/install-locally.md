# Install locally

This page describes how to install GISNav locally on a development computer. With a local non-containerized installation of GISNav it is easier re-test the software without having to rebuild a Docker image and re-deploy a container every time changes are made to the source code.

::: tip Deploy with Docker Compose
If you are only interested in running and not developing GISNav, you may want to look at the [Docker Compose services](/deploy-with-docker-compose) instead.

:::

## System requirements

These system requirements are intended for SITL simulation on a standalone development computer.

### Operating system

The development computer should be running **Ubuntu 22.04 (Jammy Jellyfish)**. Other Ubuntu and Linux releases may also work with some modifications, but are currently untested and therefore unsupported.

### Hardware

It is strongly recommended that the development computer have an **NVIDIA GPU with CUDA Toolkit and latest drivers** installed.

::: info No hard dependency on NVIDIA
The GISNav ROS 2 package itself depends on [Torch](/glossary#torch-pytorch) and should not have a direct dependency on NVIDIA products. GPUs from other vendors may work but also have not been tested. Some of the GISNav Docker Compose services on the other hand may have a direct dependency on NVIDIA.

:::

You can inspect your NVIDIA driver and CUDA versions with the `nvidia-smi` command line utility. If you don't have it installed, follow the [NVIDIA CUDA Installation Guide for Linux](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html). The output of the `nvidia-smi` command should look something like below:

```console
hmakelin@hmakelin-MS-7D48:~/colcon_ws$ nvidia-smi
Sat May  4 08:37:42 2024
+---------------------------------------------------------------------------------------+
| NVIDIA-SMI 535.171.04             Driver Version: 535.171.04   CUDA Version: 12.2     |
|-----------------------------------------+----------------------+----------------------+
| GPU  Name                 Persistence-M | Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp   Perf          Pwr:Usage/Cap |         Memory-Usage | GPU-Util  Compute M. |
|                                         |                      |               MIG M. |
|=========================================+======================+======================|
|   0  NVIDIA GeForce RTX 4060        Off | 00000000:01:00.0  On |                  N/A |
|  0%   42C    P8              N/A / 115W |    506MiB /  8188MiB |     23%      Default |
|                                         |                      |                  N/A |
+-----------------------------------------+----------------------+----------------------+

+---------------------------------------------------------------------------------------+
| Processes:                                                                            |
|  GPU   GI   CI        PID   Type   Process name                            GPU Memory |
|        ID   ID                                                             Usage      |
|=======================================================================================|
|    0   N/A  N/A      1918      G   /usr/lib/xorg/Xorg                          228MiB |
|    0   N/A  N/A      2320      G   /usr/bin/gnome-shell                         35MiB |
|    0   N/A  N/A      4528      G   ...irefox/4173/usr/lib/firefox/firefox      162MiB |
|    0   N/A  N/A    118428      G   ...erProcess --variations-seed-version       47MiB |
|    0   N/A  N/A    148095      G   ...erProcess --variations-seed-version       20MiB |
+---------------------------------------------------------------------------------------+
```

## Install ROS 2 Humble

GISNav is a ROS 2 package, and the supported version is ROS 2 Humble. Install ROS 2 Humble by following the official [install instructions](https://docs.ros.org/en/humble/Installation.html).

## Setup `colcon` workspace

Colcon is a ROS build tool and needed to build GISNav and some of its dependencies from source code.

### Create workspace

<!--@include: ./shared/create-colcon-workspace.md-->

### Clone GISNav and dependencies

<!--@include: ./shared/clone-to-colcon-workspace.md-->

### Source workspace

<!--@include: ./shared/source-colcon-workspace.md-->

## Install system dependencies

Install system dependencies for your workspace with the following commands:

```bash
cd ~/colcon_ws/src
rosdep update
rosdep install --from-paths . -y -r --ignore-src
```

::: tip ROS EOL distros
If you want to use a ROS distribution that has reached end-of-life like Foxy, you can provide the `--include-eol-distros` option to `rosdep update`.

:::

## Install Python dependencies

::: warning Warning: Python virtual environments
If you want to use a Python virtual environment for your Python
dependencies, check out [this issue](https://github.com/ros2/ros2/issues/1094) first. We do not provide instructions here
as the workspace might not play nicely with the virtual environment.

:::

GISNav's Python dependencies are divided into [core](./glossary#core-core-functionality), [extended](./glossary#extension-extended-functionality), and development dependencies. You must at least install the core dependencies.

- If you know you are not going to use a specific extension such as `NMEANode`, you can skip installing the corresponding Python [extra](/glossary#extra). `NMEANode` is required for the [mock GPS demo](/sitl-local) and enables downstream integration of GISNav as a secondary GPS device via the NMEA protocol.

- The development dependencies are required for various development tasks such as generating documentation and running tests. You do not need to install them if you do not plan to do any development work on GISNav.

Install the required and optional Python dependencies with the following commands:

::: code-group

```bash [Core]
cd ~/colcon_ws/src/gisnav
pip3 install ./gisnav
```

```bash [Extended <Badge type="info" text="Optional"/>]
cd ~/colcon_ws/src/gisnav
pip3 install ./gisnav[nmea_node,uorb_node]
```

```bash [Development <Badge type="info" text="Optional"/>]
cd ~/colcon_ws/src/gisnav
pip3 install ./gisnav[dev]
```

:::

## Build workspace

<!--@include: ./shared/build-colcon-workspace.md-->

Once GISNav is installed, you can try [deploying the development services](/deploy-for-development).
