System requirements
____________________________________________________

These instructions assume you are running **Ubuntu 22.04 (Jammy Jellyfish)**
on your development machine. Other Ubuntu and Linux releases might also work
with some modifications but are currently unsupported.

It is strongly recommended that your development machine have an
**NVIDIA GPU with CUDA Toolkit and latest drivers**  installed. You can inspect
your NVIDIA driver and CUDA versions with the ``nvidia-smi`` command line utility.
If you don't have it installed, follow the `NVIDIA CUDA Installation Guide for Linux
<https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html>`_.
The output of the ``nvidia-smi`` command should look something like below:

.. code-block:: text
    :caption: Example output of nvidia-smi command

    hmakelin@hmakelin-Nitro-AN515-54:~$ nvidia-smi
    Thu Dec 15 12:00:12 2022
    +-----------------------------------------------------------------------------+
    | NVIDIA-SMI 520.61.05    Driver Version: 520.61.05    CUDA Version: 11.8     |
    |-------------------------------+----------------------+----------------------+
    | GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
    | Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
    |                               |                      |               MIG M. |
    |===============================+======================+======================|
    |   0  NVIDIA GeForce ...  On   | 00000000:01:00.0  On |                  N/A |
    | N/A   73C    P8     7W /  N/A |     69MiB /  6144MiB |     23%      Default |
    |                               |                      |                  N/A |
    +-------------------------------+----------------------+----------------------+

    +-----------------------------------------------------------------------------+
    | Processes:                                                                  |
    |  GPU   GI   CI        PID   Type   Process name                  GPU Memory |
    |        ID   ID                                                   Usage      |
    |=============================================================================|
    |    0   N/A  N/A      1353      G   /usr/lib/xorg/Xorg                 22MiB |
    |    0   N/A  N/A      2210      G   /usr/lib/xorg/Xorg                 45MiB |
    +-----------------------------------------------------------------------------+
