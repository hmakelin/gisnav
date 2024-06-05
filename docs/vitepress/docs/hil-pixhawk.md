# Pixhawk FMU HIL

<!--@include: ./shared/warning-simulation-use-only.md-->

::: warning Warning: Propellers off
If your autopilot is connected to a vehicle, it is highly recommended to disable the propellers or take other necessary precautions to ensure the vehicle remains grounded.

:::

This page describes how to run HIL simulation on a Pixhawk board and a companion computer.

## Prerequisites

### Simulation host

<!--@include: ./shared/clone-to-colcon-workspace.md-->

::: info todo
Instructions to clone only the docker part (e.g. as a submodule).

:::

### Raspberry Pi 5

#### Docker Compose plugin

<!--@include: ./shared/docker-compose-required.md-->

::: tip Install Docker Engine on Debian
Take a look at Docker's [official instructions](https://docs.docker.com/engine/install/debian/) for installing Docker Engine on a Debian-based system.

:::

Make sure you have added your user to the `docker` group as described in the [post-installation steps](https://docs.docker.com/engine/install/linux-postinstall/) to ensure you can run `docker` without `sudo`.

#### systemd

The Raspberry Pi 5 must be running a Linux distro that uses `systemd` such as Debian or one of its derivatives like Raspberry Pi OS or Ubuntu.

### Connectivity & networking

- You need the `ssh` server enabled on your Raspberry Pi 5. Include your own `ssh` public key in the `~/.ssh/authorized_keys` file on the Pi to ensure you can `ssh` in.

- These instructions assume you are using the default hostname `raspberrypi.local` from the `rpi-imager` tool.

- Your development host and Raspberry Pi 5 must be on the same local network. You can e.g. connect them with an Ethernet cable. You may want to share Internet connection to the Raspberry Pi 5 if you want to download the Debian package directly from the Internet onto the Pi.

## Install GISNav CLI

::: info Building your own `.deb`
Instead of installing the `gisnav` Debian package from the public registry, you can also build your own `.deb` file by following [these instructions](/create-debian).

Once you have the `.deb` file built locally or built remotely and moved to the Raspberry Pi 5 (e.g. using `ssh`), you can install `gisnav` using the following command:
```bash
sudo apt-get -y install ./gisnav_*_all.deb

```
:::

Open an `ssh` shell to your Raspberry Pi 5:

```bash
ssh raspberrypi.local
```

<!--@include: ./shared/install-debian.md-->


## Manage GISNav Compose Services

### Enable

```bash
sudo systemctl enable gisnav.service
```

### Start

Once enabled, the `gisnav.service` should start automatically when the system is started. Restart the system or start the `gisnav` service manually to make it active:

```bash
sudo systemctl start gisnav.service
```

The below example shows how you can check that the `gisnav` service is active:

```console
hmakelin@hmakelin-MS-7D48:~$ sudo systemctl status gisnav.service
● gisnav.service - GISNav Docker Compose Services
     Loaded: loaded (/etc/systemd/system/gisnav.service; enabled; vendor preset: enabled)
     Active: active (exited) since Wed 2024-05-15 15:10:21 BST; 3min 35s ago
   Main PID: 241948 (code=exited, status=0/SUCCESS)
        CPU: 354ms

May 15 15:10:18 hmakelin-MS-7D48 docker[241971]:  Container gisnav-mavros-1  Started
May 15 15:10:18 hmakelin-MS-7D48 docker[241971]:  Container gisnav-gscam-1  Started
May 15 15:10:18 hmakelin-MS-7D48 docker[241971]:  Container gisnav-micro-ros-agent-1  Started
May 15 15:10:18 hmakelin-MS-7D48 docker[241971]:  Container gisnav-px4-1  Starting
May 15 15:10:18 hmakelin-MS-7D48 docker[241971]:  Container gisnav-mapserver-1  Started
May 15 15:10:19 hmakelin-MS-7D48 docker[241971]:  Container gisnav-postgres-1  Started
May 15 15:10:20 hmakelin-MS-7D48 docker[241971]:  Container gisnav-px4-1  Started
May 15 15:10:20 hmakelin-MS-7D48 docker[241971]:  Container gisnav-gisnav-1  Starting
May 15 15:10:21 hmakelin-MS-7D48 docker[241971]:  Container gisnav-gisnav-1  Started
May 15 15:10:21 hmakelin-MS-7D48 systemd[1]: Finished GISNav Docker Compose Services.

```

You can also see the service status from the onboard [Admin portal](/admin-portal).

### Stop

You can use the below commands to stop the service (and the related Docker Compose services):
```bash
sudo systemctl stop gisnav.service
```

### Uninstall

If you want to uninstall the service, use the below command:

```bash
sudo apt-get remove gisnav
```

### GSINav CLI

You can also try using the `gnc` command line client that comes with the Debian package to start and stop the services:

```bash
companion_host=raspberrypi.local
export GISNAV_COMPANION_HOST=$companion_host
export PX4_VIDEO_HOST_IP=$companion_host
gnc start px4 gisnav@raspberrypi.local
```

```bash
# In this case we need the "" to also stop on localhost, could also use @localhost
gnc stop "" @raspberrypi.local
```

## Connect Raspberry Pi 5 and Pixhawk

- We connect our development computer to the Raspberry Pi 5 over Ethernet. This is so that we can upload the containers implementing required onboard services.

- This board does not have a `GPS 2` port, so we use the `TELEM 1` port typically reserved for MAVLink communication with GCS for the uORB mock GPS messages.

- We connect the simulation host computer (assumed to be the same as the development computer but strictly speaking these could be separate computers.)


### Connection diagram

```mermaid
graph TB
    subgraph "FMUK66-E (Pixhawk FMUv4)"
        subgraph "TELEM 1"
            FMU_TELEM1_RX[RX]
            FMU_TELEM1_TX[TX]
            FMU_TELEM1_GND[GND]
        end
        subgraph "TELEM 2"
            FMU_TELEM2_RX[RX]
            FMU_TELEM2_TX[TX]
            FMU_TELEM2_GND[GND]
        end
        FMU_USB[USB Micro-B]
    end
    subgraph "Development host"
        Laptop_ETH[Ethernet]
        Laptop_USB[USB]
    end
    subgraph "Raspberry Pi 5"
        subgraph USB["USB-A (x4)"]
            Pi_USB_MAVLink[USB-A]
            Pi_USB_Mouse[USB-A]
            Pi_USB_Keyboard[USB-A]
            Pi_USB_NMEA[USB-A]
        end
        Pi_USB_C["USB-C"]
        Pi_HDMI[HDMI]
        Pi_ETH[Ethernet]
    end
    subgraph "USB to UART NMEA"
        Converter_RX_NMEA[RX]
        Converter_TX_NMEA[TX]
        Converter_GND_NMEA[GND]
        Converter_USB_NMEA[USB-A]
    end
    subgraph "USB to UART MAVLink"
        Converter_RX_MAVLink[RX]
        Converter_TX_MAVLink[TX]
        Converter_GND_MAVLink[GND]
        Converter_USB_MAVLink[USB-A]
    end
    Socket[Power Supply]
    subgraph "Peripherals (optional)"
        Display[Display]
        Mouse[USB Mouse]
        Keyboard[USB Keyboard]
    end
    FMU_TELEM2_TX --- Converter_RX_MAVLink
    FMU_TELEM2_RX --- Converter_TX_MAVLink
    FMU_TELEM2_GND --- Converter_GND_MAVLink
    FMU_TELEM1_TX --- Converter_RX_NMEA
    FMU_TELEM1_RX --- Converter_TX_NMEA
    FMU_TELEM1_GND --- Converter_GND_NMEA
    FMU_USB ---|PX4 firmware\n/dev/ttyACM0\n/dev/serial/by-id/usb-NXP_SEMICONDUCTORS_PX4_FMUK66_E_0-if00| Laptop_USB
    Converter_USB_MAVLink ---|MAVLink| Pi_USB_MAVLink
    Converter_USB_NMEA ---|NMEA 0183| Pi_USB_NMEA
    Pi_USB_C --- Socket
    Pi_HDMI --- Display
    Pi_USB_Mouse --- Mouse
    Pi_USB_Keyboard --- Keyboard
    Pi_ETH ---|ssh| Laptop_ETH
```

## Upload PX4 firmware

See the [PX4 uploading firmware instructions](https://docs.px4.io/main/en/dev_setup/building_px4.html#uploading-firmware-flashing-the-board) for how to upload your development version of PX4 onto your Pixhawk board (should look something like `make px4_fmu-v4_default upload`) for FMUv4.

To find the `make` target for your specific board, list all options with the `make list_config_targets` command on your development host computer:

```bash
# on development host (not on Raspberry Pi)
cd ~/colcon_ws/src/gisnav/docker
gnc run --no-deps px4 make list_config_targets
```

Then choose your appropriate board for the following examples. We are going to choose `nxp_fmuk66-e_default` for this example:

```bash
# on development host (not on Raspberry Pi)
# "hil" command needed here to expose USB device (Pixhawk)
gnc hil run --no-deps -e DONT_RUN=1 px4 make nxp_fmuk66-e_default upload
```

## Deploy HIL simulation

```bash
# on development host (not on Raspberry Pi)
gnc hil up px4 gisnav@raspberrypi.local
```

::: tip Admin portal
You can also use the [Admin portal](/admin-portal) hosted on the Raspberry Pi 5 to see that the core services are running.

:::

## QGroundControl

After deploying the HIL simulation, adjust the settings via the QGC application as follows:

- Precisely match the `COM_RC_IN_MODE` parameter setting if mentioned in the instructions.
- Ensure that you have HITL enabled in QGC Safety settings.
- You may also need to enable the virtual joystick enabled in QGC General settings to prevent failsafes from triggering.
