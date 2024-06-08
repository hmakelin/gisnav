# Simulate GPS failure

This page describes how to run the [SITL simulation](/sitl-local) with GISNav running on a companion computer (e.g. NVIDIA Jetson Nano).

## Prerequisites

### Simulation host

- Install the GISNav CLI via the [Debian package](/install-from-debian-package).

### Companion computer

- Install the GISNav CLI via the [Debian package](/install-from-debian-package).

### Connectivity & networking

- You need the `ssh` server enabled on your companion computer. Include your own `ssh` public key in the `~/.ssh/authorized_keys` file on the companion computer to ensure you can `ssh` in.

- These instructions assume you are using the hostname `jetsonnano` and that your network stack supports mDNS (`.local` domain). You can edit your companion computer hostname by editing the `/etc/hostname` file.

- Your simulation host and companion computer must be on the same local network. You can e.g. connect them directly with an Ethernet cable. You should share internet connection to your companion computer if you connect directly via Ethernet cable and not e.g. via a router.

## Build and start SITL simulation

Prepare the containers for the SITL simulation environment by running the following command on your simulation host.

```bash
GISNAV_COMPANION_HOST=jetsonnano.local gnc create --build px4 gisnav@jetsonnano.local
```

Start your simulation:

<!--@include: ./shared/slow-gazebo-startup-on-first-run.md-->

```bash
GISNAV_COMPANION_HOST=jetsonnano.local gnc start px4 gisnav@jetsonnano.local
```
