# Install from Debian package

> [!WARNING] Warning: Simulation use only
> Do not use this software for real flight missions. GISNav is untested and has only been demonstrated in a simulation environment.

This page describes how you can a install containerized version of GISNav for HIL simulation on your companion computer from a Debian package.

- For a SITL mock GPS demo, see [here](/README) instead. Eventually the [GISNav CLI](/gisnav-cli) that comes with the Debian package should be able to run the demo as well, but currently that is not the case.

    ::: info Todo
    Run demo from CLI, e.g. `gnc sim start` or `gnc sim sitl start`

    :::

- For development you may want to install a [non-containerized version of GISNav](/install-locally) instead.




## Prerequisites

<!--@include: ./shared/docker-compose-required.md-->

## Install

<!--@include: ./shared/install-debian.md-->

After installing the package, you may want to check out the [CLI intro page](/gisnav-cli).

## Enable on startup

Enable the `gisnav-compose.service` that was installed with the Debian package to automatically start on system startup.

::: warning Warning: Resource constrained systems
Assuming you are installing this on a resource constrained companion computer, enabling the `gisnav-compose.service` will significantly slow down your system on startup.

:::

```bash
sudo systemctl enable gisnav-compose.service
sudo systemctl start gisnav-compose.service
```
