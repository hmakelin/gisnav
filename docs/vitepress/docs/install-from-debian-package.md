# Install from Debian package

This page describes how to install GISNav from a Debian package. The Debian package includes the [GISNav CLI](/gisnav-cli) and the `gisnav.service` systemd service.

::: tip Install locally
GISNav CLI installs a containerized version of GISNav. For development you may want to install [GISNav locally](/install-locally) instead.
:::

::: info Todo
Separate CLI and service into dedicated Debian packages.
:::

## Prerequisites

<!--@include: ./shared/docker-compose-required.md-->

## Install

<!--@include: ./shared/install-debian.md-->

After installing the package, you may want to check out the [CLI intro page](/gisnav-cli).

## Enable on startup <Badge type="info" text="Optional"/>

Enable the `gisnav.service` that was installed with the Debian package to automatically start on system startup.

::: warning Warning: Resource constrained systems
Assuming you are installing this on a resource constrained companion computer, enabling the `gisnav.service` on startup will slow down your system.

:::

```bash
sudo systemctl enable gisnav.service
sudo systemctl start gisnav.service
```

## Quick start

After installing, try [simulating GPS failure](/sitl-local) in a local SITL simulation.
