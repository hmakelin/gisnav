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

## Uninstall

If you want to uninstall the service, use the below command:

```bash
sudo apt-get remove gisnav
```

## Quick start

After installing, you may want to check out the [CLI intro page](/gisnav-cli), or try [simulating GPS failure](/sitl-local) in a local SITL simulation.
