# Install

> [!WARNING] Warning: Simulation use only
> Do not use this software for real flight missions. GISNav is untested and has only been demonstrated in a simulation environment.

This page describes how you can a install containerized version of GISNav for HIL simulation on your companion computer from a Debian package.

For the mock GPS demo, see [here](/README) instead. Eventually the CLI here should be able to run the demo as well.

::: info Todo
Run demo from CLI, e.g. `gnv sim start` or `gnv sim sitl start`

:::

For development you may want to install a [non-containerized version](/install-locally) instead.

## Prerequisites

<!--@include: ./shared/docker-compose-required.md-->

## Debian package

<!--@include: ./shared/install-debian.md-->

## Using the CLI

You can also try using the `gnv` command line client that comes with the Debian package to start and stop the services:

```bash
gnv start
```

```bash
gnv stop
```

Help:

```bash
gnv --help
```
