# GISNav CLI

The [Debian package](/install-from-debian-package) comes with the `gnv` command line interface (CLI) tool that simplifies setup and deployment. `gnv` is a light wrapper around Docker Compose that eliminates the need for specifying which Compose files or overrides should be used and which services to build, create or deploy.

This page contains a quick intro to using `gnv`.

::: tip Companion computer recommended
Currently `gnv` only supports deploying services intended to run [on the companion computer in HIL simulation](/raspberry-pi-pixhawk). It does not deploy simulations such as the ones that are needed to run the [mock GPS demo](/README). Support to deploy simulations will possibly be added in the future.

:::

## Prerequisites

You must have installed the [Debian package](/install-from-debian-package).

## Using the CLI

Start services on a companion computer:

```bash
gnv start
```

Stop services:

```bash
gnv stop
```

View Docker Compose services status:

```bash
gnv status
```

See command line help:

```bash
gnv --help
```

Setup or update existing Docker Compose services:

```bash
gnv setup
```
