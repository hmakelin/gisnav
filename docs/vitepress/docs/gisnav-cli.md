# GISNav CLI

The [Debian package](/install-from-debian-package) comes with the GISNav CLI (`gnc`) command line interface that streamlines deployment of Compose services. `gnc` is a light wrapper around Docker Compose that eliminates the need for specifying which Compose files or overrides should be used and which services to build, create or deploy on any supported platform.

This page contains a quick intro to using `gnc`.

## Prerequisites

You must have installed the [Debian package](/install-from-debian-package).

You must also have installed `gnc` on any remote systems you wish to deploy on (e.g. `raspberrypi.local`, see examples below), and setup a local network with `ssh` server enabled and your public key authorized on the Raspberry Pi 5.

## Using the CLI

If you are experienced with `docker compose`, using `gnc` should be intuitive. A feature that is provided by `gnc` but not by `docker compose` is the ability to specify remote hosts individually for each service in a single command which streamlines multi-host deployment of services. Please see the examples below.

### Examples

Prepare services on `localhost`:

```bash
gnc build gisnav px4 --with-dependencies
gnc create gisnav px4
```

Prepare `gisnav` on remote host `raspberrypi.local`:

```bash
gnc build gisnav@raspberrypi.local --with-dependencies
gnc create gisnav@raspberrypi.local
```

Start both simulation and `gisnav` services on `localhost`:

```bash
gnc start px4 gisnav
```

Start simulation on `localhost` and `gisnav` on `raspberrypi.local`:

::: info Companion computer hostname
The `GISNAV_COMPANION_HOST` and `PX4_VIDEO_HOST_IP` environment variables are set below to tell the `px4` service where to find the middleware. `gnc` does not set these automatically and assumes the defaults from the `.env` file.

:::

```bash
companion_host=raspberrypi.local
export GISNAV_COMPANION_HOST=$companion_host
export PX4_VIDEO_HOST_IP=$companion_host
gnc start px4 gisnav@raspberrypi.local
```

Start simulation on `localhost` and `gisnav` on multiple remote hosts:

::: warning Multi-vehicle simulation not supported
The `px4` service does not (currently) support multi-vehicle simulation

:::

::: info Todo
- Container orchestration tool for multi-vehicle simulation
:::

```bash
gnc start px4 gisnav@raspberrypi1.local gisnav@raspberrypi2.local
```

Attach to the container to see the logs output:

```bash
gnc up nginx
```

Run in background:

```bash
gnc up nginx -d
```

Stop all services on `localhost`:

```bash
gnc stop
```

Stop all services on both `localhost` and remote host `raspberrypi.local`:

```bash
gnc stop @localhost @raspberrypi.local
```

The below is a more sophisticated alternative for stopping all services on both `localhost` and remote host `raspberrypi.local`:

```bash
gnc stop "" @raspberrypi.local
```

List running service containers:

```bash
gnc ps
```

View logs for the `gisnav` service running on `raspberrypi.local`:

```bash
gnc logs gisnav@raspberrypi.local
```

See `gnc` command line help:

```bash
gnc help
```

See `docker compose` command line help:

```bash
gnc --help
```

Inspect the canonical format Compose configuration layered by `gnc` and parsed by `docker compose` for your system:

```bash
gnc config
```
