# GISNav CLI

The [Debian package](/install-from-debian-package) installs the GISNav CLI (`gnc`), a command line interface used to deploy GISNav services.

`gnc` is a lightweight wrapper around Docker Compose. It simplifies the deployment process by automatically handling the selection of Compose files and overrides, as well as managing the building, creation, and deployment of services on any supported platform.

This page provides a quick introduction to using `gnc`.

## Prerequisites

- Install the [Debian package](/install-from-debian-package) on your `localhost`.

- Install the [Debian package](/install-from-debian-package) on any remote companion computers you wish to deploy on (e.g. `jetsonnano.local`, see examples below)
    ::: info Todo
    Use a container orchestration tool, possibly wrapped by the CLI, for multi-vehicle simulation
    :::

- Setup a local network with `ssh` server enabled and your public key authorized on any remote companion computers.

## Using the CLI

If you are experienced with `docker compose`, using `gnc` should be intuitive. A feature that is provided by `gnc` but not by `docker compose` is the ability to specify remote hosts individually for each service in a single command which streamlines multi-host deployment of services. Please see the examples below.

### Examples

Prepare services on `localhost`:

```bash
gnc create --build gisnav px4
```

Prepare `gisnav` on remote host `jetsonnano.local`:

```bash
gnc create --build gisnav@jetsonnano.local
```

Start both simulation and `gisnav` services on `localhost`:

```bash
gnc start px4 gisnav
```

Start simulation on `localhost`, and `gisnav` on `jetsonnano.local`:

::: info Companion computer hostname
The `GISNAV_COMPANION_HOST` environment variable is set below to tell the `px4` service where to find the middleware. `gnc` does not set these automatically and assumes the defaults from the `docker/.env` file.

:::

```bash
companion_host=jetsonnano.local
export GISNAV_COMPANION_HOST=$companion_host
gnc start px4 gisnav@$companion_host
```

Start simulation on `localhost` and `gisnav` on multiple remote hosts:

::: warning Multi-vehicle simulation not supported
The `px4` service does not (currently) support multi-vehicle simulation

:::

::: info Todo
Container orchestration tool for multi-vehicle simulation
:::

```bash
gnc start px4 gisnav@jetsonnano1.local gisnav@jetsonnano2.local
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

Stop all services on both `localhost` and remote host `jetsonnano.local`:

```bash
gnc stop @localhost @jetsonnano.local
```

The below is a more sophisticated alternative for stopping all services on both `localhost` and remote host `jetsonnano.local`:

```bash
gnc stop "" @jetsonnano.local
```

List running service containers:

```bash
gnc ps
```

View logs for the `gisnav` service running on `jetsonnano.local`:

```bash
gnc logs gisnav@jetsonnano.local
```

See `gnc` command line help:

```bash
gnc help
```

See `docker compose` command line help (`help` without the preceding `--` is not used by Compose and thereby taken by `gnc`):

```bash
gnc --help
```

Inspect the Compose configuration layered by `gnc` and parsed by `docker compose` for your system in canonical format :

```bash
gnc config
```

Inspect parsed environment variables <Badge type="info" text="yq required"/>:

::: info Todo
Support inspecting environment directly in `gnc`
:::

```bash
gnc config | yq '.services[] | .environment' | sort | uniq
```
