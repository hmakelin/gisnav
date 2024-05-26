# GISNav CLI

The [Debian package](/install-from-debian-package) comes with the GISNav CLI (`gnc`) command line interface that streamlines deployment of Compose services. `gnc` is a light wrapper around Docker Compose that eliminates the need for specifying which Compose files or overrides should be used and which services to build, create or deploy on any supported platform.

This page contains a quick intro to using `gnc`.

## Prerequisites

You must have installed the [Debian package](/install-from-debian-package).

You must also have installed `gnc` on any remote systems you wish to deploy on (e.g. `raspberrypi.local`, see examples below).

## Using the CLI

Prepare services on localhost:

```bash
gnc build gisnav px4 --with-dependencies
gnc create gisnav px4
```

Prepare `gisnav` on remote host `raspberrypi.local`:

```bash
gnc build gisnav@raspberrypi.local --with-dependencies
gnc create gisnav@raspberrypi.local
```


Start both simulation and `gisnav` services on localhost:

```bash
gnc start px4 gisnav
```

Start simulation on localhost and `gisnav` on `raspberrypi.local`:

```bash
gnc start px4 gisnav@raspberrypi.local
```

Stop all services on localhost:

```bash
gnc stop
```

List running service containers:

```bash
gnc ps
```

View logs for the `gisnav` service running on `raspberrypi.local`:

```bash
gnc logs gisnav@raspberrypi.local
```

See command line help:

```bash
gnc help
```
