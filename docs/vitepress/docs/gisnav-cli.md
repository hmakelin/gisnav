# GISNav CLI

The [Debian package](/install-from-debian-package) comes with the `gnv` command line interface (CLI) tool streamlines deployment of the Compose services. `gnv` is a light wrapper around Docker Compose that eliminates the need for specifying which Compose files or overrides should be used and which services to build, create or deploy.

This page contains a quick intro to using `gnv`.

## Prerequisites

You must have installed the [Debian package](/install-from-debian-package).

## Using the CLI

Start simulation and gisnav services on a companion computer:

```bash
gnv start px4 gisnav
```

Stop all services:

```bash
gnv stop
```

View services status:

```bash
gnv ps
```

See command line help:

```bash
gnv --help
```

Update existing services:

```bash
gnv build gisnav px4 --with-dependencies
gnv create gisnav px4
```
