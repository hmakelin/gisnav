# GISNav CLI

The [Debian package](/install-from-debian-package) comes with the `gnc` command line interface (CLI) tool streamlines deployment of the Compose services. `gnc` is a light wrapper around Docker Compose that eliminates the need for specifying which Compose files or overrides should be used and which services to build, create or deploy.

This page contains a quick intro to using `gnc`.

## Prerequisites

You must have installed the [Debian package](/install-from-debian-package).

## Using the CLI

Start simulation and gisnav services on a companion computer:

```bash
gnc start px4 gisnav
```

Stop all services:

```bash
gnc stop
```

View services status:

```bash
gnc ps
```

See command line help:

```bash
gnc --help
```

Update existing services:

```bash
gnc build gisnav px4 --with-dependencies
gnc create gisnav px4
```
