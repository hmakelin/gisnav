# Generate documentation

GISNav uses a two-stage process to generate its documentation. Sphinx is used to first build Markdown from the reStructuredText docstrings in the Python source code, and then VitePress is used to create the final static documentation from the Markdown files.

## Prerequisites

### Node.js

Install Node v18+ on your system by following the [official instructions](https://nodejs.org/en/download).

### Install GISNav

<!--@include: ./shared/run-in-container-prerequisites.md-->

## Make docs

The documentation is built in a two-stage process where first the reST Python docstrings are converted into Markdown files using Sphinx, after which they are built into the final static documentation page using VitePress.

The Makefile `docs` target implements this recipe:

::: code-group

```bash [Local]
cd ~/colcon_ws/src/gisnav
make docs
```

```bash [Docker]
cd ~/colcon_ws/src/gisnav/docker
docker compose -p gisnav run gisnav make docs
```

:::

The static HTML documentation will appear in the below folder:

```text
~/colcon_ws/src/gisnav/docs/vitepress/docs/dist
```

## Serve VitePress documentation

```bash
cd ~/colcon_ws/src/gisnav/docs/vitepress
npm run docs:dev
```
