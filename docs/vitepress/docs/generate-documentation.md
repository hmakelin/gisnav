# Generate documentation

GISNav uses a two-stage process to generate its documentation. Sphinx is used to first build Markdown from the reStructuredText docstrings in the Python source code, and then VitePress is used to create the final static documentation from the Markdown files.

## Prerequisites

### Node.js

Install Node v18+ on your system by following the [official instructions](https://nodejs.org/en/download).

### Install GISNav

<!--@include: ./shared/run-in-container-prerequisites.md-->

## Make docs

::: code-group

```bash [Local]
cd ~/colcon_ws/src/gisnav
make docs
```

```bash [Docker]
cd ~/colcon_ws/src/gisnav
make docs
```

:::

## Build Sphinx documentation

```bash
cd ~/colcon_ws/src/gisnav/docs
make html
```

The HTML documentation will appear in the `~/colcon_ws/src/gisnav/docs/_build/` folder.

## Serve VitePress documentation

```bash
cd ~/colcon_ws/src/gisnav/docs/vitepress
npm run docs:dev
```
