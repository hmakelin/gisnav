# Deploy with Docker Compose

GISNav utilizes several [Docker Compose](/glossary#docker-compose) services to establish its various deployment configurations. These services are orchestrated through a Makefile to improve convenience and facilitate adoption.

This page provides details on how to build and deploy these services, allowing for customization of GISNav's deployments beyond the capabilities provided by the Makefile.

## Prerequisites

### Docker Compose

<!--@include: ./shared/docker-compose-required.md-->

### NVIDIA Container Toolkit <Badge type="info" text="Optional"/>

<!--@include: ./shared/nvidia-container-toolkit-required.md-->

### GISNav source code

<!--@include: ./shared/create-colcon-workspace.md-->

<!--@include: ./shared/clone-to-colcon-workspace.md-->

### Compose project name <Badge type="info" text="Optional"/>

<!--@include: ./shared/compose-project-name.md-->

This step is optional since the `docker compose` example commands on this page will use the `-p gisnav` option to ensure the project name is always specified.

### Compose file stack

<!--@include: ./shared/export-compose-files.md-->


## Example deployments

The interdependencies between different services are hard-coded into the Docker Compose file using the `depends_on` key and typically you will need to start only a few services explicitly to get everything up and running.

### Mock GPS demo

To deploy the [mock GPS demonstration](/README) introduced locally without using the Makefile, follow the below steps to create, start, and shutdown the required containers. These steps simply repeat what the `demo` Makefile recipe would do.

#### Build images

```bash
cd ~/colcon_ws/src/gisnav/docker
docker compose $GISNAV_COMPOSE_FILES -p gisnav build gisnav px4
```

#### Create containers

```bash
cd ~/colcon_ws/src/gisnav/docker
docker compose $GISNAV_COMPOSE_FILES -p gisnav create gisnav px4
```

#### Expose X server to containers

<!--@include: ./shared/expose-x-server.md-->

#### Start containers

```bash
cd ~/colcon_ws/src/gisnav/docker
docker compose $GISNAV_COMPOSE_FILES -p gisnav start gisnav px4
```

#### Stop containers

```bash
cd ~/colcon_ws/src/gisnav/docker
docker compose $GISNAV_COMPOSE_FILES -p gisnav stop gisnav px4
```


### Local development

When deploying for local development, the difference to deploying [mock GPS demo](#mock-gps-demo) is that we do not include the `gisnav` service which is assumed to be launched directly on the Docker host.

::: tip Remember to expose X server
Remember to [expose your X server to your containers](#expose-x-server-to-containers) to ensure any GUIs are displayed properly.
:::

::: code-group

```bash [Build images and create containers]
cd ~/colcon_ws/src/gisnav/docker
docker compose $GISNAV_COMPOSE_FILES -p gisnav create --build \
    px4 \
    rviz \
    qgis

```

```bash [Start containers]
cd ~/colcon_ws/src/gisnav/docker
docker compose $GISNAV_COMPOSE_FILES -p gisnav start \
    px4 \
    rviz \
    qgis

```

```bash [Stop containers]
cd ~/colcon_ws/src/gisnav/docker
docker compose -p gisnav stop \
    px4 \
    rviz \
    qgis
```

:::

After you have your supporting services deployed, you might be interested in [launching a local GISNav app](/deploy-for-development#deploy-via-ros-launch-system).


## Private registry

::: warning Warning: Untested, not implemented
This is a suggested model for deploying Docker images which may be non-distributable, whether for licensing or other reasons, to a companion computer on a local network.

:::

::: info Todo
- Missing example commands, including building cross-platform images
- Should be moved to HIL section as this mainly concerns deployment on companion computers?

:::

You can push the built Docker images to a private (or even [air-gapped](https://distribution.github.io/distribution/#considerations-for-air-gapped-registries)) Docker registry to simplify deployment to a companion computer.

The `docker/docker-compose.yaml` file uses the environment variables in the `docker/.env` file to determine the registry host, user namespace and port. These can be all changed to point to your private registry.
