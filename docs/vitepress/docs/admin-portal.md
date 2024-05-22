# Admin portal

::: warning Warning: Experimental feature
The admin portal is very much untested and is intended for exploring how to provide ways of managing configuration parameters and onboard maps without coding or GIS expertise.

:::

GISNav includes a captive or self-hosted Homepage admin portal with links to relevant resources. A FileGator file server for editing configuration files without need for programming knowledge or commandline work. A Glances instance is available to monitor the system.


## Serve admin portal

You can use the [CLI tool](/gisnav-cli) or [Docker Compose](/deploy-with-docker-compose) to serve the admin portal.

### GISNav CLI

The admin portal is automatically served when using the CLI:

```bash
gnv start
```

### Docker Compose

To launch the admin portal along with any required supporting services, use the following command:

```bash
cd ~/colcon_ws/src/gisnav/docker
docker compose -p gisnav up nginx
```

Once the services are running, you can open the admin portal in a web browser using the hostname of the server hosting the homepage on port `80`. Below are examples for accessing the homepage hosted on the `localhost` as well as on an external [Raspberry Pi](/raspberry-pi-pixhawk) with the default hostname `raspberrypi.local`:


## Visit admin portal

You can find the admin portal at port `80` on the host machine (e.g. `localhost`, or a [Raspberry Pi](/raspberry-pi-pixhawk)). Use the below commands to open the web page with your default web browser:

::: code-group

```bash [localhost]
xdg-open http://localhost
```

```bash [raspberrypi.local]
xdg-open http://raspberrypi.local
```

:::
