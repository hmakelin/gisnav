# Admin portal

::: warning Warning: Experimental feature
The admin portal is very much untested and is intended for exploring how to provide ways of managing configuration parameters and onboard maps without coding or GIS expertise.

:::

GISNav includes a captive or self-hosted Homepage admin portal with links to relevant resources and a FileGator file server for editing configuration files without need for programming knowledge or commandline work.


## Launch Home Page

To launch the home page along with any required supporting services, use the following command:

```bash
docker compose -p gisnav up nginx
```

Once the services are running, you can open the admin portal in a web browser using the hostname of the server hosting the homepage on port `80`. Below are examples for accessing the homepage hosted on the `localhost` as well as on an external [Raspberry Pi](/raspberry-pi-pixhawk) with the default hostname `raspberrypi.local`:

::: code-group

```bash [localhost]
firefox http://localhost:80
```

```bash [raspberrypi.local]
firefox http://raspberrypi.local:80
```

:::
