# Admin portal

GISNav includes a captive (self-hosted) admin portal that can be accessed via a web browser. Some of the functionality available via the portal includes but is not limited to:

- Editing configuration files and managing maps via FileGator fileserver
- Monitoring system status via Glances
- Viewing uploaded maps via OpenLayers
- Monitoring health of Docker Compose services
- Downloading more maps using [links to external imagery](/setup-gis-server#orthoimagery-and-dems)

## Serve admin portal

The admin portal is automatically served when starting `gisnav` using [GISNav CLI](/gisnav-cli):

```bash
gnc create --build gisnav
gnc start gisnav
```

You can also only start the admin tools by starting `nginx` which `gisnav` depends on:

```bash
gnc create --build nginx
gnc start nginx
```

## Access admin portal

You can access the admin portal at port `80` on the host machine such as `localhost`, or e.g. `jetsonnano.local` if using [a separate companion computer on a local network](/hil-pixhawk). Use the below example commands to open the admin portal with your default web browser.

::: info SSL/TLS not yet supported
SSL/TLS i.e. HTTPS over port `443` is not yet supported.
:::

::: tip Edit hostname
Edit the hostname to match your companion computer hostname if not running on `localhost`.

These examples assume your network stack supports mDNS i.e. can resolve the `.local` top-level domain. This is likely if you are running Ubuntu and connecting to your companion computer directly via Ethernet cable.
:::

::: code-group

```bash [localhost]
xdg-open http://localhost
```

```bash [jetsonnano.local]
xdg-open http://jetsonnano.local
```

```bash [raspberrypi.local]
# not verified to work - 4GB memory may be too little on RPi
xdg-open http://raspberrypi.local
```
:::
