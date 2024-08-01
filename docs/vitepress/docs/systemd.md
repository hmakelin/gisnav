# GISNav systemd service

This page describes how to use the `gisnav.service` systemd service that is included with the [Debian package](/install-from-debian-package).

The service is intended to be installed on a dedicated companion computer where GISNav is required to start automatically when the board is powered on. The service does not build images, it only creates a new container from an existing image if no container is available.

## Install

Install the service by [installing the Debian package](/install-from-debian-package).

## Enable on startup <Badge type="info" text="Optional"/>

The service is disabled by default. Enable it with the following command:

::: warning Warning: Resource constrained systems
Assuming you are installing this on a resource constrained companion computer, enabling the `gisnav.service` on startup will slow down your system.

:::

```bash
sudo systemctl enable gisnav.service
```

## Disable

The following command disables the service on startup:

```bash
sudo systemctl disable gisnav.service
```

## Start

Once enabled, the `gisnav.service` should start automatically when the system is started. Restart the system or start the `gisnav` service manually with the following command:

```bash
sudo systemctl start gisnav.service
```

## Stop

```bash
sudo systemctl stop gisnav.service
```

## Check status

The below example shows how you can check that the `gisnav` service is active:

```console
hmakelin@hmakelin-MS-7D48:~$ sudo systemctl status gisnav.service
● gisnav.service - GISNav Docker Compose Services
     Loaded: loaded (/etc/systemd/system/gisnav.service; enabled; vendor preset: enabled)
     Active: active (exited) since Wed 2024-05-15 15:10:21 BST; 3min 35s ago
   Main PID: 241948 (code=exited, status=0/SUCCESS)
        CPU: 354ms

May 15 15:10:18 hmakelin-MS-7D48 docker[241971]:  Container gisnav-mavros-1  Started
May 15 15:10:18 hmakelin-MS-7D48 docker[241971]:  Container gisnav-gscam-1  Started
May 15 15:10:18 hmakelin-MS-7D48 docker[241971]:  Container gisnav-micro-ros-agent-1  Started
May 15 15:10:18 hmakelin-MS-7D48 docker[241971]:  Container gisnav-px4-1  Starting
May 15 15:10:18 hmakelin-MS-7D48 docker[241971]:  Container gisnav-mapserver-1  Started
May 15 15:10:19 hmakelin-MS-7D48 docker[241971]:  Container gisnav-postgres-1  Started
May 15 15:10:20 hmakelin-MS-7D48 docker[241971]:  Container gisnav-px4-1  Started
May 15 15:10:20 hmakelin-MS-7D48 docker[241971]:  Container gisnav-gisnav-1  Starting
May 15 15:10:21 hmakelin-MS-7D48 docker[241971]:  Container gisnav-gisnav-1  Started
May 15 15:10:21 hmakelin-MS-7D48 systemd[1]: Finished GISNav Docker Compose Services.

```
