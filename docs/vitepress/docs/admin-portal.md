# Admin portal

::: warning Warning: Experimental feature
The admin portal is very much untested and is intended for exploring how to provide ways of managing configuration parameters and onboard maps without coding or GIS expertise.

:::

GISNav includes a captive or self-hosted Homepage admin portal with links to relevant resources and a FileGator file server for editing configuration files without need for programming knowledge or commandline work.

## Launch home page

The following command should launch the home page and any required supporting services:

```
docker compose -p gisnav up homepage
```

You should then find the homepage at `localhost:3000`.
