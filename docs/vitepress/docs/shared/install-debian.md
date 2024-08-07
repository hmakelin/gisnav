Download the `gisnav` Debian package and install it using the below commands. You can edit the release string to match your needs.

::: tip Private Docker registry
You can make this process quicker by building your own (potentially cross-platform) images on your development host and pulling them onto your Raspberry Pi 5 using a [private container registry](/deploy-with-docker-compose#private-registry).

:::

```bash
GISNAV_RELEASE=v0.68.0
wget https://github.com/hmakelin/gisnav/releases/download/${GISNAV_RELEASE}/gisnav-${GISNAV_RELEASE}_all.deb -O gisnav-${GISNAV_RELEASE}_all.deb
sudo dpkg -i ./gisnav-${GISNAV_RELEASE}_all.deb
```

::: info Create Debian
See the [Create Debian](/create-debian) page if you want to create your own Debian package instead of using the hosted version.

:::

After installing, you may want to prepare the key Docker containers before trying to deploy your services.

::: warning Warning: Long build time
Building the images will take a long time and use a lot of network bandwidth and storage space.
:::

```bash
gnc create --build px4 gisnav
```
