Download the `gisnav-compose` Debian package and install it using the below commands. You can edit the release string to match your needs.

::: warning Warning: Long build time
The `postinst` script of the Debian package will attempt to pull and build all required Docker images and then create the containers. If the images are not available to be pulled, be prepared for a very long build time.

:::

::: tip Private Docker registry
You can make this process quicker by building your own (potentially cross-platform) images on your development host and pulling them onto your Raspberry Pi 5 using a [private container registry](/deploy-with-docker-compose#private-registry).

:::

```bash
GISNAV_RELEASE=v0.67.0
wget https://github.com/hmakelin/gisnav/releases/download/${GISNAV_RELEASE}/gisnav-compose_${GISNAV_RELEASE}_all.deb -O gisnav-compose_${GISNAV_RELEASE}_all.deb
sudo dpkg -i gisnav-compose_${GISNAV_RELEASE}_all.deb
```

After installing you can fix any missing dependencies using `apt-get`:

```bash
sudo apt-get install -f
```
