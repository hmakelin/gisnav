PACKAGE_NAME = gisnav-compose
VERSION = 1.0.0
BUILD_DIR = build
DEB_DIR = $(BUILD_DIR)/$(PACKAGE_NAME)

all: dist

clean:
    rm -rf $(BUILD_DIR)

setup:
    mkdir -p $(DEB_DIR)/DEBIAN
    mkdir -p $(DEB_DIR)/etc/gisnav-compose
    mkdir -p $(DEB_DIR)/etc/systemd/system

copy_files: setup
	cp ../docker/docker-compose.yaml $(DEB_DIR)/etc/gisnav-compose
    cp systemd/gisnav-compose/etc/systemd/system/gisnav-compose.service $(DEB_DIR)/etc/systemd/system/
    cp systemd/gisnav-compose/DEBIAN/control $(DEB_DIR)/DEBIAN/
    cp systemd/gisnav-compose/DEBIAN/postrm $(DEB_DIR)/DEBIAN/

build_package: copy_files
    dpkg-deb --build $(DEB_DIR)

dist: clean build_package
    @echo "Debian package built successfully."

.PHONY: all clean setup copy_files build_package dist
