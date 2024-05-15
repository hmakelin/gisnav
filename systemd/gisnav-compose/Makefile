SHELL := /bin/bash

PACKAGE_NAME = gisnav-compose
VERSION = 0.67.0
BUILD_DIR = build
BUILD_ID = $(shell date +%Y%m%d%H%M%S)
ARCHITECTURE = all
DEB_DIR = $(BUILD_DIR)/$(PACKAGE_NAME)
COMPOSE_FILE_PATH = ../../docker
DEB_FILE = $(PACKAGE_NAME)_$(VERSION)-$(BUILD_ID)_$(ARCHITECTURE).deb

all: dist

clean:
	@rm -rf $(BUILD_DIR)

setup:
	@mkdir -p $(DEB_DIR)/DEBIAN
	@mkdir -p $(DEB_DIR)/etc/gisnav-compose
	@mkdir -p $(DEB_DIR)/etc/systemd/system

# Copy source files to build folder
copy_files: setup
	@cp ${COMPOSE_FILE_PATH}/{docker-compose.yaml,.env} $(DEB_DIR)/etc/gisnav-compose/
	@cp etc/systemd/system/gisnav-compose.service $(DEB_DIR)/etc/systemd/system
	@cp DEBIAN/control $(DEB_DIR)/DEBIAN/
	@cp DEBIAN/postinst $(DEB_DIR)/DEBIAN/
	@cp DEBIAN/postrm $(DEB_DIR)/DEBIAN/

build_package: copy_files
	@dpkg-deb --build $(DEB_DIR)
	@mv $(BUILD_DIR)/${PACKAGE_NAME}.deb ${DEB_DIR}/$(DEB_FILE)

dist: clean build_package
	@echo "Debian package built successfully."

.PHONY: all clean setup copy_files build_package dist
