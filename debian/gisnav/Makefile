SHELL := /bin/bash

PACKAGE_NAME = gisnav
VERSION = $(shell git describe --tags $(git rev-list --tags --max-count=1))
BUILD_DIR = build
BUILD_ID = $(shell date +%Y%m%d%H%M%S)
ARCHITECTURE = all
DEB_DIR = $(BUILD_DIR)/$(PACKAGE_NAME)
REPO_ROOT_PATH = ../..
DEB_FILE = $(PACKAGE_NAME)_$(VERSION)-$(BUILD_ID)_$(ARCHITECTURE).deb

all: dist

clean:
	@rm -rf $(BUILD_DIR)

build: $(DEB_FILE)

# TODO: move build_package recipe under build target
build_package: copy_files
	@dpkg-deb --build $(DEB_DIR)
	@mv $(BUILD_DIR)/${PACKAGE_NAME}.deb ${DEB_DIR}/$(DEB_FILE)

$(DEB_FILE): $(DEB_DIR)/DEBIAN/control $(DEB_DIR)/DEBIAN/copyright
	dpkg-deb --build $(DEB_DIR) $(BUILD_DIR)

$(DEB_DIR)/DEBIAN/control: debian/control.template
	mkdir -p $(DEB_DIR)/DEBIAN
	VERSION=$(VERSION) envsubst < debian/control.template > $(DEB_DIR)/DEBIAN/control

$(DEB_DIR)/DEBIAN/copyright: debian/copyright.template
	mkdir -p $(DEB_DIR)/DEBIAN
	VERSION=$(VERSION) envsubst < debian/copyright.template > $(DEB_DIR)/DEBIAN/copyright

clean:
	rm -rf $(BUILD_DIR)

# TODO: makes a dummy docs/vitepress/package.json so that gisnav image will
# build - need to define a base image that does not need docs or dev dependencies
setup:
	@mkdir -p ${BUILD_DIR}
	@touch ${BUILD_DIR}/COLCON_IGNORE
	@mkdir -p $(DEB_DIR)/DEBIAN
	@mkdir -p $(DEB_DIR)/etc/gisnav/gisnav
	@mkdir -p $(DEB_DIR)/etc/gisnav/gisnav_msgs
	@mkdir -p $(DEB_DIR)/etc/gisnav/docker
	@mkdir -p $(DEB_DIR)/etc/gisnav/docs/vitepress
	@echo "{}" > $(DEB_DIR)/etc/gisnav/docs/vitepress/package.json
	@mkdir -p $(DEB_DIR)/etc/systemd/system

# Copy source files to build folder
# We copy the gisnav ROS package source too so that we can build the gisnav
# image locally and not be dependent on any external container registries
# TODO: do not copy development and testing files over to the .deb package
# TODO: make a "production" gisnav image that does not need docs folder to build,
# rename current image to gisnav-deb and use the "production" or core image as
# base
# Need to be in repo root path to make .gitignore work
copy_files: setup
	@cd ${REPO_ROOT_PATH}; rsync -av --exclude-from='.gitignore' gisnav/ systemd/$(PACKAGE_NAME)/$(DEB_DIR)/etc/$(PACKAGE_NAME)/gisnav/
	@cd ${REPO_ROOT_PATH}; rsync -av --exclude-from='.gitignore' gisnav_msgs/ systemd/$(PACKAGE_NAME)/$(DEB_DIR)/etc/$(PACKAGE_NAME)/gisnav_msgs/
	@cd ${REPO_ROOT_PATH}; rsync -av --exclude-from='.gitignore' docker/ systemd/$(PACKAGE_NAME)/$(DEB_DIR)/etc/$(PACKAGE_NAME)/docker/
	@cp etc/systemd/system/gisnav.service $(DEB_DIR)/etc/systemd/system
	@cp -r usr/ $(DEB_DIR)/usr
	@cp DEBIAN/control $(DEB_DIR)/DEBIAN/
	@cp DEBIAN/postinst $(DEB_DIR)/DEBIAN/
	@cp DEBIAN/postrm $(DEB_DIR)/DEBIAN/
	@echo -e "\033[1;33mWarning! Check the file lists above that no unintended files were copied over to the distributable.\033[0m"

dist: clean build  # build_package
	@echo "Debian package built successfully."

.PHONY: all clean setup copy_files build_package dist
