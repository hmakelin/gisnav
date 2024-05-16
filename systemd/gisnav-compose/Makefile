SHELL := /bin/bash

PACKAGE_NAME = gisnav-compose
VERSION = 0.67.0
BUILD_DIR = build
BUILD_ID = $(shell date +%Y%m%d%H%M%S)
ARCHITECTURE = all
DEB_DIR = $(BUILD_DIR)/$(PACKAGE_NAME)
REPO_ROOT_PATH = ../..
COMPOSE_FILE_PATH = ${REPO_ROOT_PATH}/docker
GISNAV_ROS_PACKAGE_PATH = ${REPO_ROOT_PATH}/gisnav
DEB_FILE = $(PACKAGE_NAME)_$(VERSION)-$(BUILD_ID)_$(ARCHITECTURE).deb

all: dist

clean:
	@rm -rf $(BUILD_DIR)

setup:
	@mkdir -p $(DEB_DIR)/DEBIAN
	@mkdir -p $(DEB_DIR)/etc/gisnav-compose/gisnav
	@mkdir -p $(DEB_DIR)/etc/gisnav-compose/docker
	@mkdir -p $(DEB_DIR)/etc/systemd/system

# Copy source files to build folder
# We copy the gisnav ROS package source too so that we can build the gisnav
# image locally and not be dependent on any external container registries
# TODO: do not copy development and testing files over to the .deb package
# TODO: make the rsync part less brittle - might unintendedly copy over files
# that are not intended for distribution as the .gitignore relative paths are
# not respected (are not the same)
copy_files: setup
	@cp ${REPO_ROOT_PATH}/.gitignore .rsyncignore
	@echo '${REPO_ROOT_PATH}/.git/' >> .rsyncignore
	@rsync -av --exclude-from='.rsyncignore' ${GISNAV_ROS_PACKAGE_PATH}/ $(DEB_DIR)/etc/gisnav-compose/gisnav/
	@rsync -av --exclude-from='.rsyncignore' ${COMPOSE_FILE_PATH}/ $(DEB_DIR)/etc/gisnav-compose/docker/
	@rm .rsyncignore
	@cp etc/systemd/system/gisnav-compose.service $(DEB_DIR)/etc/systemd/system
	@cp DEBIAN/control $(DEB_DIR)/DEBIAN/
	@cp DEBIAN/postinst $(DEB_DIR)/DEBIAN/
	@cp DEBIAN/postrm $(DEB_DIR)/DEBIAN/
	@echo -e "\033[1;33mWarning! Check the file lists above that no unintended files were copied over to the distributable.\033[0m"

build_package: copy_files
	@dpkg-deb --build $(DEB_DIR)
	@mv $(BUILD_DIR)/${PACKAGE_NAME}.deb ${DEB_DIR}/$(DEB_FILE)

dist: clean build_package
	@echo "Debian package built successfully."

.PHONY: all clean setup copy_files build_package dist
