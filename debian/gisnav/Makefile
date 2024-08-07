# Makefile for building the gisnav Debian package
# This file defines the steps to build the Debian package for the gisnav project.
# It cleans the build directory, prepares necessary files, and creates the Debian package.

SHELL := /bin/bash

# Package configuration variables
PACKAGE_NAME = gisnav
BUILD_DIR = build
DIST_DIR = dist
REPO_ROOT_PATH = ../..
ARCHITECTURE = all

# Get the latest git tag and format the version string
VERSION_RAW = $(shell git describe --tags --abbrev=0)
VERSION = $(shell echo $(VERSION_RAW) | sed 's/^[^0-9]*//')

# Generate a unique build ID based on the current date and time
BUILD_ID = $(shell date +%Y%m%d%H%M%S)

# Define the output Debian package file name
#DEB_FILE = $(PACKAGE_NAME)_$(VERSION)-$(BUILD_ID)_$(ARCHITECTURE).deb
DEB_FILE = $(PACKAGE_NAME)_$(VERSION)_$(ARCHITECTURE).deb

# Default target to build the distribution package
all: dist

# Phony target for cleaning the build directory
.PHONY: clean
clean:
	@rm -rf $(BUILD_DIR) $(DIST_DIR)

.PHONY: distclean
distclean:
	@rm -rf $(DIST_DIR)

.PHONY: install
install: dist
	@sudo dpkg -i $(DIST_DIR)/$(DEB_FILE)

# Create distributable
dist: $(DIST_DIR)/$(DEB_FILE)

# Ensure the build directory and necessary subdirectories exist
$(BUILD_DIR): $(BUILD_DIR)/DEBIAN $(BUILD_DIR)/etc $(BUILD_DIR)/usr

# Target to build the Debian package
$(DIST_DIR)/$(DEB_FILE): $(BUILD_DIR)
	@mkdir -p dist
	dpkg-deb --build $< $@
	@echo "Debian package built successfully."

$(BUILD_DIR)/DEBIAN: $(BUILD_DIR)/DEBIAN/control $(BUILD_DIR)/DEBIAN/copyright $(BUILD_DIR)/DEBIAN/postinst $(BUILD_DIR)/DEBIAN/postrm

$(BUILD_DIR)/DEBIAN/control: DEBIAN/control.template
	@mkdir -p $(BUILD_DIR)/DEBIAN
	@VERSION=$(VERSION) envsubst < $< > $@

$(BUILD_DIR)/DEBIAN/copyright: DEBIAN/copyright.template
	@mkdir -p $(BUILD_DIR)/DEBIAN
	@VERSION=$(VERSION) envsubst < $< > $@

$(BUILD_DIR)/DEBIAN/postinst: DEBIAN/postinst
	@mkdir -p $(BUILD_DIR)/DEBIAN
	@cp $< $@

$(BUILD_DIR)/DEBIAN/postrm: DEBIAN/postrm
	@mkdir -p $(BUILD_DIR)/DEBIAN
	@cp $< $@

$(BUILD_DIR)/usr:
	@mkdir -p $@
	@cp -r usr/ $(BUILD_DIR)/

$(BUILD_DIR)/etc: $(BUILD_DIR)/etc/systemd $(BUILD_DIR)/etc/gisnav

$(BUILD_DIR)/etc/systemd:
	@mkdir -p $@
	@cp -r etc/systemd/ $@

# We need to copy .git to make git describe work for the docs (docs are built
# with git version string). pre-commit might also need the .git folder.
# TODO: only copy over a shallow copy of git repo  - we do not need entire history
# nor all branches
$(BUILD_DIR)/etc/gisnav: $(BUILD_DIR)/etc/gisnav/ros $(BUILD_DIR)/etc/gisnav/docker $(BUILD_DIR)/etc/gisnav/docs
	@dest=debian/gisnav/$@ && \
	mkdir -p $@ && \
	touch $(BUILD_DIR)/COLCON_IGNORE && \
	cd ${REPO_ROOT_PATH}; rsync -av --exclude-from='.gitignore' Makefile .pre-commit-config.yaml LICENSE.md README.md pyproject.toml .git $$dest

$(BUILD_DIR)/etc/gisnav/ros:
	@dest=debian/gisnav/$@ && \
	mkdir -p $@ && \
	touch $(BUILD_DIR)/COLCON_IGNORE && \
	cd ${REPO_ROOT_PATH}; rsync -av --exclude-from='.gitignore' ros/ $$dest
	@echo -e "\033[1;33;40mWarning! Check the file list above that no unintended files were copied over to the distributable.\033[0m"

$(BUILD_DIR)/etc/gisnav/docker:
	@dest=debian/gisnav/$@ && \
	mkdir -p $@ && \
	cd ${REPO_ROOT_PATH}; rsync -av --exclude-from='.gitignore' docker/ $$dest
	@echo -e "\033[1;33;40mWarning! Check the file list above that no unintended files were copied over to the distributable.\033[0m"

$(BUILD_DIR)/etc/gisnav/docs:
	@dest=debian/gisnav/$@ && \
	mkdir -p $@ && \
	touch $(BUILD_DIR)/COLCON_IGNORE && \
	cd ${REPO_ROOT_PATH}; rsync -av --exclude-from='.gitignore' docs/ $$dest
