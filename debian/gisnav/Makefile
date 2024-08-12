# Makefile for building the gisnav Debian package
# This file defines the steps to build the Debian package for the gisnav project.
# It cleans the build directory, prepares necessary files, and creates the Debian package.

SHELL := /bin/bash

# Package configuration variables
PACKAGE_NAME = gisnav
BUILD_DIR = build
DIST_DIR = dist
REPO_ROOT_PATH = $(shell realpath ../..)
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
	@rm -rf $(MAKEFILE_DIR)$(BUILD_DIR) $(MAKEFILE_DIR)$(DIST_DIR)

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

# We need a shallow copy of git repo to make git describe work for the docs
# (docs are built with git version string). pre-commit might also need the
# .git folder.
# We also grab any uncommitted changes from the local repo and apply them
# as a patch to the shallow copy, to make development more convenient. We apply
# staged changes first, and then unstaged changes (we create a combined patch
# file)
$(BUILD_DIR)/etc/gisnav:
	@mkdir -p $@
	@touch $(BUILD_DIR)/COLCON_IGNORE
	@git clone --depth 1 file://${REPO_ROOT_PATH} $@
	@cd ${REPO_ROOT_PATH} && git diff --cached > /tmp/gisnav-diff.patch \
	&& git diff >> /tmp/gisnav-diff.patch
	@if [ -s /tmp/gisnav-diff.patch ]; then \
		cd $@ && git apply /tmp/gisnav-diff.patch; \
	else \
		echo "No changes to apply."; \
	fi
	@rm -rf /tmp/gisnav-diff.patch
