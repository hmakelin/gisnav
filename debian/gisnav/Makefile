SHELL := /bin/bash

PACKAGE_NAME = gisnav
BUILD_DIR = build

REPO_ROOT_PATH = ../..

ARCHITECTURE = all
VERSION = $(shell git describe --tags $(git rev-list --tags --max-count=1))
BUILD_ID = $(shell date +%Y%m%d%H%M%S)
DEB_FILE = $(PACKAGE_NAME)_$(VERSION)-$(BUILD_ID)_$(ARCHITECTURE).deb


all: dist

.PHONY: clean
clean:
	@rm -rf $(BUILD_DIR)

dist: clean $(BUILD_DIR)

$(BUILD_DIR): $(BUILD_DIR)/$(DEB_FILE)
	@mv $(BUILD_DIR)/${PACKAGE_NAME}.deb ${BUILD_DIR}/$(DEB_FILE)
	@echo "Debian package built successfully."

$(BUILD_DIR)/$(DEB_FILE): $(BUILD_DIR)/DEBIAN $(BUILD_DIR)/etc $(BUILD_DIR)/usr
	dpkg-deb --build $(BUILD_DIR)

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
	@mkdir -p $(BUILD_DIR)/{usr}
	@cp -r usr/ $(BUILD_DIR)/

$(BUILD_DIR)/etc: $(BUILD_DIR)/etc/systemd $(BUILD_DIR)/etc/gisnav

$(BUILD_DIR)/etc/systemd:
	@mkdir -p $(BUILD_DIR)/etc/systemd
	@cp -r etc/systemd/ $(BUILD_DIR)/etc/systemd

$(BUILD_DIR)/etc/gisnav: $(BUILD_DIR)/etc/gisnav/ros $(BUILD_DIR)/etc/gisnav/docker $(BUILD_DIR)/etc/gisnav/docs

$(BUILD_DIR)/etc/gisnav/ros:
	@dest=debian/gisnav/$@ && \
	mkdir -p $@ && \
	cd ${REPO_ROOT_PATH}; rsync -av --exclude-from='.gitignore' ros/ $$dest
	@echo -e "\033[1;33mWarning! Check the file list above that no unintended files were copied over to the distributable.\033[0m"

$(BUILD_DIR)/etc/gisnav/docker:
	@dest=debian/gisnav/$@ && \
	mkdir -p $@ && \
	cd ${REPO_ROOT_PATH}; rsync -av --exclude-from='.gitignore' docker/ $$dest
	@echo -e "\033[1;33mWarning! Check the file list above that no unintended files were copied over to the distributable.\033[0m"

# TODO package.json needed for gisnav image build to go through - try to
# refactor it out (need better defined gisnav image)
$(BUILD_DIR)/etc/gisnav/docs:
	@mkdir -p $@/vitepress && \
	@echo "{}" > $@/vitepress/package.json
