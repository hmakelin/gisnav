SHELL := /bin/bash

# Get the directory of the Makefile
MAKEFILE_DIR := $(dir $(realpath $(lastword $(MAKEFILE_LIST))))

# test
.PHONY: test-sitl
test-sitl:
	@python $(MAKEFILE_DIR)test/sitl/test_sitl.py px4 \
		|| (echo "SITL test failed, exit code: $$?"; exit 1)

.PHONY: test-launch
test-launch:
	@launch_test $(MAKEFILE_DIR)test/launch/test_default_launch.py

.PHONY: test-unit
test-unit:
	@python3 -m unittest discover -s $(MAKEFILE_DIR)test/unit -p "test_*.py"

# TODO: move test-static to same directory as .pre-commit-config.yaml
.PHONY: test-static
test-static:
	@pre-commit run --all-files --config $(MAKEFILE_DIR)/../../.pre-commit-config.yaml
# test end
