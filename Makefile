SHELL := /bin/bash

# Default output protocol for mock GPS messages (uorb or ublox). u-blox requires
# an exposed serial port while uORB uses ROS.
PROTOCOL=ublox

.PHONY: docs
docs:
	@cd docs/sphinx && sphinx-build -M markdown ./source ./build
	@mkdir -p docs/vitepress/docs/reference && cp -r docs/sphinx/build/markdown/* docs/vitepress/docs/reference
	@cd docs/vitepress && npm run docs:build
	@cd docs/vitepress/docs/.vitepress/dist && touch .nojekyll  # for GitHub Pages

.PHONY: docs\ preview
docs\ preview:
	@cd docs/vitepress && npm run docs:preview

.PHONY: docs\ dev
docs\ dev:
	@cd docs/vitepress && npm run docs:dev

.PHONY:
build:
	@echo "Building the project..."
	@$(MAKE) -C debian/gisnav $@

.PHONY:
dist: build
	@echo "Creating distribution package..."
	@$(MAKE) -C debian/gisnav $@

.PHONY: clean
clean: clean\ docs
	@echo "Cleaning up..."
	@$(MAKE) -C debian/gisnav $@
	# TODO - build and dist

.PHONY: clean\ docs
clean\ docs:
	@echo "Cleaning up documentation build files..."
	@rm -rf docs/sphinx/build
	@rm -rf docs/vitepress/docs/reference
	@rm -rf docs/vitepress/docs/.vitepress/dist

.PHONY: install
install: dist
	@echo "Installing the project and dependencies..."
	@$(MAKE) -C debian/gisnav $@

.PHONY: test
test:
	@echo "Running tests..."
	# TODO - run unit and launch tests (do not run simulation tests)

.PHONY: lint
lint:
	@echo "Running linter..."
	@pre-commit run --all-files

# alias for lint - we do not have a "dry-run" option for lint, both
# lint and format may modify the files
.PHONY: format
format: lint

.PHONY: check
check: lint test
	@echo "Running code quality checks..."

.PHONY: dev
dev:
	@echo "Launching GISNav locally using $(PROTOCOL) protocol..."
	@ros2 launch gisnav local.launch.py protocol:=$(PROTOCOL)

# This part has moved to the entrypoint script of the ubx middleware service:
#@echo "Setting up socat bridge to send $(PROTOCOL) serial output to simulator container over TCP port 15000..."
#@socat pty,link=/tmp/gisnav-pty-link,raw,echo=0 tcp:localhost:15000 || (echo "Could not establish serial-to-TCP bridge. Is the SITL simulation container running?"; exit 1)
#@sleep 3  # Give socat time to create the pty
#@echo PTS device created at: `readlink /tmp/gisnav-pty-link`
#@echo "Launching GISNav locally..."
#@ros2 launch gisnav local.launch.py protocol:=$(PROTOCOL) port:=`readlink /tmp/gisnav-pty-link` baudrate:=9600

.PHONY: help
help:
	@echo "Available targets:"
	@echo "  docs           - Build the documentation"
	@echo "  docs preview   - Preview the documentation"
	@echo "  docs dev       - Run the documentation development server"
	@echo "  build          - Build the project"
	@echo "  dev            - Run GISNav locally"
	@echo "  dist           - Create a distribution package"
	@echo "  clean          - Clean up all generated files"
	@echo "  clean docs     - Clean up documentation build files only"
	@echo "  install        - Install the project and dependencies"
	@echo "  test           - Run the project's tests"
	@echo "  lint           - Run the linter to check code style"
	@echo "  format         - Automatically format the code"
	@echo "  check          - Run linter and tests"
	@echo "  help           - Show this help message"
