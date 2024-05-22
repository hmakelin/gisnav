SHELL := /bin/bash

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
	@dpkg -i debian/gisnav/build/gisnav_*_all.deb
	@echo "Installation complete. Try 'gnv help' to see available commands."

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

.PHONY: help
help:
	@echo "Available targets:"
	@echo "  docs           - Build the documentation"
	@echo "  docs preview   - Preview the documentation"
	@echo "  docs dev       - Run the documentation development server"
	@echo "  build          - Build the project"
	@echo "  dist           - Create a distribution package"
	@echo "  clean          - Clean up all generated files"
	@echo "  clean docs     - Clean up documentation build files only"
	@echo "  install        - Install the project and dependencies"
	@echo "  test           - Run the project's tests"
	@echo "  lint           - Run the linter to check code style"
	@echo "  format         - Automatically format the code"
	@echo "  check          - Run linter and tests"
	@echo "  help           - Show this help message"
