SHELL := /bin/bash

include gisnav/Makefile
include docker/Makefile
include systemd/Makefile

# The docs/Makefile has a catch-all target so it is not included here
#include docs/Makefile

.PHONY: docs
docs:
	@cd docs/sphinx && sphinx-build -M markdown ./source ./build
	@mkdir -p docs/vitepress/docs/reference && cp -r docs/sphinx/build/markdown/* docs/vitepress/docs/reference
	@cd docs/vitepress && npm run docs:build
	@cd docs/vitepress/docs/.vitepress/dist && touch .nojekyll  # for GitHub Pages

.PHONY: docs-preview
docs-preview:
	@cd docs/vitepress && npm run docs:preview

.PHONY: docs-dev
docs-preview:
	@cd docs/vitepress && npm run docs:dev
