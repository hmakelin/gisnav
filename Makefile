SHELL := /bin/bash

include gisnav/Makefile
include docker/Makefile

# The docs/Makefile has a catch-all target so it is not included here
#include docs/Makefile

.PHONY: docs
docs:
	@cd docs/vitepress && npm run docs:build
	@cd docs/.vitepress/dist && touch .nojekyll  # for GitHub Pages

.PHONY: docs-preview
docs-preview:
	@cd docs/vitepress && npm run docs:preview
