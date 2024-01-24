SHELL := /bin/bash

include gisnav/Makefile
include docker/Makefile

# The docs/Makefile has a catch-all target so it is not included here
#include docs/Makefile

.PHONY: docs
docs:
	@$(MAKE) -C docs html
	@cd docs/_build/html && touch .nojekyll  # for GitHub Pages
