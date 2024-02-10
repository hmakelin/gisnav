SHELL := /bin/bash

include gisnav/Makefile
include docker/Makefile

# The docs/Makefile has a catch-all target so it is not included here
#include docs/Makefile

.PHONY: docs
docs:
	mkdir -p docs/_build && \
	    d2 --font-regular="docs/_static/fonts/Poppins Regular.ttf" \
	    --theme=0 --dark-theme=200 \
	    --font-bold="docs/_static/fonts/Poppins Medium.ttf" \
	    --font-italic="docs/_static/fonts/Poppins Regular.ttf" \
	    docs/pages/developer_guide/offboard/_external_interfaces.d2 \
	    docs/_build/external_interfaces.html
	@$(MAKE) -C docs html
	@cd docs/_build/html && touch .nojekyll  # for GitHub Pages
