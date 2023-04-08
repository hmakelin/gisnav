SHELL := /bin/bash

.PHONY: docs
docs:
	@$(MAKE) -C docs html
	@cd docs/_build/html && touch .nojekyll  # for GitHub Pages
