import os
import sys

# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html
sys.path.insert(0, os.path.abspath("../gisnav"))

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = "GISNav"
copyright = "2024, Harri Makelin"
author = "Harri Makelin"
release = "v0.66.0"

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.coverage",
    "sphinx.ext.todo",
    "sphinx.ext.viewcode",
    "sphinx_design",
    "autodocsumm",
    "sphinxcontrib.mermaid",
    "sphinx_copybutton",
]
templates_path = ["_templates"]

autoclass_content = "both"  # document __init__

# sphinx-markdown-builder
markdown_anchor_signatures = True

# automodule default options
autodoc_default_options = {
    "autosummary": True,
    "members": True,
    "undoc-members": True,
    "show-inheritance": True,
}

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = "alabaster"
html_static_path = ["_static"]
