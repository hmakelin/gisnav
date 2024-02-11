from __future__ import annotations

import datetime
import os
import subprocess
import sys
from dataclasses import dataclass
from typing import Optional, Union
from xml.etree import ElementTree

sys.path.insert(0, os.path.abspath("../gisnav"))

# -- Version information -----------------------------------------------------


@dataclass(frozen=True)
class PackageData:
    """Stores data parsed from package.xml (not comprehensive)"""

    package_name: str
    version: str
    description: Optional[str]
    author: Optional[str]
    author_email: Optional[str]
    maintainer: Optional[str]
    maintainer_email: Optional[str]
    license_name: Optional[str]

    @staticmethod
    def require_not_none(text: Optional[Union[str, ElementTree.Element]]):
        """Raises ValueError if input is not a string but None"""
        if text is None:
            raise ValueError("Expected not None")
        return text

    @classmethod
    def parse_package_data(cls, package_file: str) -> PackageData:
        """Parses package.xml in current folder

        :param package_file: Absolute path to package.xml file
        :return: Parsed package data
        :raise FileNotFoundError: If package.xml file is not found
        """
        if os.path.isfile(package_file):
            tree = ElementTree.parse(package_file)
            root = tree.getroot()

            author = root.find("author")
            maintainer = root.find("maintainer")
            kwargs = {
                "package_name": cls.require_not_none(root.findtext("name")),
                "version": cls.require_not_none(root.findtext("version")),
                "description": root.findtext("description"),
                "author": root.findtext("author"),
                "author_email": author.attrib.get("email")
                if author is not None
                else None,
                "maintainer": root.findtext("maintainer"),
                "maintainer_email": maintainer.attrib.get("email")
                if maintainer is not None
                else None,
                "license_name": root.findtext("license"),
            }
            package_data = PackageData(**kwargs)
            return package_data
        else:
            raise FileNotFoundError(f"Could not find package file at {package_file}.")


package_data = PackageData.parse_package_data(os.path.abspath("../gisnav/package.xml"))

# -- Project information -----------------------------------------------------

project = package_data.package_name
copyright = f"2022, {package_data.author}"
author = package_data.author

# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.coverage",
    "sphinx.ext.autosectionlabel",
    "sphinx.ext.todo",
    "sphinx.ext.viewcode",
    "sphinx_design",
    "autodocsumm",
    "myst_parser",
    "sphinxcontrib.video",
    "sphinxcontrib.mermaid",
    "sphinx_copybutton",
    "sphinx_substitution_extensions",
]

# Add any paths that contain templates here, relative to this directory.
templates_path = ["_templates"]

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ["_build", "Thumbs.db", ".DS_Store"]

todo_include_todos = True

language = "en"

html_last_updated_fmt = "%b %d, %Y"

# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
html_theme = "pydata_sphinx_theme"

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ["_static"]

# Custom CSS styles
html_css_files = [
    "css/style.css",
]

html_js_files = [
    "js/custom.js",
]

# Configure pydata theming options here
html_theme_options = {
    "logo": {
        "image_light": "_static/svg/logo-no-background.svg",
        "image_dark": "_static/svg/logo-no-background-white.svg",
    },
    "collapse_navigation": True,
    "icon_links": [
        {
            "name": "GitHub",
            "url": "https://github.com/hmakelin/gisnav",
            "icon": "fab fa-github",
            "type": "fontawesome",
        }
    ],
    "icon_links_label": "Quick Links",
    "show_toc_level": 3,
    "primary_sidebar_end": ["indices.html"],
    "favicons": [
        {
            "rel": "icon",
            "sizes": "128x128",
            "href": "png/gisnav-website-favicon-color.png",
        },
    ],
}

# Handle dark mode mermaid diagrams - see also _static/js/custom.js
mermaid_init_js = """
    document.addEventListener('DOMContentLoaded', function(event) {
        var mermaidDivs = document.querySelectorAll('.mermaid');
        mermaidDivs.forEach(function(div) {
            div.setAttribute('data-mermaid', div.textContent);
        });
        mermaid.initialize({startOnLoad: true, theme: 'default'});
    });
"""

# -- Version information -----------------------------------------------------

# Make version number accessible in .rst files
# rst_epilog = f'.. |version| replace:: **v{package_data.version}**'
version = package_data.version
release = version

# Add git tag to release
try:
    release = (
        subprocess.check_output(["git", "describe", "--tags"]).strip().decode("utf-8")
    )
except subprocess.CalledProcessError:
    raise

ros_version = "humble"
# Define dynamic content (substitutions) here
# This should reduce documentation maintenance burden
# Substitutions must be in prolog (not epilot) - otherwise
# Sphinx-Substition-Extensions might not work (substitutions inside directives)
rst_prolog = f"""
.. |release| replace:: {release}
.. |version| replace:: {version}
.. |vversion| replace:: {'v' + version}
.. |ros_version| replace:: {ros_version}
.. |ros_version_capitalized| replace:: Humble
.. |ROS 2 install instructions| replace:: ROS 2 install instructions
.. _ROS 2 install instructions: https://docs.ros.org/en/{ros_version}/Installation.html
.. |Docker Compose file| replace:: Docker Compose file
.. _Docker Compose file: https://github.com/hmakelin/gisnav/blob/v{version}/docker/docker-compose.yaml
"""

rst_epilog = f"""
Updated on {datetime.datetime.today().strftime("%b %d, %Y")}

GISNav release: |release|
"""
