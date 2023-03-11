# Configuration file for the Sphinx documentation builder.
#
# This file only contains a selection of the most common options. For a full
# list see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Path setup --------------------------------------------------------------

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
#
import os
import sys
sys.path.insert(0, os.path.abspath('../'))

# -- Version information -----------------------------------------------------
from gisnav.data import PackageData
package_data = PackageData.parse_package_data(os.path.abspath('../package.xml'))


# -- Add SuperGluePoseEstimator to path ---------------------------------------------------
if 'get_package_share_directory' not in sys.modules:
    from ament_index_python.packages import get_package_share_directory
share_dir = get_package_share_directory(package_data.package_name)
superglue_dir = os.path.join(share_dir, 'SuperGluePretrainedNetwork')
sys.path.append(os.path.abspath(superglue_dir))

# -- Project information -----------------------------------------------------

project = package_data.package_name
copyright = f'2022, {package_data.author}'
author = package_data.author

# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = ['sphinx.ext.autodoc',
              'sphinx.ext.coverage',
              'sphinx.ext.autosectionlabel',
              'sphinx_design',
              'autodocsumm',
              'myst_parser',
              'sphinxcontrib.video',
              ]

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
html_theme = 'pydata_sphinx_theme'

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']

# Custom CSS styles
html_css_files = [
    'css/style.css',
]

# Configure pydata theming options here
html_theme_options = {
    "logo": {
        "image_light": "img/logo.png",
        "image_dark": "img/logo_inverted.png",
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
}

# Make version number accessible in .rst files
#rst_epilog = f'.. |version| replace:: **v{package_data.version}**'
version = package_data.version
release = version
