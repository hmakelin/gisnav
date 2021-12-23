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
# TODO: same logic as in setup.py - make util function?
import xml.etree.ElementTree as ET

folder = os.path.dirname(os.path.realpath(__file__))

# Parse info from package.xml
package_file = os.path.join(folder, '../package.xml')
if os.path.isfile(package_file):
    tree = ET.parse(package_file)
    root = tree.getroot()
    package_name = root.find('name').text
    version = root.find('version').text
    description = root.find('description').text
    author = root.find('author').text
    author_email = root.find('author').attrib.get('email', '')
    maintainer = root.find('maintainer').text
    maintainer_email = root.find('maintainer').attrib.get('email', '')
    license_name = root.find('license').text
else:
    raise FileNotFoundError(f'Could not find package file at {package_file}.')

#version_string = f'v{version}'
#rst_prolog = f"""
#python_px4_ros2_map_nav {version_string}
#"""

# -- Add SuperGlue to path ---------------------------------------------------
#from python_px4_ros2_map_nav.util import setup_sys_path
#_, __ = setup_sys_path()
# TODO: same logic as in setup_sys_path
if 'get_package_share_directory' not in sys.modules:
    from ament_index_python.packages import get_package_share_directory
share_dir = get_package_share_directory(package_name)
superglue_dir = os.path.join(share_dir, 'SuperGluePretrainedNetwork')
sys.path.append(os.path.abspath(superglue_dir))

# -- Project information -----------------------------------------------------

project = package_name
copyright = f'2021, {author}'
author = author


# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = ['sphinx.ext.autodoc', 'sphinx.ext.coverage', 'sphinx_rtd_theme', 'myst_parser']

# Add any paths that contain templates here, relative to this directory.
templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']


# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
import sphinx_rtd_theme
html_theme = 'sphinx_rtd_theme'

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
html_static_path = ['_static']
