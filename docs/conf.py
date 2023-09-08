# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import sys

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'ROS 2 DEF'
copyright = '2023, Jonas Otto'
author = 'Jonas Otto'

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = [
    'sphinx.ext.autodoc',
    'sphinx-jsonschema',
    'sphinx.ext.intersphinx',
    'breathe'
]
autoclass_content = 'both'
sys.path.insert(0, '../ros2/orchestrator')

jsonschema_options = {
    'lift_definitions': True,
    'auto_reference': True,
    'lift_description': True,
}

intersphinx_mapping = {
    "rclcpp": ("https://docs.ros.org/en/rolling/p/rclcpp", None),
    "rclpy": ("https://docs.ros.org/en/rolling/p/rclpy/", None),
    "tf2_ros_py": ("https://docs.ros.org/en/rolling/p/tf2_ros_py/", None),
}
intersphinx_disabled_reftypes = ["*"]

breathe_projects = {
    "orchestrator_helper": "../ros2/orchestrator_helper/docs/xml"
}
breathe_default_project = "orchestrator_helper"

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']
numfig = True

# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'alabaster'
