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

# If extensions (or modules to document with autodoc) are in another directory,
# add these directories to sys.path here. If the directory is relative to the
# documentation root, use os.path.abspath to make it absolute, like shown here.
sys.path.insert(0, os.path.abspath('../src'))

from plafosim import  __version__  # noqa 402


# -- Project information -----------------------------------------------------

project = 'PlaFoSim'
copyright = '2020-2024, Julian Heinovski'
author = 'Julian Heinovski'

# The full version, including alpha/beta/rc tags
release = __version__

# -- General configuration ---------------------------------------------------

# Add any Sphinx extension module names here, as strings. They can be
# extensions coming with Sphinx (named 'sphinx.ext.*') or your custom
# ones.
extensions = [
    'myst_parser',
    'sphinx.ext.autodoc',
    'sphinx.ext.autosectionlabel',
    'sphinx.ext.autosummary',
    'sphinx.ext.coverage',
    'sphinx.ext.doctest',
    'sphinx.ext.extlinks',
    'sphinx.ext.ifconfig',
    'sphinx.ext.napoleon',
    'sphinx.ext.todo',
    'sphinx.ext.viewcode',
    'sphinx_rtd_theme',
]
source_suffix = {
    '.rst': 'restructuredtext',
    '.txt': 'markdown',
    '.md': 'markdown',
}

autodoc_default_options = {
    'member-order': 'groupwise',
    'undoc-members': True,  # don't hide undocumented members
    'members': True,  # show members by default
    'attributes': True,
    'show-inheritance': True,
    'imported-members': True,
    'inherited-members': True,
    'special-members': '__init__',
    'autosummary': True,  # for 'autodocsumm' extension (separate package)
    'autosummary_generate': True,
}
autosummary_generate = True
autoclass_content = 'class'

# Add any paths that contain templates here, relative to this directory.
#templates_path = ['_templates']

# List of patterns, relative to source directory, that match files and
# directories to ignore when looking for source files.
# This pattern also affects html_static_path and html_extra_path.
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

numfig = True

# -- Options for HTML output -------------------------------------------------

# The theme to use for HTML and HTML Help pages.  See the documentation for
# a list of builtin themes.
#
html_theme = 'sphinx_rtd_theme'

html_theme_options = {
    # Toc options
    'collapse_navigation': False,
    'sticky_navigation': True,
    'navigation_depth': 4,
    'includehidden': True,
    'titles_only': False
}

# Add any paths that contain custom static files (such as style sheets) here,
# relative to this directory. They are copied after the builtin static files,
# so a file named "default.css" will overwrite the builtin "default.css".
#html_static_path = ['_static']

### The dirtiest HACK among all HACKS ###
# Replace CHANGELOG with CHANGELOG.html after including README.md
# based on https://stackoverflow.com/a/52998585
from sphinx.transforms import SphinxTransform  # noqa 402


class ReplaceMyBase(SphinxTransform):

    default_priority = 750
    prefix = 'CHANGELOG'

    def apply(self):
        from docutils.nodes import reference, Text
        baseref = lambda o: (isinstance(o, reference) and o.get('refuri', '').startswith(self.prefix))
        basetext = lambda o: (isinstance(o, Text) and o.startswith(self.prefix))
        base = self.config.mybase.rstrip('/')
        for node in self.document.traverse(baseref):
            target = node['refuri'].replace(self.prefix, base, 1)
            node.replace_attr('refuri', target)
            for t in node.traverse(basetext):
                t1 = Text(t.replace(self.prefix, base, 1), t.rawsource)
                t.parent.replace(t, t1)
        return

# end of class


def setup(app):
    app.add_config_value('mybase', 'CHANGELOG.html', 'env')
    app.add_transform(ReplaceMyBase)
    return
