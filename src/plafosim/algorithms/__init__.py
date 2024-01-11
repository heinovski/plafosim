#
# Copyright (c) 2020-2024 Julian Heinovski <heinovski@ccs-labs.org>
#
# SPDX-License-Identifier: GPL-3.0-or-later
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#

import os
from importlib import import_module
from inspect import isclass
from pkgutil import iter_modules

from ..formation_algorithm import FormationAlgorithm
from .dummy import Dummy

# iterate through the modules in the current package
# and dynamically load all available algorithms
globals()['algorithms'] = list()
# based on https://julienharbulot.com/python-dynamical-import.html
package_dir = os.path.dirname(__file__)
for (_, module_name, _) in iter_modules([package_dir]):
    # import the module and iterate through its attributes
    module = import_module(f"{__name__}.{module_name}")
    for attribute_name in dir(module):
        attribute = getattr(module, attribute_name)

        if (
            isclass(attribute)  # require is class
            and issubclass(attribute, FormationAlgorithm)  # require is sub-class
            and attribute != FormationAlgorithm  # skip abstract base class
            and attribute != Dummy  # skip dummy algorithm
        ):
            # add the class to this package's variables
            globals()[attribute_name] = attribute
            globals()['algorithms'].append(attribute_name)
