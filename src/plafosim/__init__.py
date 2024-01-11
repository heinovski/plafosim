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

import argparse
import textwrap

from plafosim.util import add_logging_level

__version__ = "0.17.3"

__description__ = textwrap.dedent(
    f"""\
    Platoon Formation Simulator (PlaFoSim) -- Version {__version__}.
    A simple and scalable simulator for platoon formation.
    Website: https://www.plafosim.de/

    Copyright (c) 2020-2024 Julian Heinovski <heinovski@ccs-labs.org>

    SPDX-License-Identifier: GPL-3.0-or-later

    This program comes with ABSOLUTELY NO WARRANTY.
    This is free software, and you are welcome to redistribute it under certain conditions.

    If you are working with PlaFoSim, please cite the following paper:

    Julian Heinovski, Dominik S. Buse and Falko Dressler,
    "Scalable Simulation of Platoon Formation Maneuvers with PlaFoSim,"
    Proceedings of 13th IEEE Vehicular Networking Conference (VNC 2021),
    Poster Session, Virtual Conference, November 2021, pp. 137–138.
    https://www.tkn.tu-berlin.de/bib/heinovski2021scalable/
"""
)

__citation__ = textwrap.dedent(
    """\
    @inproceedings{heinovski2021scalable,
        author = {Heinovski, Julian and Buse, Dominik S. and Dressler, Falko},
        doi = {10.1109/VNC52810.2021.9644678},
        title = {{Scalable Simulation of Platoon Formation Maneuvers with PlaFoSim}},
        pages = {137--138},
        publisher = {IEEE},
        issn = {2157-9865},
        isbn = {978-1-66544-450-7},
        address = {Virtual Conference},
        booktitle = {13th IEEE Vehicular Networking Conference (VNC 2021), Poster Session},
        month = {11},
        year = {2021},
    }
"""
)

# add additional custom log level
add_logging_level("TRACE", 5)


class CustomFormatter(
    argparse.ArgumentDefaultsHelpFormatter,
    argparse.RawDescriptionHelpFormatter,
    argparse.MetavarTypeHelpFormatter,
):
    """
    Metaclass combining multiple formatter classes for argparse.
    """
