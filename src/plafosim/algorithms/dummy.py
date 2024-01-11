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
import logging

from ..formation_algorithm import FormationAlgorithm

LOG = logging.getLogger(__name__)

# default values for this algorithm's parameters
DEFAULTS = {
    'dummy': -1,
}


class Dummy(FormationAlgorithm):
    """
    Dummy Platoon Formation Algorithm.
    """

    def __init__(
        self,
        owner: object,
        dummy: int = DEFAULTS['dummy'],
        **kw_args,
    ):
        """
        Initialize an instance of this formation algorithm to be used in a vehicle or an infrastructure.

        Parameters
        ----------
        owner : object
            The owning object that is execution this algorithm.
            This can be either a PlatooningVehicle or an Infrastructure.
        dummy : int, optional
            The value for the dummy parameter
        """

        super().__init__(owner)

        self._dummy = dummy

    @classmethod
    def add_parser_argument_group(self, parser: argparse.ArgumentParser) -> argparse._ArgumentGroup:
        """
        Create and return specific argument group for this algorithm to use in global argument parser.

        Parameters
        ----------
        parser : argparse.ArgumentParser
            The global argument parser

        Returns
        -------
        argparse._ArgumentGroup
            The specific argument group for this algorithm
        """

        group = parser.add_argument_group(f"Formation Algorithm -- {self.__name__}")
        group.add_argument(
            "--dummy",
            type=int,
            default=DEFAULTS['dummy'],
            help="A dummy parameter",
        )
        return group

    def do_formation(self):
        """
        Run platoon formation algorithm to search for a platooning opportunity and perform the corresponding join maneuver.
        """

        from ..infrastructure import Infrastructure
        if isinstance(self._owner, Infrastructure):
            print(f'Running formation algorithm {self.name} on infrastructure {self.owner.iid}. Unfortunately, this is just a dummy!')
        else:
            print(f'Running formation algorithm {self.name} on a vehicle {self._owner.vid}. Unfortunately, this is just a dummy!')

    def finish(self):
        """
        Clean up the instance of the formation algorithm.

        This includes mostly statistic recording.
        """

        pass
