#
# Copyright (c) 2020-2022 Julian Heinovski <heinovski@ccs-labs.org>
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


class Dummy(FormationAlgorithm):
    """
    Dummy Formation Algorithm
    """

    def __init__(
        self,
        owner: object,
        **kw_args,
    ):

        """
        Initializes an instance of this formation algorithm to be used in a vehicle or an infrastructure.

        Parameters
        ----------
        owner : object
            The owning object that is execution this algorithm.
            This can be either a PlatooningVehicle or an Infrastructure.
        """

        super().__init__(owner)

    @classmethod
    def add_parser_argument_group(cls, parser: argparse.ArgumentParser):
        group = parser.add_argument_group(f"Formation Algorithm -- {cls.__name__}")
        group.add_argument(
            "--dummy",
            type=int,
            default=-1,
            help="A dummy parameter",
        )

    def do_formation(self):
        """
        Runs platoon formation algorithm to search for a platooning opportunity
        and performs the corresponding maneuvers.
        """

        from ..infrastructure import Infrastructure
        if isinstance(self._owner, Infrastructure):
            print(f'Running formation algorithm {self.name} on infrastructure! Unfortunately, this is just a dummy!')
        else:
            print(f'Running formation algorithm {self.name} on a vehicle! Unfortunately, this is just a dummy!')

    def finish(self):
        """
        Cleans up the instance of the formation algorithm.

        This includes mostly statistic recording.
        """

        pass
