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
import sys
from abc import ABC, abstractmethod

LOG = logging.getLogger(__name__)


class FormationAlgorithm(ABC):
    """
    Abstract base class for any type of platoon formation algorithm (i.e., assignment calculation).

    Implementing sub-classes need to override the do_formation() method.
    """

    def __init__(self, owner: object):
        """
        Initializes an instance of a formation algorithm.

        Parameters
        ----------
        owner : object
            The owning object that is execution this algorithm.
            This can be either a PlatooningVehicle or an Infrastructure.
        """

        from .infrastructure import Infrastructure
        from .platooning_vehicle import PlatooningVehicle  # noqa 811
        assert (isinstance(owner, PlatooningVehicle) or isinstance(owner, Infrastructure))
        self._owner = owner  # the owning vehicle or infrastructure

    @property
    def name(self):
        """Prints the name of the formation algorithm."""

        return self.__class__.__name__

    @abstractmethod
    def add_parser_argument_group(cls, parser: argparse.ArgumentParser) -> argparse._ArgumentGroup:
        """
        Abstract method for performing any type of platoon formation (i.e., assignment calculation).

        This methods needs to be overridden in implementing sub-classes.
        """

        sys.exit("ERROR: There shouldn't be an instance of this abstract base class!")

    @abstractmethod
    def do_formation(self):
        """
        Abstract method for performing any type of platoon formation (i.e., assignment calculation).

        This methods needs to be overridden in implementing sub-classes.
        """

        sys.exit("ERROR: There shouldn't be an instance of this abstract base class!")

    def finish(self):
        """Reserved for future use."""

        pass
