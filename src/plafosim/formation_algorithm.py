#
# Copyright (c) 2020 Julian Heinovski <heinovski@ccs-labs.org>
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

import logging
import sys

from abc import ABC, abstractmethod

LOG = logging.getLogger(__name__)


class FormationAlgorithm(ABC):
    def __init__(self, name: str, owner: object):
        self._name = name  # the name of the formation algorithm
        from .platooning_vehicle import PlatooningVehicle  # noqa 811
        from .infrastructure import Infrastructure
        assert(isinstance(owner, PlatooningVehicle) or isinstance(owner, Infrastructure))
        self._owner = owner  # the owning vehicle

    @property
    def name(self):
        return self._name

    @abstractmethod
    def do_formation(self):
        sys.exit("ERROR: There shouldn't be an instance of this abstract base class!")
