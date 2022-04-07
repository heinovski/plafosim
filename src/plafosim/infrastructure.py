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

import logging
import sys

from .algorithms.speed_position import SpeedPosition
from .platooning_vehicle import PlatooningVehicle

#from .simulator import Simulator  # TODO fix circular import

LOG = logging.getLogger(__name__)


class Infrastructure:

    def __init__(
            self,
            simulator,
            iid: int,
            position: int,
            formation_algorithm: str,
            execution_interval: int,
            **kw_args,
    ):
        """
        Parameters
        ----------
        simulator : Simulator
            The global simulator object
        iid : int
            The id of the infrastructure
        position : int
            The position (x) of the infrastructure
        formation_algorithm : str
            The platoon formation (i.e., assignment calculation) algorithm to run
        execution_interval : int
            The execution interval for the formation algorithm
        """

        self._simulator = simulator  # the simulator
        self._iid = iid  # the id of the infrastructure
        self._position = position  # the x position of the infrastructure

        if formation_algorithm is not None:
            # initialize formation algorithm
            # TODO make enum
            if formation_algorithm == SpeedPosition.__name__:
                self._formation_algorithm = SpeedPosition(self, **kw_args)
            else:
                sys.exit(f"ERROR: Unknown formation algorithm {formation_algorithm}!")
            self._execution_interval = execution_interval

            # initialize timer
            self._last_formation_step = 0  # initialize with vehicle start
        else:
            self._formation_algorithm = None

    @property
    def iid(self) -> int:
        """Returns the id of an infrastructure."""

        return self._iid

    @property
    def position(self) -> int:
        """Returns the position of an infrastructure."""

        return self._position

    def action(self, step: int):
        """
        Triggers actions of an infrastructure

        Parameters
        ----------
        step : int
            The current simulation step
        """

        # What has to be triggered periodically?
        if self._simulator._actions:
            self._action(step)

    def _action(self, step: int):
        """
        Triggers concrete actions of an infrastructure.

        Parameters
        ----------
        step : int
            The current simulation step
        """

        LOG.trace(f"{self.iid} was triggered at {step}")

        if self._formation_algorithm:
            if step >= self._last_formation_step + self._execution_interval:
                # search for a platoon (depending on the algorithm)
                self._formation_algorithm.do_formation()
                self._last_formation_step = step

    # TODO currently not used --> remove?
    def _get_neighbors(self):
        neighbors = []
        for vehicle in self._simulator._vehicles.values():

            # filter vehicles that are technically not able to do platooning
            if not isinstance(vehicle, PlatooningVehicle):
                continue

            # filter based on communication range
            communication_range = self._simulator.road_length
            if abs(vehicle.position - self.position) > communication_range:
                continue

            neighbors.append(vehicle)

        return neighbors

    def finish(self):
        """Cleans up the instance of the infrastructure"""

        if self._formation_algorithm is not None:
            self._formation_algorithm.finish()
