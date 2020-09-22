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

from abc import ABC, abstractmethod

from .platoon_role import PlatoonRole


class FormationAlgorithm(ABC):
    def __init__(self, name: str, owner):
        self._name = name  # the name of the formation algorithm
        from .platooning_vehicle import PlatooningVehicle
        assert(isinstance(owner, PlatooningVehicle))
        self._owner = owner  # the owning vehicle

    @property
    def name(self):
        return self._name

    @abstractmethod
    def do_formation(self):
        print("There shouldn't be an instance of this abstract base class!")
        exit(1)


class SpeedPosition(FormationAlgorithm):
    def __init__(self, owner,
                 alpha: float,
                 speed_deviation_threshold: float,
                 position_deviation_threshold: int):
        super().__init__(self.__class__.__name__, owner)

        assert(alpha >= 0 and alpha <= 1.0)
        self._alpha = alpha
        self._speed_deviation_threshold = speed_deviation_threshold
        self._position_deviation_threshold = position_deviation_threshold

    @property
    def alpha(self) -> float:
        return self._alpha

    @property
    def speed_deviation_threshold(self) -> float:
        return self.speed_deviation_threshold

    @property
    def position_deviation_threshold(self) -> int:
        return self.speed_deviation_threshold

    def _ds(self, neighbor_speed: float):
        return abs(self._owner.desired_speed - neighbor_speed)

    def _dp(self, neighbor_position: int, neighbor_rear_position: int):
        if self._owner.rear_position > neighbor_position:
            # we are in front of the neighbor
            return abs(self._owner.rear_position - neighbor_position)
        else:
            # we are behind the neighbor
            return abs(neighbor_rear_position - self._owner.position)

    def _cost_speed_position(self, ds: float, dp: int, alpha: float, beta: float):
        return (alpha * ds) + (beta * dp)

    def do_formation(self):
        """Run platoon formation algorithms to search for a platooning opportunity and perform corresponding maneuvers"""

        logging.info("%d is running formation algorithm %s" % (self._owner.vid, self.name))

        # we can only run the algorithm if we are not yet in a platoon
        # because this algorithm does not support changing the platoon later on
        if self._owner.platoon_role != PlatoonRole.NONE:
            return

        found_candidates = []

        # get all available vehicles from the neighbor table
        for neighbor in self._owner._get_neighbors():

            # calculate deviation values
            ds = self._ds(neighbor.speed)
            dp = self._dp(neighbor.position, neighbor.rear_position)

            # TODO HACK for skipping vehicles in front of us
            if self._owner.position > neighbor.rear_position:
                logging.debug("%d's neighbor %d not applicable because of its absolute position" % (self._owner.vid, neighbor.vid))
                continue

            # remove neighbor if not in speed range
            if ds > self._speed_deviation_threshold * self._owner.desired_speed:
                logging.debug("%d's neighbor %d not applicable because of its speed difference" % (self._owner.vid, neighbor.vid))
                continue

            # remove neighbor if not in position range
            if dp > self._position_deviation_threshold:
                logging.debug("%d's neighbor %d not applicable because of its position difference" % (self._owner.vid, neighbor.vid))
                continue

            # calculate deviation/cost
            fx = self._cost_speed_position(ds, dp, self._alpha, 1 - self._alpha)

            # add neighbor to list
            found_candidates.append((neighbor.vid, neighbor.platoon.platoon_id, neighbor.platoon.leader.vid, fx))
            logging.info("%d found %d applicable candidates" % (self._owner.vid, len(found_candidates)))

        if len(found_candidates) == 0:
            return

        # find best candidate to join
        # pick the neighbor with the lowest deviation
        best = min(found_candidates, key=lambda x: x[3])

        logging.info("%d' best candidate is %d from platoon %d (leader %d) with %d" % (self._owner.vid, best[0], best[1], best[2], best[3]))

        # perform a join maneuver with the candidate's platoon
        # do we really want the candidate to advertise its platoon or do we just want the leader to advertise its platoon?
        self._owner._join(best[1], best[2])
