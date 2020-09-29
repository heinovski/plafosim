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

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .platooning_vehicle import PlatooningVehicle


class FormationAlgorithm(ABC):
    def __init__(self, name: str, owner):
        self._name = name  # the name of the formation algorithm
        from .platooning_vehicle import PlatooningVehicle
        from .infrastructure import Infrastructure
        assert(isinstance(owner, PlatooningVehicle) or isinstance(owner, Infrastructure))
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
        return self.position_deviation_threshold

    def _ds(self, vehicle: 'PlatooningVehicle', other_vehicle: 'PlatooningVehicle'):
        return abs(vehicle.desired_speed - other_vehicle.speed)

    def _dp(self, vehicle: 'PlatooningVehicle', other_vehicle: 'PlatooningVehicle'):
        if vehicle.rear_position > other_vehicle.position:
            # we are in front of the neighbor
            return abs(vehicle.rear_position - other_vehicle.position)
        else:
            # we are behind the neighbor
            return abs(other_vehicle.rear_position - vehicle.position)

    def _cost_speed_position(self, ds: float, dp: int, alpha: float, beta: float):
        return (alpha * ds) + (beta * dp)

    def do_formation(self):
        """Run platoon formation algorithms to search for a platooning opportunity and perform corresponding maneuvers"""

        from .infrastructure import Infrastructure
        if isinstance(self._owner, Infrastructure):
            possible_matches = []
            # select all searching vehicles
            for vehicle in self._owner._simulator._vehicles.values():
                logging.info("Running formation algorithm %s for %d" % (self.name, vehicle.vid))

                # filter vehicles which are already in a platoon
                if vehicle.platoon_role != PlatoonRole.NONE:
                    logging.debug("%d is already in a platoon" % vehicle.vid)
                    continue
                # filter vehicles which are already in a maneuver
                if vehicle.in_maneuver:
                    logging.debug("%d is already in a maneuver" % vehicle.vid)
                    continue

                # select all available candidates
                for other_vehicle in self._owner._simulator._vehicles.values():
                    # filter same car because we assume driving alone is worse than to do platooning
                    if other_vehicle is vehicle:
                        continue
                    # filter vehicles which are not available to become a new leader
                    # we only have this information due to oracle knowledge in the centralized version
                    if other_vehicle.platoon_role != PlatoonRole.NONE and other_vehicle.platoon_role != PlatoonRole.LEADER:
                        # we can only join an existing platoon or built a new one
                        logging.debug("%d is not available" % vehicle.vid)
                        continue
                    # filter vehicles which are already in a maneuver
                    # we only have this information due to oracle knowledge in the centralized version
                    if other_vehicle.in_maneuver:
                        logging.debug("%d is not available" % vehicle.vid)
                        continue

                    # for one vehicle A we are looking at a different vehicle B to
                    # check whether it is useful that A joins B

                    # calculate deviation values
                    ds = self._ds(vehicle, other_vehicle)
                    dp = self._dp(vehicle, other_vehicle)

                    # TODO HACK for skipping vehicles in front of us
                    if vehicle.position > other_vehicle.rear_position:
                        logging.debug("%d's other_vehicle %d not applicable because of its absolute position" % (vehicle.vid, other_vehicle.vid))
                        continue

                    # remove other_vehicle if not in speed range
                    if ds > self._speed_deviation_threshold * vehicle.desired_speed:
                        logging.debug("%d's other_vehicle %d not applicable because of its speed difference" % (vehicle.vid, other_vehicle.vid))
                        continue

                    # remove other_vehicle if not in position range
                    if dp > self._position_deviation_threshold:
                        logging.debug("%d's other_vehicle %d not applicable because of its position difference" % (vehicle.vid, other_vehicle.vid))
                        continue

                    # calculate deviation/cost
                    fx = self._cost_speed_position(ds, dp, self._alpha, 1 - self._alpha)

                    # add neighbor to list
                    possible_matches.append({'from': vehicle.vid, 'to': other_vehicle.vid, 'pid': other_vehicle.platoon.platoon_id, 'lid': other_vehicle.platoon.leader.vid, 'cost': fx})
                    logging.info("%d found applicable candidates %d" % (vehicle.vid, other_vehicle.vid))

            if len(possible_matches) == 0:
                logging.debug("%d found no possible matches" % self._owner.iid)
                return

            # get unique list of searching vehicles from within the possible matches
            uids = set()
            uids = [x['from'] for x in possible_matches if x['from'] not in uids and (uids.add(x['from']) or True)]

            for v in uids:
                # get vehicle data and candidates
                vehicle = self._owner._simulator._vehicles[v]
                found_candidates = [x for x in possible_matches if x['from'] == v]

                if len(found_candidates) == 0:
                    # this vehicle has no candidates (anymore)
                    logging.debug("%d has no candidates (anymore)" % vehicle.vid)
                    continue

                # find best candidate to join
                # pick the neighbor with the lowest deviation
                best = min(found_candidates, key=lambda x: x['cost'])

                logging.info("%d' best candidate is %d from platoon %d (leader %d) with %d" % (vehicle.vid, best['to'], best['pid'], best['lid'], best['cost']))

                # perform a join maneuver with the candidate's platoon
                # do we really want the candidate to advertise its platoon or do we just want the leader to advertise its platoon?
                vehicle._join(best['pid'], best['lid'])

                # remove all matches from the list of possible matches that would include
                def is_available(x: dict) -> bool:
                    return (x['from'] != best['from'] and  # obviously this vehicle does not search anymore
                            x['to'] != best['from'] and  # this vehicle is not applicable as target anymore
                            x['from'] != best['to'])  # the target vehicle is not applicable anymore
#                            x['to'] != best['to'])  # the target vehicle is still available  # TODO

                possible_matches = [x for x in possible_matches if is_available(x)]
        else:
            logging.info("%d is running formation algorithm %s" % (self._owner.vid, self.name))

            # we can only run the algorithm if we are not yet in a platoon
            # because this algorithm does not support changing the platoon later on
            if self._owner.platoon_role != PlatoonRole.NONE:
                return

            found_candidates = []

            # get all available vehicles from the neighbor table
            for neighbor in self._owner._get_neighbors():

                # calculate deviation values
                ds = self._ds(self._owner, neighbor)
                dp = self._dp(self._owner, neighbor)

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
