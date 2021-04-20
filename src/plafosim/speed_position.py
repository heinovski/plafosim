#
# Copyright (c) 2020-2021 Julian Heinovski <heinovski@ccs-labs.org>
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

from .formation_algorithm import FormationAlgorithm
from .platoon_role import PlatoonRole

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .platooning_vehicle import PlatooningVehicle  # noqa 401
    from .platooning_vehicle import Platoon  # noqa 401

LOG = logging.getLogger(__name__)


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

        # statistics
        self._assignments_solved = 0
        self._assigments_not_solveable = 0
        self._assignments_none = 0
        self._assignments_self = 0
        self._assignments_candidate_joined_already = 0
        self._assingments_vehicle_became_leader = 0
        self._assignments_successful = 0

    @property
    def alpha(self) -> float:
        return self._alpha

    @property
    def speed_deviation_threshold(self) -> float:
        return self.speed_deviation_threshold

    @property
    def position_deviation_threshold(self) -> int:
        return self.position_deviation_threshold

    def _ds(self, vehicle: 'PlatooningVehicle', platoon: 'Platoon'):
        return abs(vehicle.desired_speed - platoon.desired_speed)

    def _dp(self, vehicle: 'PlatooningVehicle', platoon: 'Platoon'):
        if vehicle.rear_position > platoon.position:
            # we are in front of the platoon
            return abs(vehicle.rear_position - platoon.position)
        else:
            # we are behind the platoo
            return abs(platoon.rear_position - vehicle.position)

    def _cost_speed_position(self, ds: float, dp: int, alpha: float, beta: float):
        return (alpha * ds) + (beta * dp)

    def do_formation(self):
        """Run platoon formation algorithms to search for a platooning opportunity and perform corresponding maneuvers"""

        from .infrastructure import Infrastructure
        if isinstance(self._owner, Infrastructure):
            LOG.info(f"{self._owner.iid} is running formation algorithm {self.name} ({self._owner._formation_kind}) at {self._owner._simulator.step}")
            if self._owner._formation_kind == 'optimal':
                self._do_formation_optimal()
            else:
                self._do_formation_centralized()
        else:
            self._do_formation_distributed()

    def finish(self):
        # write statistics
        if not self._owner._simulator._record_platoon_formation:
            return
        if self._owner._formation_kind != 'optimal':
            return

        from .infrastructure import Infrastructure
        assert(isinstance(self._owner, Infrastructure))

        if self._owner._simulator._record_infrastructure_assignments:
            with open(f'{self._owner._simulator._result_base_filename}_infrastructure_assignments.csv', 'a') as f:
                f.write(
                    f"{self._owner.iid},"
                    f"{self._assignments_solved},"
                    f"{self._assigments_not_solveable},"
                    f"{self._assignments_none},"
                    f"{self._assignments_self},"
                    f"{self._assignments_candidate_joined_already},"
                    f"{self._assingments_vehicle_became_leader},"
                    f"{self._assignments_successful}"
                    "\n"
                )

    def _do_formation_distributed(self):
        # we can only run the algorithm if we are not yet in a platoon
        # because this algorithm does not support changing the platoon later on
        if self._owner.platoon_role != PlatoonRole.NONE:
            LOG.debug(f"{self._owner.vid} is already in a platoon")
            return

        # only if not currently in a maneuver
        if self._owner.in_maneuver:
            LOG.debug(f"{self._owner.vid} is already in a maneuver")
            return

        LOG.info(f"{self._owner.vid} is running formation algorithm {self.name} (distributed)")

        found_candidates = []

        # get all available platoons or platoon candidates
        for platoon in self._owner._get_available_platoons():

            # calculate deviation values
            ds = self._ds(self._owner, platoon)
            dp = self._dp(self._owner, platoon)

            # TODO HACK for skipping platoons in front of us
            if self._owner.position > platoon.rear_position:
                LOG.debug(f"{self._owner.vid}'s platoon {platoon.platoon_id} not applicable because of its absolute position")
                continue

            # remove platoon if not in speed range
            if ds > self._speed_deviation_threshold * self._owner.desired_speed:
                LOG.debug(f"{self._owner.vid}'s platoon {platoon.platoon_id} not applicable because of its speed difference")
                continue

            # remove platoon if not in position range
            if dp > self._position_deviation_threshold:
                LOG.debug(f"{self._owner.vid}'s platoon {platoon.platoon_id} not applicable because of its position difference")
                continue

            # calculate deviation/cost
            fx = self._cost_speed_position(ds, dp, self._alpha, 1 - self._alpha)

            # add platoon to list
            found_candidates.append({'vid': self._owner.vid, 'pid': platoon.platoon_id, 'lid': platoon.leader.vid, 'cost': fx})
            LOG.info(f"{self._owner.vid} found {len(found_candidates)} applicable candidates")

        if len(found_candidates) == 0:
            LOG.debug(f"{self._owner.vid} has no candidates")
            return

        # the number of candidates found in this iteration
        self._owner._candidates_found += len(found_candidates)

        # find best candidate to join
        # pick the platoon with the lowest deviation
        best = min(found_candidates, key=lambda x: x['cost'])
        LOG.info(f"{self._owner.vid}'s best platoon is {best['pid']} (leader {best['lid']}) with {best['cost']}")

        # perform a join maneuver with the candidate's platoon
        # do we really want the candidate to advertise its platoon or do we just want the leader to advertise its platoon?
        self._owner._join(best['pid'], best['lid'])

    def _do_formation_centralized(self):
        all_found_candidates = []

        from .platooning_vehicle import PlatooningVehicle  # noqa 811
        # select all searching vehicles
        for vehicle in self._owner._simulator._vehicles.values():
            # filter vehicles that are technically not able to do platooning
            if not isinstance(vehicle, PlatooningVehicle):
                LOG.debug(f"{vehicle.vid} is not capable of platooning")
                continue
            # filter vehicles which are already in a platoon
            if vehicle.platoon_role != PlatoonRole.NONE:
                LOG.debug(f"{vehicle.vid} is already in a platoon")
                continue
            # filter vehicles which are already in a maneuver
            if vehicle.in_maneuver:
                LOG.debug(f"{vehicle.vid} is already in a maneuver")
                continue

            # get all available platoons or platoon candidates
            for other_vehicle in self._owner._simulator._vehicles.values():
                # filter same car because we assume driving alone is worse than to do platooning
                if other_vehicle is vehicle:
                    continue
                # filter vehicles that are technically not able to do platooning
                if not isinstance(other_vehicle, PlatooningVehicle):
                    LOG.debug(f"{other_vehicle.vid} is not capable of platooning")
                    continue

                # here, the logic similar to the distributed approach begins

                # filter vehicles which are not available to become a new leader
                # we only have this information due to oracle knowledge in the centralized version
                # we use this to replace management of neighbors and their advertisements
                if other_vehicle.platoon_role != PlatoonRole.NONE and other_vehicle.platoon_role != PlatoonRole.LEADER:
                    # we can only join an existing platoon or built a new one
                    LOG.debug(f"{other_vehicle.vid} is not available")
                    vehicle._candidates_filtered += 1
                    vehicle._candidates_filtered_follower += 1
                    continue

                # filter vehicles which are already in a maneuver
                # we only have this information due to oracle knowledge in the centralized version
                if other_vehicle.in_maneuver:
                    LOG.debug(f"{other_vehicle.vid} is already in a maneuver")
                    vehicle._candidates_filtered += 1
                    vehicle._candidates_filtered_maneuver += 1
                    continue

                platoon = other_vehicle.platoon

                # for one vehicle A we are looking at a different vehicle B to
                # check whether it is useful that A joins B

                # calculate deviation values
                ds = self._ds(vehicle, platoon)
                dp = self._dp(vehicle, platoon)

                # TODO HACK for skipping vehicles in front of us
                if vehicle.position > platoon.rear_position:
                    LOG.debug(f"{vehicle.vid}'s platoon {platoon.platoon_id} not applicable because of its absolute position")
                    continue

                # remove platoon if not in speed range
                if ds > self._speed_deviation_threshold * vehicle.desired_speed:
                    LOG.debug(f"{vehicle.vid}'s platoon {platoon.platoon_id} not applicable because of its speed difference")
                    continue

                # remove platoon if not in position range
                if dp > self._position_deviation_threshold:
                    LOG.debug(f"{vehicle.vid}'s platoon {platoon.platoon_id} not applicable because of its position difference")
                    continue

                # calculate deviation/cost
                fx = self._cost_speed_position(ds, dp, self._alpha, 1 - self._alpha)

                # add platoon to list
                all_found_candidates.append({'vid': vehicle.vid, 'pid': platoon.platoon_id, 'lid': platoon.leader.vid, 'cost': fx})
                LOG.info(f"{vehicle.vid} found applicable candidate {platoon.platoon_id}")
                vehicle._candidates_found += 1

        if len(all_found_candidates) == 0:
            LOG.debug(f"{self._owner.iid} found no possible matches")
            return

        # get unique list of searching vehicles from within the possible matches
        uids = set()
        uids = [x['vid'] for x in all_found_candidates if x['vid'] not in uids and (uids.add(x['vid']) or True)]

        for v in uids:
            # get vehicle data and candidates
            vehicle = self._owner._simulator._vehicles[v]
            found_candidates = [x for x in all_found_candidates if x['vid'] == v]

            if len(found_candidates) == 0:
                # this vehicle has no candidates (anymore)
                LOG.debug(f"{vehicle.vid} has no candidates (anymore)")
                continue

            # find best candidate to join
            # pick the platoon with the lowest deviation
            best = min(found_candidates, key=lambda x: x['cost'])
            LOG.info(f"{v}'s best platoon is {best['pid']} (leader {best['lid']}) with {best['cost']}")

            # perform a join maneuver with the candidate's platoon
            # do we really want the candidate to advertise its platoon or do we just want the leader to advertise its platoon?
            vehicle._join(best['pid'], best['lid'])

            # remove all matches from the list of possible matches that would include the selected vehicle
            def is_available(x: dict) -> bool:
                return (x['vid'] != best['vid'] and  # noqa 504 # this vehicle does not search anymore
                        x['lid'] != best['vid'] and  # noqa 504 # this vehicle will not be applicable as leader anymore # TODO what about transitive joins?
                        x['vid'] != best['lid'])  # the other vehicle is not searching anymore, since it will become a leader
            # it still possible to join the other vehicles (which will become a leader now)

            all_found_candidates = [x for x in all_found_candidates if is_available(x)]

    def _do_formation_optimal(self):
        from ortools.linear_solver import pywraplp
        solver = pywraplp.Solver(f"{self.name} solver", pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)

        import sys
        infinity = sys.float_info.max  # does work
        individual = 1e+19  # magic big number for driving individually

        objective = solver.Objective()
        objective.SetMinimization()

        decision_variables = {}

        from .platooning_vehicle import PlatooningVehicle  # noqa 811
        # select all searching vehicles
        for vehicle in self._owner._simulator._vehicles.values():

            # filter vehicles that are technically not able to do platooning
            if not isinstance(vehicle, PlatooningVehicle):
                LOG.debug(f"{vehicle.vid} is not capable of platooning")
                continue
            # filter vehicles which are already in a platoon
            if vehicle.platoon_role != PlatoonRole.NONE:
                LOG.debug(f"{vehicle.vid} is already in a platoon")
                continue
            # filter vehicles which are already in a maneuver
            if vehicle.in_maneuver:
                LOG.debug(f"{vehicle.vid} is already in a maneuver")
                continue

            # allow a vehicle to be assigned to exactly one platoon
            constraint_one_target_platoon = solver.RowConstraint(1, 1, f"one platoon: {vehicle.vid}")

            # get all available platoons or platoon candidates
            for other_vehicle in self._owner._simulator._vehicles.values():
                # filter vehicles that are technically not able to do platooning
                if not isinstance(other_vehicle, PlatooningVehicle):
                    LOG.debug(f"{other_vehicle.vid} is not capable of platooning")
                    continue

                # here, the logic similar to the distributed approach begins

                # filter vehicles which are not available to become a new leader
                # we only have this information due to oracle knowledge in the centralized version
                # we use this to replace management of neighbors and their advertisements
                if other_vehicle.platoon_role != PlatoonRole.NONE and other_vehicle.platoon_role != PlatoonRole.LEADER:
                    # we can only join an existing platoon or built a new one
                    LOG.debug(f"{other_vehicle.vid} is not available")
                    vehicle._candidates_filtered += 1
                    vehicle._candidates_filtered_follower += 1
                    continue

                # filter vehicles which are already in a maneuver
                # we only have this information due to oracle knowledge in the centralized version
                if other_vehicle.in_maneuver:
                    LOG.debug(f"{other_vehicle.vid} is already in a maneuver")
                    vehicle._candidates_filtered += 1
                    vehicle._candidates_filtered_maneuver += 1
                    continue

                platoon = other_vehicle.platoon

                # for one vehicle A we are looking at a different vehicle B to
                # check whether it is useful that A joins B

                # we assume driving alone is worse than to do platooning
                if platoon is vehicle.platoon:
                    fx = individual
                    LOG.debug(f"Considering driving individually for vehicle {vehicle.vid}")
                elif vehicle.position > platoon.rear_position:
                    # TODO HACK for skipping vehicles in front of us
                    LOG.debug(f"{vehicle.vid}'s platoon {platoon.platoon_id} not applicable because of its absolute position")
                    fx = infinity
                else:
                    # calculate deviation values
                    ds = self._ds(vehicle, platoon)
                    dp = self._dp(vehicle, platoon)

                    # remove platoon if not in speed range
                    if ds > self._speed_deviation_threshold * vehicle.desired_speed:
                        LOG.debug(f"{vehicle.vid}'s platoon {platoon.platoon_id} not applicable because of its speed difference ({ds})")
                        fx = infinity
                    # remove platoon if not in position range
                    elif dp > self._position_deviation_threshold:
                        LOG.debug(f"{vehicle.vid}'s platoon {platoon.platoon_id} not applicable because of its position difference ({dp})")
                        fx = infinity
                    else:
                        # calculate deviation/cost
                        LOG.debug(f"Considering platoon {platoon.platoon_id} for vehicle {vehicle.vid}")
                        fx = self._cost_speed_position(ds, dp, self._alpha, 1 - self._alpha)
                        vehicle._candidates_found += 1

                # define (0,1) decision variable for assignment of vehicle to platoon
                variable = solver.IntVar(0, 1, f"{vehicle.vid} -> {platoon.platoon_id}")

                # add variable to assignment matrix
                decision_variables[variable.index()] = {'vid': vehicle.vid, 'pid': platoon.platoon_id, 'lid': platoon.leader.vid, 'cost': fx}

                # add decision variable from vehicle to platoon to row sum
                constraint_one_target_platoon.SetCoefficient(variable, 1)

                # add decision variable from vehicle to platoon with corresponding cost to the row sum
                objective.SetCoefficient(variable, fx)

        # run the solver to calculate the optimal assignments
        LOG.info(f"{self._owner.iid} is running the solver for {solver.NumConstraints()} vehicles and {solver.NumVariables()} possible assignments")

        result_status = solver.Solve()
        self._assignments_solved += 1

        if result_status >= solver.INFEASIBLE:
            LOG.warning(f"{self._owner.iid}'s optimization problem was not solvable!")
            self._assigments_not_solveable += 1
            return

        if objective.Value() == 0:
            LOG.warning(f"{self._owner.iid} made no assignment!")
            self._assignments_none += 1
            return

        LOG.info(f"{self._owner.iid}'s optimal objective value is {objective.Value()}")
        LOG.info(f"{self._owner.iid}'s best bound is {objective.BestBound()}")
        LOG.info(f"{self._owner.iid} solved the optimization problem in {solver.wall_time()} ms")
        LOG.info(f"{self._owner.iid} solved the optimization problem in {solver.iterations()} iterations")
        LOG.info(f"{self._owner.iid} solved the optimization problem with {solver.nodes()} nodes")

        for variable in solver.variables():
            if variable.solution_value() > 0:
                mapping = decision_variables[variable.index()]
                LOG.debug(f"{mapping['vid']} was assigned to platoon {mapping['pid']} (leader {mapping['lid']}) with cost {mapping['cost']}")
                # HACK for oracle knowledge
                leader = self._owner._simulator._vehicles[mapping['lid']]
                vehicle = self._owner._simulator._vehicles[mapping['vid']]
                target_platoon = leader.platoon
                if vehicle.platoon.platoon_id == mapping['pid']:
                    # self-assignment
                    LOG.debug(f"{vehicle.vid} keeps driving individually")
                    self._assignments_self += 1
                    self._assignments_successful += 1
                    continue
                if target_platoon.platoon_id != mapping['pid']:
                    # meanwhile, the leader became a platoon member
                    assert(leader.is_in_platoon() and leader.platoon_role == PlatoonRole.FOLLOWER)
                    LOG.warning(f"{vehicle.vid}'s assigned platoon {mapping['pid']} (leader {leader.vid}) meanwhile joined another platoon {target_platoon.platoon_id}! {vehicle.vid} is joining this platoon transitively")
                    self._assignments_candidate_joined_already += 1
                else:
                    assert(not leader.is_in_platoon() or leader.platoon_role == PlatoonRole.LEADER)
                # let vehicle join platoon
                if vehicle.is_in_platoon():
                    # meanwhile, we became a platoon leader
                    assert(vehicle.platoon_role == PlatoonRole.LEADER)
                    LOG.warning(f"{vehicle.vid} meanwhile became the leader of platoon {vehicle.platoon.platoon_id}. Hence, no assignment is possible/necessary anymore")
                    self._assingments_vehicle_became_leader += 1
                    continue
                assert(not vehicle.in_maneuver)
                assert(not leader.in_maneuver)

                # actual join
                vehicle._join(target_platoon.platoon_id, target_platoon.leader.vid)
                self._assignments_successful += 1
