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
from timeit import default_timer as timer
from typing import TYPE_CHECKING

from ..formation_algorithm import FormationAlgorithm
from ..platoon_role import PlatoonRole
from ..statistics import record_infrastructure_assignments

if TYPE_CHECKING:
    from ..platooning_vehicle import Platoon  # noqa 401
    from ..platooning_vehicle import PlatooningVehicle  # noqa 401

LOG = logging.getLogger(__name__)
DEFAULTS = {
    'alpha': 0.5,
    'speed_deviation_threshold': -1,
    'position_deviation_threshold': 2000,
    'formation_centralized_kind': 'greedy',
    'solver_time_limit': 60 * 1000,  # s -> ms
}


class SpeedPosition(FormationAlgorithm):
    """
    Formation Algorithm based on:

    Julian Heinovski and Falko Dressler,
    "Platoon Formation: Optimized Car to Platoon Assignment Strategies and Protocols,"
    Proceedings of 10th IEEE Vehicular Networking Conference (VNC 2018), Taipei, Taiwan, December 2018.
    """

    def __init__(
        self,
        owner: object,
        alpha: float = DEFAULTS['alpha'],
        speed_deviation_threshold: float = DEFAULTS['speed_deviation_threshold'],
        position_deviation_threshold: int = DEFAULTS['position_deviation_threshold'],
        formation_centralized_kind: str = DEFAULTS['formation_centralized_kind'],
        solver_time_limit: int = DEFAULTS['solver_time_limit'],
        **kw_args,
    ):

        """
        Initializes an instance of this formation algorithm to be used in a vehicle or an infrastructure.

        Parameters
        ----------
        owner : object
            The owning object that is execution this algorithm.
            This can be either a PlatooningVehicle or an Infrastructure.
        alpha : float
            The weighting factor alpha
        speed_deviation_threshold : float
            The threshold for speed deviation
        position_deviation_threshold : float
            The threshold for position deviation
        """

        super().__init__(owner)

        if alpha < 0 or alpha > 1:
            sys.exit("ERROR: The weighting factor alpha needs to be in [0,1]!")
        self._alpha = alpha  # the weight of the speed deviation
        # the maximum deviation from the desired driving speed
        if speed_deviation_threshold == -1:
            self._speed_deviation_threshold = 1.0
        else:
            self._speed_deviation_threshold = speed_deviation_threshold
        # the maximum deviation from the current position
        if position_deviation_threshold == -1:
            self._position_deviation_threshold = self._owner._simulator.road_length  # FIXME
        else:
            self._position_deviation_threshold = position_deviation_threshold
        if solver_time_limit < 2 * 1000:
            LOG.warning("The time limit for the solver should be at least 2s! Otherwise it may not be possible for the solver to produce a solution (especially with many vehicles)!")
        self._formation_centralized_kind = formation_centralized_kind  # the kind of the centralized formation
        self._solver_time_limit = solver_time_limit  # the time limit for the optimal solver per assignment problem
        if formation_centralized_kind == "optimal":
            LOG.warning("NOTE: The optimal solver is still experimental!")

        # statistics
        self._assignments_solved = 0
        self._assignments_not_solvable = 0
        self._assignments_solved_optimal = 0
        self._assignments_solved_feasible = 0
        self._assignments_none = 0
        self._assignments_self = 0
        self._assignments_candidate_joined_already = 0
        self._assingments_vehicle_became_leader = 0
        self._assignments_successful = 0

    @classmethod
    def add_parser_argument_group(cls, parser: argparse.ArgumentParser):
        group = parser.add_argument_group(f"Formation Algorithm -- {cls.__name__}")
        group.add_argument(
            "--alpha",
            type=float,
            default=DEFAULTS['alpha'],
            help="The weight of the speed deviation in comparison to the position deviation",
        )
        group.add_argument(
            "--speed-deviation-threshold",
            type=float,
            default=DEFAULTS['speed_deviation_threshold'],
            help="The maximum allowed (relative) deviation from the desired speed for considering neighbors as candidates. A value of -1 disables the threshold",
        )
        group.add_argument(
            "--position-deviation-threshold",
            type=int,
            default=DEFAULTS['position_deviation_threshold'],
            help="The maximum allowed absolute deviation from the current position for considering neighbors as candidates. A value of -1 disables the threshold",
        )
        group.add_argument(
            "--formation-centralized-kind",
            type=str,
            default=DEFAULTS['formation_centralized_kind'],
            choices=["greedy", "optimal"],
            help="The kind of the centralized formation",
        )
        group.add_argument(
            "--solver-time-limit",
            type=int,
            default=int(DEFAULTS['solver_time_limit'] / 1000),  # ms -> s
            help="The time limit for the optimal solver per assignment problem in s. Influences the quality of the solution.",
        )

    def ds(self, vehicle: 'PlatooningVehicle', platoon: 'Platoon'):
        """
        Returns the deviation in speed from a given platoon.

        NOTE: In the original version of the paper, the deviation calculated here was not normalized.

        Parameters
        ----------
        vehicle : PlatooningVehicle
            The vehicle for which the deviation is calculated
        platoon : Platoon
            The platoon to which the deviation is calculated
        """

        diff = abs(vehicle._desired_speed - platoon.desired_speed)

        # normalize deviation by maximum allowed speed deviation
        return (diff / (self._speed_deviation_threshold * vehicle._desired_speed))

    def dp(self, vehicle: 'PlatooningVehicle', platoon: 'Platoon'):
        """
        Returns the deviation in position from a given platoon.

        NOTE: In the original version of the paper, the deviation calculated here was not normalized.

        Parameters
        ----------
        vehicle : PlatooningVehicle
            The vehicle for which the deviation is calculated
        platoon : Platoon
            The platoon to which the deviation is calculated
        """

        if vehicle.rear_position > platoon.position:
            # we are in front of the platoon
            diff = abs(vehicle.rear_position - platoon.position)
        else:
            # we are behind the platoon
            diff = abs(platoon.rear_position - vehicle.position)

        # normalize deviation by maximum allowed position deviation
        return (diff / self._position_deviation_threshold)

    def cost_speed_position(self, ds: float, dp: int):
        """
        Returns the overall cost (i.e., the weighted deviation) for a candidate.

        Parameters
        ----------
        ds : float
            The deviation in speed
        dp : int
            The deviation in position
        """

        return (self._alpha * ds) + ((1.0 - self._alpha) * dp)

    def do_formation(self):
        """
        Runs platoon formation algorithms to search for a platooning opportunity
        and performs the corresponding join maneuver.
        """

        from ..infrastructure import Infrastructure
        if isinstance(self._owner, Infrastructure):
            LOG.info(f"{self._owner.iid} is running formation algorithm {self.name} ({self._formation_centralized_kind}) at {self._owner._simulator.step}")
            if self._formation_centralized_kind == 'optimal':
                # optimal
                self._do_formation_optimal()  # still experimental
            else:
                # greedy
                self._do_formation_centralized()
        else:
            # greedy
            self._do_formation_distributed()

    def finish(self):
        """
        Cleans up the instance of the formation algorithm.

        This includes mostly statistic recording.
        """

        # write statistics
        if not self._owner._simulator._record_platoon_formation:
            return
        if self._formation_centralized_kind != 'optimal':
            return

        from ..infrastructure import Infrastructure
        assert isinstance(self._owner, Infrastructure)

        if self._owner._simulator._record_infrastructure_assignments:
            record_infrastructure_assignments(
                basename=self._owner._simulator._result_base_filename,
                iid=self._owner.iid,
                algorithm=self,
            )

    def _do_formation_distributed(self):
        """
        Runs distributed greedy formation approach.

        This selects a candidate and triggers a join maneuver.
        """

        # we can only run the algorithm if we are not yet in a platoon
        # because this algorithm does not support changing the platoon later on
        if self._owner.platoon_role != PlatoonRole.NONE:
            LOG.trace(f"{self._owner.vid} is already in a platoon")
            return

        # only if not currently in a maneuver
        if self._owner.in_maneuver:
            LOG.trace(f"{self._owner.vid} is already in a maneuver")
            return

        LOG.debug(f"{self._owner.vid} is running formation algorithm {self.name} (distributed)")

        self._owner._formation_iterations += 1

        found_candidates = []

        # get all available platoons or platoon candidates
        for platoon in self._owner._get_available_platoons():

            # calculate deviation values
            ds = self.ds(self._owner, platoon)
            dp = self.dp(self._owner, platoon)

            # TODO HACK for skipping platoons behind us
            if self._owner.position > platoon.rear_position:
                LOG.trace(f"{self._owner.vid}'s platoon {platoon.platoon_id} not applicable because of its absolute position")
                continue

            # remove platoon if not in speed range
            if ds > 1.0:
                LOG.trace(f"{self._owner.vid}'s platoon {platoon.platoon_id} not applicable because of its speed difference")
                continue

            # remove platoon if not in position range
            if dp > 1.0:
                LOG.trace(f"{self._owner.vid}'s platoon {platoon.platoon_id} not applicable because of its position difference")
                continue

            # calculate deviation/cost
            fx = self.cost_speed_position(ds, dp)

            # add platoon to list
            found_candidates.append({'vid': self._owner.vid, 'pid': platoon.platoon_id, 'lid': platoon.leader.vid, 'cost': fx})

        # the number of candidates found in this iteration
        LOG.debug(f"{self._owner.vid} found {len(found_candidates)} applicable candidates")
        self._owner._candidates_found += len(found_candidates)

        if len(found_candidates) == 0:
            LOG.debug(f"{self._owner.vid} has no candidates")
            return

        # find best candidate to join
        # pick the platoon with the lowest deviation
        best = min(found_candidates, key=lambda x: x['cost'])
        LOG.debug(f"{self._owner.vid}'s best platoon is {best['pid']} (leader {best['lid']}) with {best['cost']}")

        # perform a join maneuver with the candidate's platoon
        # do we really want the candidate to advertise its platoon
        # or do we just want the leader to advertise its platoon?
        self._owner._join(best['pid'], best['lid'])

    def _do_formation_centralized(self):
        """
        Runs centralized greedy formation approach.

        This selects candidates and triggers join maneuvers.
        """

        all_found_candidates = []

        from ..platooning_vehicle import PlatooningVehicle  # noqa 811

        # select all searching vehicles
        for vehicle in self._owner._simulator._vehicles.values():
            # filter vehicles that are technically not able to do platooning
            if not isinstance(vehicle, PlatooningVehicle):
                LOG.trace(f"{vehicle.vid} is not capable of platooning")
                continue
            # filter vehicles which are already in a platoon
            if vehicle.platoon_role != PlatoonRole.NONE:
                LOG.trace(f"{vehicle.vid} is already in a platoon")
                continue
            # filter vehicles which are already in a maneuver
            if vehicle.in_maneuver:
                LOG.trace(f"{vehicle.vid} is already in a maneuver")
                continue

            vehicle._formation_iterations += 1

            # get all available platoons or platoon candidates
            for other_vehicle in self._owner._simulator._vehicles.values():
                # filter same car because we assume driving alone is worse than to do platooning
                if other_vehicle is vehicle:
                    continue
                # filter vehicles that are technically not able to do platooning
                if not isinstance(other_vehicle, PlatooningVehicle):
                    LOG.trace(f"{other_vehicle.vid} is not capable of platooning")
                    continue

                # here, the logic similar to the distributed approach begins

                # filter vehicles which are not available to become a new leader
                # we only have this information due to oracle knowledge in the centralized version
                # we use this to replace management of neighbors and their advertisements
                if other_vehicle.platoon_role != PlatoonRole.NONE and other_vehicle.platoon_role != PlatoonRole.LEADER:
                    # we can only join an existing platoon or built a new one
                    LOG.trace(f"{other_vehicle.vid} is not available")
                    vehicle._candidates_filtered += 1
                    vehicle._candidates_filtered_follower += 1
                    continue

                # filter vehicles which are already in a maneuver
                # we only have this information due to oracle knowledge in the centralized version
                if other_vehicle.in_maneuver:
                    LOG.trace(f"{other_vehicle.vid} is already in a maneuver")
                    vehicle._candidates_filtered += 1
                    vehicle._candidates_filtered_maneuver += 1
                    continue

                platoon = other_vehicle.platoon

                # for one vehicle A we are looking at a different vehicle B to
                # check whether it is useful that A joins B

                # calculate deviation values
                ds = self.ds(vehicle, platoon)
                dp = self.dp(vehicle, platoon)

                # TODO HACK for skipping platoons behind us
                if vehicle.position > platoon.rear_position:
                    LOG.trace(f"{vehicle.vid}'s platoon {platoon.platoon_id} not applicable because of its absolute position")
                    continue

                # remove platoon if not in speed range
                if ds > 1.0:
                    LOG.trace(f"{vehicle.vid}'s platoon {platoon.platoon_id} not applicable because of its speed difference")
                    continue

                # remove platoon if not in position range
                if dp > 1.0:
                    LOG.trace(f"{vehicle.vid}'s platoon {platoon.platoon_id} not applicable because of its position difference")
                    continue

                # calculate deviation/cost
                fx = self.cost_speed_position(ds, dp)

                # add platoon to list
                all_found_candidates.append({'vid': vehicle.vid, 'pid': platoon.platoon_id, 'lid': platoon.leader.vid, 'cost': fx})
                LOG.debug(f"{vehicle.vid} found applicable candidate {platoon.platoon_id}")
                vehicle._candidates_found += 1

            # end vehicle

        # end all vehicles

        if len(all_found_candidates) == 0:
            LOG.debug(f"{self._owner.iid} found no possible matches")
            return

        # get unique list of searching vehicles from within the possible matches
        uids = set(x['vid'] for x in all_found_candidates)

        for v in uids:
            # get vehicle data and candidates
            vehicle = self._owner._simulator._vehicles[v]
            found_candidates = [x for x in all_found_candidates if x['vid'] == v]

            if len(found_candidates) == 0:
                # this vehicle has no candidates (anymore)
                LOG.trace(f"{vehicle.vid} has no candidates (anymore)")
                continue

            # find best candidate to join
            # pick the platoon with the lowest deviation
            best = min(found_candidates, key=lambda x: x['cost'])
            LOG.debug(f"{v}'s best platoon is {best['pid']} (leader {best['lid']}) with {best['cost']}")

            # perform a join maneuver with the candidate's platoon
            # do we really want the candidate to advertise its platoon
            # or do we just want the leader to advertise its platoon?
            vehicle._join(best['pid'], best['lid'])

            # remove all matches from the list of possible matches that would include the selected vehicle
            def is_available(x: dict) -> bool:
                """
                Returns whether an entry from the list of possible matches is (still) available.

                Parameters
                ----------
                x : dict
                    The entry from the list of possible matches
                """

                return (
                    x['vid'] != best['vid'] and  # noqa 504 # this vehicle does not search anymore
                    x['lid'] != best['vid'] and  # noqa 504 # this vehicle will not be applicable as leader anymore
                    x['vid'] != best['lid'] and  # noqa 504 # the other vehicle is not searching anymore, since it will become a leader
                    x['lid'] != best['lid']  # the leader is not applicable as leader anymore
                )

            all_found_candidates = [x for x in all_found_candidates if is_available(x)]

    def _do_formation_optimal(self):
        """
        Run centralized optimal formation approach.

        This selects candidates and triggers join maneuvers.

        NOTE: This functionality is still experimental!
        """

        from ortools.linear_solver import pywraplp
        solver = pywraplp.Solver(f"{self.name} solver", pywraplp.Solver.CBC_MIXED_INTEGER_PROGRAMMING)
        solver.SetNumThreads(1)
        # influences the quality of the solution
        solver.set_time_limit(self._solver_time_limit)

        # import sys
        # infinity = sys.float_info.max  # does work
        # infinity = solver.infinity() does work?
        individual = 1e+6  # big magic number for the cost of driving individually

        objective = solver.Objective()
        objective.SetMinimization()

        decision_variables = {}

        from ..platooning_vehicle import PlatooningVehicle  # noqa 811

        LOG.debug(f"{self._owner.iid} is adding assignment variables for vehicles")

        # select all searching vehicles
        for vehicle in self._owner._simulator._vehicles.values():

            # filter vehicles that are technically not able to do platooning
            if not isinstance(vehicle, PlatooningVehicle):
                LOG.trace(f"{vehicle.vid} is not capable of platooning")
                continue
            # filter vehicles which are already in a platoon
            if vehicle.platoon_role != PlatoonRole.NONE:
                LOG.trace(f"{vehicle.vid} is already in a platoon")
                continue
            # filter vehicles which are already in a maneuver
            if vehicle.in_maneuver:
                LOG.trace(f"{vehicle.vid} is already in a maneuver")
                continue

            vehicle._formation_iterations += 1

            # allow a vehicle to be assigned to exactly one platoon
            constraint_one_target_platoon = solver.RowConstraint(1, 1, f"one platoon: {vehicle.vid}")

            # get all available platoons or platoon candidates
            for other_vehicle in self._owner._simulator._vehicles.values():
                # filter vehicles that are technically not able to do platooning
                if not isinstance(other_vehicle, PlatooningVehicle):
                    LOG.trace(f"{other_vehicle.vid} is not capable of platooning")
                    continue

                # here, the logic similar to the distributed approach begins

                # filter vehicles which are not available to become a new leader
                # we only have this information due to oracle knowledge in the centralized version
                # we use this to replace management of neighbors and their advertisements
                if other_vehicle.platoon_role != PlatoonRole.NONE and other_vehicle.platoon_role != PlatoonRole.LEADER:
                    # we can only join an existing platoon or built a new one
                    LOG.trace(f"{other_vehicle.vid} is not available")
                    vehicle._candidates_filtered += 1
                    vehicle._candidates_filtered_follower += 1
                    continue

                # filter vehicles which are already in a maneuver
                # we only have this information due to oracle knowledge in the centralized version
                if other_vehicle.in_maneuver:
                    LOG.trace(f"{other_vehicle.vid} is already in a maneuver")
                    vehicle._candidates_filtered += 1
                    vehicle._candidates_filtered_maneuver += 1
                    continue

                platoon = other_vehicle.platoon

                # for one vehicle A we are looking at a different vehicle B to
                # check whether it is useful that A joins B

                # we assume driving alone is worse than to do platooning
                if platoon is vehicle.platoon:
                    fx = individual
                    LOG.trace(f"Considering driving individually for vehicle {vehicle.vid}")
                elif vehicle.position > platoon.rear_position:
                    # TODO HACK for skipping platoons behind us
                    LOG.trace(f"{vehicle.vid}'s platoon {platoon.platoon_id} not applicable because of its absolute position")
                    continue
                else:
                    # calculate deviation values
                    ds = self.ds(vehicle, platoon)
                    dp = self.dp(vehicle, platoon)

                    # remove platoon if not in speed range
                    if ds > 1.0:
                        LOG.trace(f"{vehicle.vid}'s platoon {platoon.platoon_id} not applicable because of its speed difference ({ds})")
                        continue
                    # remove platoon if not in position range
                    elif dp > 1.0:
                        LOG.trace(f"{vehicle.vid}'s platoon {platoon.platoon_id} not applicable because of its position difference ({dp})")
                        continue
                    else:
                        # calculate deviation/cost
                        LOG.debug(f"Considering platoon {platoon.platoon_id} for vehicle {vehicle.vid}")
                        fx = self.cost_speed_position(ds, dp)
                        vehicle._candidates_found += 1

                # define (0,1) decision variable for assignment of vehicle to platoon
                variable = solver.IntVar(0, 1, f"{vehicle.vid} -> {platoon.platoon_id}")

                # add variable to assignment matrix
                decision_variables[variable.index()] = {
                    'vid': vehicle.vid,
                    'pid': platoon.platoon_id,
                    'lid': platoon.leader.vid,
                    'cost': fx,
                    'var': variable.index(),
                }

                # add decision variable from vehicle to platoon to row sum
                constraint_one_target_platoon.SetCoefficient(variable, 1)

                # add decision variable from vehicle to platoon with corresponding cost to the row sum
                objective.SetCoefficient(variable, fx)

            # end vehicle

        # end all vehicles

        # add more platoon constraints
        LOG.debug(f"{self._owner.iid} is adding additional platoon constraints")
        # get all platoons
        for pid in set([x['pid'] for x in decision_variables.values()]):
            # create constraint for this platoon
            # assign only one (other) vehicle to this platoon
            constraint_one_member_per_platoon = solver.RowConstraint(0, 1, f"one other member per platoon: {pid}")
            # assign a vehicle only if no other vehicles has been assigned
            constraint_one_assignment_per_vehicle = solver.RowConstraint(0, 1, f"one assignment per vehicle: {pid}")
            # get all variables that assign a vehicle to that platoon
            for var in [x['var'] for x in decision_variables.values() if x['pid'] == pid and x['vid'] != pid]:
                # not a self-assignment
                variable = solver.variable(var)
                constraint_one_member_per_platoon.SetCoefficient(variable, 1)
                constraint_one_assignment_per_vehicle.SetCoefficient(variable, 1)

            # get all variables that assign this vehicle to someone else
            for var in [x['var'] for x in decision_variables.values() if x['vid'] == pid and x['pid'] != pid]:
                # not a self-assignment
                constraint_one_assignment_per_vehicle.SetCoefficient(solver.variable(var), 1)

        if solver.NumConstraints() == 0:
            LOG.info(f"{self._owner.iid} has no vehicles to run the solver for")
            return

        # run the solver to calculate the optimal assignments
        LOG.info(f"{self._owner.iid} is running the solver for {solver.NumVariables()} possible assignments, and {solver.NumConstraints()} constraints")

        start_time = timer()
        result_status = solver.Solve()
        end_time = timer()
        run_time = end_time - start_time

        if result_status == solver.OPTIMAL:
            LOG.info(f"{self._owner.iid}'s solution is optimal")
            self._assignments_solved_optimal += 1
        elif result_status == solver.FEASIBLE:
            LOG.info(f"{self._owner.iid}'s solution is not optimal")
            self._assignments_solved_feasible += 1
        elif result_status >= solver.INFEASIBLE:
            LOG.warning(f"{self._owner.iid}'s optimization problem was not solvable!")
            self._assignments_not_solvable += 1
            return

        self._assignments_solved += 1

        if objective.Value() == 0:
            LOG.info(f"{self._owner.iid} made no assignment!")
            self._assignments_none += 1
            return

        # TODO record mean runtime?

        LOG.info(f"{self._owner.iid} solved the optimization problem in {run_time}s ({solver.wall_time()}ms)")
        LOG.info(f"{self._owner.iid} solved the optimization problem in {solver.iterations()} iterations")  # broken?
        LOG.debug(f"{self._owner.iid}'s optimal objective value is {objective.Value()}")
        LOG.debug(f"{self._owner.iid}'s best bound is {objective.BestBound()}")

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
                    assert (leader.is_in_platoon() and leader.platoon_role == PlatoonRole.FOLLOWER)
                    LOG.warning(f"{vehicle.vid}'s assigned platoon {mapping['pid']} (leader {leader.vid}) meanwhile joined another platoon {target_platoon.platoon_id}!")
                    self._assignments_candidate_joined_already += 1
                    continue
                else:
                    assert not leader.in_maneuver
                    assert (not leader.is_in_platoon() or leader.platoon_role == PlatoonRole.LEADER)
                # let vehicle join platoon
                if vehicle.is_in_platoon():
                    # meanwhile, we became a platoon leader
                    assert vehicle.platoon_role == PlatoonRole.LEADER
                    LOG.warning(f"{vehicle.vid} meanwhile became the leader of platoon {vehicle.platoon.platoon_id}. Hence, no assignment is possible/necessary anymore")
                    self._assingments_vehicle_became_leader += 1
                    continue
                assert not vehicle.in_maneuver
                assert not leader.in_maneuver

                # actual join
                vehicle._join(target_platoon.platoon_id, target_platoon.leader.vid)
                self._assignments_successful += 1
