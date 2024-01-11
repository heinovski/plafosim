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
import sys
from distutils.util import strtobool
from timeit import default_timer as timer
from typing import TYPE_CHECKING

from ..formation_algorithm import FormationAlgorithm
from ..platoon_role import PlatoonRole

if TYPE_CHECKING:
    from ..platooning_vehicle import Platoon  # noqa 401
    from ..platooning_vehicle import PlatooningVehicle  # noqa 401

LOG = logging.getLogger(__name__)

# default values for this algorithm's parameters
DEFAULTS = {
    'alpha': 0.5,
    'speed_deviation_threshold': 0.2,
    'position_deviation_threshold': 1000,  # m
    'formation_centralized_kind': 'greedy',
    'solver_time_limit': 60,  # s
    'record_solver_traces': False,
    'record_infrastructure_assignments': False,
}


def initialize_solver_traces(basename: str):
    """
    Initialize the solver trace file.

    Parameters
    ----------
    basename : str
        The basename of the trace file
    """

    assert basename
    with open(f'{basename}_solver_traces.csv', 'w') as f:
        f.write(
            "step,"
            "id,"
            "numVariables,"
            "numConstraints,"
            "runTime,"
            "resultStatus,"
            "solutionValue,"
            "bestBound,"
            "solutionQuality"
            "\n"
        )


def record_solver_trace(
    basename: str,
    step: float,
    iid: int,
    variables: int,
    constraints: int,
    run_time: float,
    result_status: int,
    solution_value: float,
    best_bound: float,
    solution_quality: float,
):
    """
    Record one line in the solver trace file.

    Parameters
    ----------
    basename : str
        The basename of the trace file
    step : float
        The current simulation step
    iid : int
        The id of the infrastructure which executed the solver run
    variables : int
        The number of variables
    constraints : int
        The number of constraints
    run_time : float
        The run time of the solver
    result_status : int
        The result status of the solver
    solution_value : float
        The solution value of the solver
    best_bound : float
        The best bound of the problem
    solution_quality : float
        The quality of the solution
    """

    assert basename
    with open(f'{basename}_solver_traces.csv', 'a') as f:
        f.write(
            f"{step},"
            f"{iid},"
            f"{variables},"
            f"{constraints},"
            f"{run_time},"
            f"{result_status},"
            f"{solution_value},"
            f"{best_bound},"
            f"{solution_quality}"
            "\n"
        )


def initialize_infrastructure_assignments(basename: str):
    """
    Initialize the infrastructure assignments result file.

    Parameters
    ----------
    basename : str
        The basename of the result file
    """

    assert basename
    with open(f'{basename}_infrastructure_assignments.csv', 'w') as f:
        f.write(
            "id,"
            "assignmentsSolved,"
            "assignmentsNotSolved,"
            "assignmentsSolvedOptimal,"
            "assignmentsSolvedFeasible,"
            "assignmentsNone,"
            "assignmentsSelf,"
            "assignmentsCandidateJoinedAlready,"
            "assignmentsVehicleBecameLeader,"
            "assignmentsSuccessful"
            "\n"
        )


class SpeedPosition(FormationAlgorithm):
    """
    Platoon Formation Algorithm based on Similarity, considering Speed and Position.

    See papers

    Julian Heinovski and Falko Dressler,
    "Where to Decide? Centralized vs. Distributed Vehicle Assignment for Platoon Formation,"
    arXiv, cs.MA, 2310.09580, October 2023.
    https://www.tkn.tu-berlin.de/bib/heinovski2023where-preprint/

    and

    Julian Heinovski and Falko Dressler,
    "Platoon Formation: Optimized Car to Platoon Assignment Strategies and Protocols,"
    Proceedings of 10th IEEE Vehicular Networking Conference (VNC 2018), Taipei, Taiwan, December 2018.
    https://www.tkn.tu-berlin.de/bib/heinovski2018platoon/
    """

    def __init__(
        self,
        owner: object,
        alpha: float = DEFAULTS['alpha'],
        speed_deviation_threshold: float = DEFAULTS['speed_deviation_threshold'],
        position_deviation_threshold: int = DEFAULTS['position_deviation_threshold'],
        formation_centralized_kind: str = DEFAULTS['formation_centralized_kind'],
        solver_time_limit: int = DEFAULTS['solver_time_limit'],
        record_solver_traces: bool = DEFAULTS['record_solver_traces'],
        record_infrastructure_assignments: bool = DEFAULTS['record_infrastructure_assignments'],
        **kw_args,
    ):
        """
        Initialize an instance of this formation algorithm to be used in a vehicle or an infrastructure.

        Parameters
        ----------
        owner : object
            The owning object that is execution this algorithm.
            This can be either a PlatooningVehicle or an Infrastructure.
        alpha : float
            The weighting factor alpha
        speed_deviation_threshold : float
            The threshold for speed deviation
        position_deviation_threshold : int
            The threshold for position deviation
        formation_centralized_kind : str
            TODO
        solver_time_limit : int
            The time limit in s to apply to the solver
        record_solver_traces : bool
            Whether to record continuous solver traces
        record_infrastructure_assignments : bool
            Whether to record infrastructure assignments

        """

        super().__init__(owner)

        if alpha < 0 or alpha > 1:
            sys.exit("ERROR [{__name__}]: The weighting factor alpha needs to be in [0,1]!")
        self._alpha = alpha  # the weight of the speed deviation
        # the maximum deviation from the desired driving speed
        if speed_deviation_threshold == -1:
            # use 100% speed deviation from own desired driving speed
            # e.g., des = 30 m/s, possible speed in [0, 60] m/s
            self._speed_deviation_threshold = 1.0
        else:
            self._speed_deviation_threshold = speed_deviation_threshold
        # the maximum deviation from the current position
        if position_deviation_threshold == -1:
            # use maximum possible distance aka total road length
            self._position_deviation_threshold = self._owner._simulator.road_length
        else:
            self._position_deviation_threshold = position_deviation_threshold
        self._formation_centralized_kind = formation_centralized_kind  # the kind of the centralized formation
        if formation_centralized_kind == "optimal":
            if solver_time_limit <= 0:
                LOG.warning("Running the solver without a time limit may lead to long simulation times!")
            elif solver_time_limit < 2:
                LOG.warning("The time limit for the solver should be at least 2s! Otherwise it may not be possible for the solver to produce a solution (especially with many vehicles)!")
        self._solver_time_limit = solver_time_limit  # the time limit for the optimal solver per assignment problem
        self._record_solver_traces = record_solver_traces  # whether to record continuous solver traces
        self._record_infrastructure_assignments = record_infrastructure_assignments  # whether to record infrastructure assignments

        # statistics
        self._assignments_solved = 0
        self._assignments_not_solved = 0
        self._assignments_solved_optimal = 0
        self._assignments_solved_feasible = 0
        self._assignments_none = 0
        self._assignments_self = 0
        self._assignments_candidate_joined_already = 0
        self._assingments_vehicle_became_leader = 0
        self._assignments_successful = 0

        from ..infrastructure import Infrastructure
        if isinstance(self._owner, Infrastructure) and self._formation_centralized_kind == 'optimal':
            if self._record_solver_traces:
                # create output file for solver traces
                initialize_solver_traces(basename=self._owner._simulator._result_base_filename)
            if self._record_infrastructure_assignments:
                # create output file for infrastructure assignments
                initialize_infrastructure_assignments(basename=self._owner._simulator._result_base_filename)

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
            "--alpha",
            type=float,
            default=DEFAULTS['alpha'],
            help="The weight of the speed deviation in comparison to the position deviation",
        )
        group.add_argument(
            "--speed-deviation-threshold",
            type=float,
            default=DEFAULTS['speed_deviation_threshold'],
            help="The maximum allowed relative deviation from the desired speed for considering neighbors as candidates. A value of -1 disables the threshold (i.e., 100%% desired speed)",
        )
        group.add_argument(
            "--position-deviation-threshold",
            type=int,
            default=DEFAULTS['position_deviation_threshold'],
            help="The maximum allowed absolute deviation from the current position for considering neighbors as candidates. A value of -1 disables the threshold (i.e., total road length)",
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
            default=int(DEFAULTS['solver_time_limit']),
            help="The time limit for the optimal solver per assignment problem in s. Influences the quality of the solution. A value below 1 disables the limit.",
        )
        group.add_argument(
            "--record-solver-traces",
            type=lambda x: bool(strtobool(x)),
            default=DEFAULTS['record_solver_traces'],
            choices=(True, False),
            help="Whether to record solver traces",
        )
        group.add_argument(
            "--record-infrastructure-assignments",
            type=lambda x: bool(strtobool(x)),
            default=DEFAULTS['record_infrastructure_assignments'],
            choices=(True, False),
            help="Whether to record infrastructure assignments",
        )
        return group

    def ds(self, vehicle: 'PlatooningVehicle', platoon: 'Platoon') -> float:
        """
        Return the deviation in speed from a given platoon.

        NOTE: In the original version of the paper, the deviation calculated here was not normalized.

        Parameters
        ----------
        vehicle : PlatooningVehicle
            The vehicle for which the deviation is calculated
        platoon : Platoon
            The platoon to which the deviation is calculated

        Returns
        -------
        float
            The relative deviation in speed
        """

        diff = abs(vehicle._desired_speed - platoon.desired_speed)

        # normalize deviation by maximum allowed speed deviation
        # a value in [0, 1] from maximum
        return (diff / (self._speed_deviation_threshold * vehicle._desired_speed))

    def dp(self, vehicle: 'PlatooningVehicle', platoon: 'Platoon') -> float:
        """
        Return the deviation in position from a given platoon.

        NOTE: In the original version of the paper, the deviation calculated here was not normalized.

        Parameters
        ----------
        vehicle : PlatooningVehicle
            The vehicle for which the deviation is calculated
        platoon : Platoon
            The platoon to which the deviation is calculated

        Returns
        -------
        float
            The relative deviation in position
        """

        # NOTE: we have multiple options
        # a) take the leader's position
        # b) take the last's position
        # c) take the platoon's middle position
        # d) take minimum from a) and b)
        diff = min(
            # TODO consider vehicles' length as well
            abs(vehicle.position - platoon.position),  # vehicle is in front of platoon
            abs(platoon.last.position - vehicle.position),  # platoon is in front of vehicle
        )

        # normalize deviation by maximum allowed position deviation
        # a value in [0, 1] from maximum
        return (diff / self._position_deviation_threshold)

    def cost_speed_position(self, ds: float, dp: float) -> float:
        """
        Return the overall cost (i.e., the weighted deviation) for a candidate.

        Parameters
        ----------
        ds : float
            The deviation in speed
        dp : int
            The deviation in position

        Returns
        -------
        float
            The weighted relative deviation
        """

        fx = (self._alpha * ds) + ((1.0 - self._alpha) * dp)
        assert 0.0 <= fx <= 1.0
        return fx

    def do_formation(self):
        """
        Run platoon formation algorithms to search for a platooning opportunity and perform the corresponding join maneuver.
        """

        from ..infrastructure import Infrastructure
        if isinstance(self._owner, Infrastructure):
            LOG.info(f"{self._owner.iid} is running formation algorithm {self.name} ({self._formation_centralized_kind}) at {self._owner._simulator.step}")
            if self._formation_centralized_kind == 'optimal':
                # optimal
                self._do_formation_optimal()
            else:
                # greedy
                self._do_formation_centralized()
        else:
            # greedy
            self._do_formation_distributed()

    def _record_infrastructure_assignments(self, basename: str):
        """
        Record infrastructure assignments.

        Parameters
        ----------
        basename : str
            The basename of the result file
        """

        assert basename
        with open(f'{basename}_infrastructure_assignments.csv', 'a') as f:
            f.write(
                f"{self._owner.iid},"
                f"{self._assignments_solved},"
                f"{self._assignments_not_solved},"
                f"{self._assignments_solved_optimal},"
                f"{self._assignments_solved_feasible},"
                f"{self._assignments_none},"
                f"{self._assignments_self},"
                f"{self._assignments_candidate_joined_already},"
                f"{self._assingments_vehicle_became_leader},"
                f"{self._assignments_successful}"
                "\n"
            )

    def finish(self):
        """
        Clean up the instance of the formation algorithm.

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
            self._record_infrastructure_assignments(self._owner._simulator._result_base_filename)

    def _do_formation_distributed(self):
        """
        Run distributed greedy formation approach.

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

            # FIXME HACK for skipping platoons behind us
            if self._owner.position > platoon.rear_position:
                LOG.trace(f"{self._owner.vid}'s platoon {platoon.platoon_id} (leader {platoon.leader.vid}) not applicable because of its absolute position")
                # we do not record stats here since this is different to the communication aspect filtering
                continue

            # remove platoon if not in speed range
            if ds > 1.0:
                LOG.trace(f"{self._owner.vid}'s platoon {platoon.platoon_id} (leader {platoon.leader.vid}) not applicable because of its speed difference")
                continue

            # remove platoon if not in position range
            if dp > 1.0:
                LOG.trace(f"{self._owner.vid}'s platoon {platoon.platoon_id} (leader {platoon.leader.vid}) not applicable because of its position difference")
                continue

            # calculate deviation/cost
            fx = self.cost_speed_position(ds, dp)

            # add platoon to list
            found_candidates.append({'vid': self._owner.vid, 'pid': platoon.platoon_id, 'lid': platoon.leader.vid, 'cost': fx})

            # stats
            if platoon.size > 1:
                self._owner._candidates_found_platoon += 1
            else:
                self._owner._candidates_found_individual += 1

        # the number of candidates found in this iteration
        LOG.debug(f"{self._owner.vid} found {len(found_candidates)} applicable candidates")
        self._owner._candidates_found += len(found_candidates)

        if not found_candidates:
            LOG.debug(f"{self._owner.vid} has no candidates")
            return

        # find best candidate to join
        # pick the platoon with the lowest deviation
        best = min(found_candidates, key=lambda x: x['cost'])
        LOG.debug(f"{self._owner.vid}'s best platoon is {best['pid']} (leader {best['lid']}) with {best['cost']}")

        # perform a join maneuver with the candidate's platoon
        self._owner._join(best['pid'], best['lid'])

    def _do_formation_centralized(self):
        """
        Run centralized greedy formation approach.

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

                # FIXME HACK for skipping platoons behind us
                if vehicle.position > platoon.rear_position:
                    LOG.trace(f"{vehicle.vid}'s platoon {platoon.platoon_id} (leader {platoon.leader.vid}) not applicable because of its absolute position")
                    # we do not record stats here since this is different to the communication aspect filtering
                    continue

                # remove platoon if not in speed range
                if ds > 1.0:
                    # simply applying the threshold
                    LOG.trace(f"{vehicle.vid}'s platoon {platoon.platoon_id} (leader {platoon.leader.vid}) not applicable because of its speed difference")
                    continue

                # remove platoon if not in position range
                if dp > 1.0:
                    # simply applying the threshold
                    LOG.trace(f"{vehicle.vid}'s platoon {platoon.platoon_id} (leader {platoon.leader.vid}) not applicable because of its position difference")
                    continue

                # calculate deviation/cost
                fx = self.cost_speed_position(ds, dp)

                # NOTE: The distinction of vid and lid is superfluous, since we can only join either individual vehicles or platoon leaders.
                # More specifically, only individual vehicles or platoon leader are able to "advertise" their platoon (see above)
                assert other_vehicle.vid == platoon.leader.vid, f"We can only join individual vehicles or platoon leaders! {other_vehicle.vid}, {platoon.platoon_id}, {platoon.leader.vid}"
                # add platoon to list
                all_found_candidates.append({'vid': vehicle.vid, 'pid': platoon.platoon_id, 'lid': platoon.leader.vid, 'cost': fx})
                LOG.trace(f"{vehicle.vid} found applicable platoon {platoon.platoon_id} (leader {platoon.leader.vid})")

                # stats
                vehicle._candidates_found += 1
                if platoon.size > 1:
                    vehicle._candidates_found_platoon += 1
                else:
                    vehicle._candidates_found_individual += 1

            # end vehicle

        # end all vehicles

        if len(all_found_candidates) == 0:
            LOG.debug(f"{self._owner.iid} found no possible matches")
            return

        # get unique list of searching vehicles from within the possible matches
        uids = set(x['vid'] for x in all_found_candidates)

        # TODO apply random to uids?
        # go through all searching vehicles (i.e., their ids)
        for v in uids:
            # get vehicle data and candidates
            vehicle = self._owner._simulator._vehicles[v]
            # get all candidates that vehicle could join
            found_candidates = [x for x in all_found_candidates if x['vid'] == v]

            if len(found_candidates) == 0:
                # this vehicle has no candidates (anymore)
                LOG.trace(f"{vehicle.vid} has no candidates (anymore)")
                continue

            # find best candidate to join
            # pick the platoon with the lowest deviation
            best = min(found_candidates, key=lambda x: x['cost'])
            LOG.trace(f"{v}'s best platoon is {best['pid']} (leader {best['lid']}) with cost {best['cost']}")

            # perform a join maneuver with the candidate's platoon
            vehicle._join(best['pid'], best['lid'])

            # remove all matches from the list of possible matches that would include the selected vehicle
            # this is exactly avoiding using candidates that became followers meanwhile or are at least within a maneuver
            def is_available(x: dict) -> bool:
                """
                Return whether an entry from the list of possible matches is (still) available.

                Parameters
                ----------
                x : dict
                    The entry from the list of possible matches
                """

                return (
                    x['vid'] != best['vid'] and  # noqa 504 # this vehicle does not search anymore, since it just started a join maneuver
                    x['lid'] != best['vid'] and  # noqa 504 # this vehicle will not be applicable as leader anymore, since it just started a join maneuver
                    x['vid'] != best['lid'] and  # noqa 504 # the other vehicle is not searching anymore, since it will become a leader in the current join maneuver
                    x['lid'] != best['lid']  # the other vehicle is not available as leader anymore, since it is busy with the current join maneuver
                )

            all_found_candidates = [x for x in all_found_candidates if is_available(x)]

    def _do_formation_optimal(self):
        """
        Run centralized optimal formation approach.

        This selects candidates and triggers join maneuvers.
        """

        from ortools import __version__ as solver_version
        from ortools.linear_solver import pywraplp
        solver = pywraplp.Solver(f"{self.name} solver (version {solver_version}) @ {self._owner.iid}", pywraplp.Solver.SCIP_MIXED_INTEGER_PROGRAMMING)
        solver.SetNumThreads(1)
        if self._solver_time_limit > 0:
            # influences the quality of the solution
            solver.set_time_limit(self._solver_time_limit * 1000)  # s --> ms

        # import sys
        # infinity = sys.float_info.max  # does work
        # infinity = solver.infinity() does work?
        individual = 1  # big magic number for the cost of driving individually

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
            constraint_one_target_platoon = solver.RowConstraint(1, 1, f"only one platoon for {vehicle.vid}")

            # get all available platoons or platoon candidates
            for other_vehicle in self._owner._simulator._vehicles.values():
                # NOTE: we do not filter same car because staying individual is possible here

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
                    LOG.trace(f"Considering driving individually for vehicle {vehicle.vid} (with cost {fx})")
                elif vehicle.position > platoon.rear_position:
                    # FIXME HACK for skipping platoons behind us
                    # we do not record stats here since this is different to the communication aspect filtering
                    LOG.trace(f"{vehicle.vid}'s platoon {platoon.platoon_id} (leader {platoon.leader.vid}) not applicable because of its absolute position")
                    continue
                else:
                    # calculate deviation values
                    ds = self.ds(vehicle, platoon)
                    dp = self.dp(vehicle, platoon)

                    # remove platoon if not in speed range
                    if ds > 1.0:
                        # simply applying the threshold
                        LOG.trace(f"{vehicle.vid}'s platoon {platoon.platoon_id} (leader {platoon.leader.vid}) not applicable because of its speed difference ({ds})")
                        continue

                    # remove platoon if not in position range
                    elif dp > 1.0:
                        # simply applying the threshold
                        LOG.trace(f"{vehicle.vid}'s platoon {platoon.platoon_id} (leader {platoon.leader.vid}) not applicable because of its position difference ({dp})")
                        continue

                    else:
                        # calculate deviation/cost
                        fx = self.cost_speed_position(ds, dp)
                        LOG.trace(f"Considering platoon {platoon.platoon_id} (leader {platoon.leader.vid}) for vehicle {vehicle.vid} with cost {fx}")

                        # stats
                        vehicle._candidates_found += 1
                        if platoon.size > 1:
                            vehicle._candidates_found_platoon += 1
                        else:
                            vehicle._candidates_found_individual += 1

                # NOTE: The distinction of vid and lid is superfluous, since we can only join either individual vehicles or platoon leaders.
                # More specifically, only individual vehicles or platoon leader are able to "advertise" their platoon (see above)
                assert other_vehicle.vid == platoon.leader.vid, f"We can only join individual vehicles or platoon leaders! {other_vehicle.vid}, {platoon.platoon_id}, {platoon.leader.vid}"

                # define (0,1) decision variable for assignment of vehicle to platoon
                variable = solver.IntVar(0, 1, f"{vehicle.vid} -> {platoon.platoon_id} ({platoon.leader.vid})")

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

        # print cost matrix
        if LOG.getEffectiveLevel() <= logging.DEBUG:
            print(' ', end=' ')
            for lid in set([x['lid'] for x in decision_variables.values()]):
                print(lid, ' ', end=' ')
            print('(lid)')
            for vid in set([x['vid'] for x in decision_variables.values()]):
                print(vid, end=' ')
                for lid in set([x['lid'] for x in decision_variables.values()]):
                    variables = [x['var'] for x in decision_variables.values() if x['vid'] == vid and x['lid'] == lid]
                    if variables:
                        print(round(decision_variables[solver.variable(variables[0]).index()]['cost'], 1), end=' ')
                    else:
                        print(' - ', end=' ')
                print()
            print('(vid)')

        # add more platoon constraints
        # NOTE: We need to take the leader here, since the platoon id does not necessarily match the leader id (e.g., when the leader left already)
        # Also, we want to compare the vehicle id to the leader id to make sure that only one assignment is done for this vehicle
        LOG.debug(f"{self._owner.iid} is adding additional platoon constraints")

        # get all platoons
        for lid in set([x['lid'] for x in decision_variables.values()]):
            # create constraint for this platoon
            # assign only one (other) vehicle to this leader
            constraint_one_member_per_platoon = solver.RowConstraint(0, 1, f"one other member for leader {lid}")
            # assign a vehicle only if no other vehicles has been assigned to this vehicle
            constraint_one_assignment_per_vehicle = solver.RowConstraint(0, 1, f"one assignment per vehicle {lid}")
            # get all variables that assign a vehicle to this vehicle (as leader)
            for var in [x['var'] for x in decision_variables.values() if x['lid'] == lid and x['vid'] != lid]:
                # not a self-assignment
                variable = solver.variable(var)
                constraint_one_member_per_platoon.SetCoefficient(variable, 1)
                constraint_one_assignment_per_vehicle.SetCoefficient(variable, 1)

            # get all variables that assign this vehicle to someone else (as leader)
            for var in [x['var'] for x in decision_variables.values() if x['vid'] == lid and x['lid'] != lid]:
                # not a self-assignment
                constraint_one_assignment_per_vehicle.SetCoefficient(solver.variable(var), 1)

        if solver.NumConstraints() == 0:
            LOG.info(f"{self._owner.iid} has no vehicles to run the solver for")
            return

        # run the solver to calculate the optimal assignments
        LOG.info(f"{self._owner.iid} is running the solver for {solver.NumVariables()} possible assignments, and {solver.NumConstraints()} constraints")

        # solver debug output
        if LOG.getEffectiveLevel() <= logging.DEBUG:
            solver.EnableOutput()

        start_time = timer()
        result_status = solver.Solve()
        end_time = timer()
        run_time = end_time - start_time

        LOG.info(f"{self._owner.iid}'s solver ran for {run_time}s")
        LOG.info(f"{self._owner.iid}'s solver ran for {solver.iterations()} iterations")  # broken?
        solution_quality = objective.BestBound() / objective.Value()

        # record solver trace
        if self._record_solver_traces:
            record_solver_trace(
                basename=self._owner._simulator._result_base_filename,
                step=self._owner._simulator.step,
                iid=self._owner.iid,
                variables=solver.NumVariables(),
                constraints=solver.NumConstraints(),
                run_time=run_time,
                result_status=result_status,  # this is a simple int
                solution_value=objective.Value(),
                best_bound=objective.BestBound(),
                solution_quality=solution_quality,
            )

        # TODO record mean runtime

        if result_status == solver.OPTIMAL:  # 0
            LOG.info(f"{self._owner.iid}'s solution is optimal")
            self._assignments_solved += 1
            self._assignments_solved_optimal += 1
        elif result_status == solver.FEASIBLE:  # 1
            # from or-tools' documentation:
            # The solver had enough time to find some solution that satisfies all
            # constraints, but it did not prove optimality (which means it may or may
            # not have reached the optimal).
            # This can happen for large LP models (Linear Programming), and is a frequent
            # response for time-limited MIPs (Mixed Integer Programming). In the MIP
            # case, the difference between the solution 'objective_value' and
            # 'best_objective_bound' fields of the MPSolutionResponse will give an
            # indication of how far this solution is from the optimal one.
            LOG.info(f"{self._owner.iid}'s solution is not optimal")
            self._assignments_solved += 1
            self._assignments_solved_feasible += 1
            LOG.debug(f"{self._owner.iid}'s optimal objective value is {objective.Value()}")
            LOG.debug(f"{self._owner.iid}'s best bound is {objective.BestBound()}")
            LOG.info(f"{solution_quality} approximation of the optimal solution")
        elif result_status >= solver.INFEASIBLE:  # 2
            LOG.warning(f"{self._owner.iid}'s optimization problem was not solvable!")
            self._assignments_not_solved += 1
            return

        if objective.Value() == 0:
            LOG.info(f"{self._owner.iid} made no assignment!")
            self._assignments_none += 1
            return

        # print assingment matrix
        if LOG.getEffectiveLevel() <= logging.DEBUG:
            print(' ', end=' ')
            for lid in set([x['lid'] for x in decision_variables.values()]):
                print(lid, '', end=' ')
            print('(lid)')
            for vid in set([x['vid'] for x in decision_variables.values()]):
                print(vid, end=' ')
                for lid in set([x['lid'] for x in decision_variables.values()]):
                    variables = [x['var'] for x in decision_variables.values() if x['vid'] == vid and x['lid'] == lid]
                    if variables:
                        print(int(solver.variable(variables[0]).solution_value()), '', end=' ')
                    else:
                        print('- ', end=' ')
                print()
            print('(vid)')

        LOG.debug("Applying solver solution...")
        for variable in solver.variables():
            if variable.solution_value() > 0:
                mapping = decision_variables[variable.index()]
                LOG.trace(f"{mapping['vid']} was assigned to leader {mapping['lid']} (platoon {mapping['pid']}) with cost {mapping['cost']}")
                # get vehicle & platoon data
                # HACK for oracle knowledge
                leader = self._owner._simulator._vehicles[mapping['lid']]
                vehicle = self._owner._simulator._vehicles[mapping['vid']]
                target_platoon = leader.platoon
                if mapping['vid'] == mapping['lid']:
                    # self-assignment
                    assert mapping['cost'] == individual
                    LOG.trace(f"{vehicle.vid} keeps driving individually")
                    self._assignments_self += 1
                    self._assignments_successful += 1
                    continue
                if target_platoon.platoon_id != mapping['pid']:
                    # meanwhile, the leader became a platoon member (during application of the solver's solution)
                    # NOTE: this should never happen
                    assert (leader.is_in_platoon() and leader.platoon_role == PlatoonRole.FOLLOWER)
                    LOG.warning(f"{vehicle.vid}'s assigned leader {leader.vid} (platoon {mapping['pid']}) meanwhile joined another platoon {target_platoon.platoon_id}!")
                    self._assignments_candidate_joined_already += 1
                    sys.exit("ERROR [{__name__}]: This should never happen!")
                    continue
                else:
                    assert not leader.in_maneuver
                    assert (not leader.is_in_platoon() or leader.platoon_role == PlatoonRole.LEADER)
                # let vehicle join platoon
                if vehicle.is_in_platoon():
                    # meanwhile, we became a platoon leader (during application of the solver's solution)
                    # NOTE: this should never happen
                    assert vehicle.platoon_role == PlatoonRole.LEADER
                    LOG.warning(f"{vehicle.vid} meanwhile became the leader of platoon {vehicle.platoon.platoon_id}. Hence, no assignment is possible/necessary anymore")
                    self._assingments_vehicle_became_leader += 1
                    sys.exit("ERROR [{__name__}]: This should never happen!")
                    continue
                assert not vehicle.in_maneuver
                assert not leader.in_maneuver

                # actual join
                vehicle._join(target_platoon.platoon_id, target_platoon.leader.vid)
                self._assignments_successful += 1
