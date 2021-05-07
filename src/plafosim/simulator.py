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
import os
import random
import re
import sys
import time
from collections import namedtuple
from math import copysign

import pandas as pd
from tqdm import tqdm

from .infrastructure import Infrastructure
from .platoon_role import PlatoonRole
from .platooning_vehicle import PlatooningVehicle
from .util import get_crashed_vehicles, update_position
from .vehicle import Vehicle
from .vehicle_type import VehicleType

LOG = logging.getLogger(__name__)

# assumptions
# you just reach your arrival_position
# position is in the middle of the front bumper
# a vehicle ends at position + length
# crash detection does not work with step length greater than 1

# vehicle properties
_length = 4  # m # TODO make parameter
_max_speed = 55  # m/s # TODO make parameter
_max_acceleration = 2.5  # m/s # TODO make parameter
_max_deceleration = 15  # m/s # TODO make parameter
_min_gap = 2.5  # m # TODO make parameter
_desired_headway_time = 1.0  # s # TODO make parameter
vtype = VehicleType(
    "car",
    _length,
    _max_speed,
    _max_acceleration,
    _max_deceleration,
    _min_gap,
    _desired_headway_time
)  # TODO support multiple vtypes
TV = namedtuple('TV', ['position', 'rear_position', 'lane'])


class Simulator:
    """A collection of parameters and information of the simulator."""

    def __init__(
            self,
            road_length: int = 100 * 1000,
            number_of_lanes: int = 3,
            ramp_interval: int = 5000,
            pre_fill: bool = False,
            number_of_vehicles: int = 100,
            vehicle_density: int = 0,
            max_speed: float = 55,
            acc_headway_time: float = 1.0,
            cacc_spacing: float = 5.0,
            penetration_rate: float = 1.0,
            random_depart_position: bool = False,
            random_depart_lane: bool = False,
            desired_speed: float = 36.0,
            random_desired_speed: bool = True,
            speed_variation: float = 0.1,
            min_desired_speed: float = 22.0,
            max_desired_speed: float = 50.0,
            random_depart_speed: bool = False,
            depart_desired: bool = False,
            depart_flow: bool = False,
            depart_method: str = 'interval',
            depart_time_interval: int = 1,
            depart_probability: float = 1.0,
            depart_rate: int = 3600,
            depart_fixed_time: int = 0,
            random_arrival_position: bool = False,
            minimum_trip_length: int = 0,
            communication_range: int = 1000,
            start_as_platoon: bool = False,
            maximum_teleport_distance: int = 2000,
            formation_algorithm: str = None,
            formation_strategy: str = 'distributed',
            formation_centralized_kind: str = 'greedy',
            execution_interval: int = 1,
            alpha: float = 0.5,
            speed_deviation_threshold: float = 0.1,
            position_deviation_threshold: int = 300,
            number_of_infrastructures: int = 0,
            step_length: int = 1,
            max_step: int = 1 * 60 * 60,
            actions: bool = True,
            lane_changes: bool = True,
            collisions: bool = True,
            random_seed: int = -1,
            log_level: int = 'warning',
            gui: bool = False,
            gui_delay: int = 0,
            gui_track_vehicle: int = -1,
            result_base_filename: str = 'results',
            record_end_trace: bool = True,
            record_vehicle_trips: bool = False,
            record_vehicle_emissions: bool = False,
            record_vehicle_traces: bool = False,
            record_vehicle_changes: bool = False,
            record_emission_traces: bool = False,
            record_platoon_trips: bool = False,
            record_platoon_maneuvers: bool = False,
            record_platoon_formation: bool = False,
            record_platoon_traces: bool = False,
            record_platoon_changes: bool = False,
            record_infrastructure_assignments: bool = False,
            record_prefilled: bool = False):
        """Initializes a simulator instance."""

        # TODO add custom filter that prepends the log entry with the step time
        logging.basicConfig(level=log_level, format="%(levelname)s [%(name)s]: %(message)s")

        # road network properties
        self._road_length = road_length  # the length of the road
        self._number_of_lanes = number_of_lanes  # the number of lanes
        self._ramp_interval = ramp_interval  # the distance between any two on-/off-ramps

        # vehicle properties
        self._vehicles = {}  # the list (dict) of vehicles in the simulation
        self._last_vehicle_id = -1  # the id of the last vehicle generated
        self._number_of_vehicles = number_of_vehicles  # the maximum number of vehicles
        self._vehicle_density = vehicle_density  # the number of vehicles per km and lane
        self._max_speed = max_speed  # the maximum driving speed # FIXME not used
        self._acc_headway_time = acc_headway_time  # the headway time for ACC
        if acc_headway_time < 1.0:
            LOG.warning("Values for ACC headway time lower 1.0s are not recommended to avoid crashes!")
        self._cacc_spacing = cacc_spacing  # the constant spacing for CACC
        if cacc_spacing < 5.0:
            LOG.warning("Values for CACC spacing lower than 5.0m are not recommended to avoid crashes!")
        self._penetration_rate = penetration_rate  # the penetration rate of platooning vehicles

        # trip properties
        self._random_depart_position = random_depart_position  # whether to use random depart positions
        self._random_depart_lane = random_depart_lane  # whether to use random depart lanes
        self._desired_speed = desired_speed  # the desired driving speed
        self._random_desired_speed = random_desired_speed  # whether to use random desired driving speeds
        self._speed_variation = speed_variation  # the deviation from the desired driving speed
        self._min_desired_speed = min_desired_speed  # the minimum desired driving speed
        self._max_desired_speed = max_desired_speed  # the maximum desired driving speed
        self._random_depart_speed = random_depart_speed  # whether to use random departure speeds
        self._depart_desired = depart_desired  # whether to depart with the desired driving speed
        if random_depart_position and not depart_desired:
            sys.exit("ERROR: random-depart-position is only possible in conjunction with depart-desired!")
        self._depart_flow = depart_flow  # whether to spawn vehicles in a continuous flow
        self._depart_method = depart_method  # the departure method to use
        self._depart_time_interval = depart_time_interval  # the interval between two vehicle departures
        self._depart_probability = depart_probability  # the departure probability
        self._depart_rate = depart_rate  # the departure rate
        self._depart_fixed_time = depart_fixed_time  # the fixed departure time for all vehicles
        self._random_arrival_position = random_arrival_position  # whether to use random arrival positions
        self._minimum_trip_length = minimum_trip_length  # the minimum trip length
        if minimum_trip_length > road_length:
            sys.exit("ERROR: Minimum trip length cannot be bigger than the length of the entire road!")

        # communication properties
        self._communication_range = communication_range  # the maximum communication range between two vehicles
        if communication_range == -1:
            self._communication_range = road_length
        if self._communication_range <= 0:
            sys.exit("ERROR: Communication range has to be > 0!")

        # platoon properties
        self._start_as_platoon = start_as_platoon  # whether vehicles start as one platoon
        if start_as_platoon:
            if penetration_rate < 1.0:
                sys.exit("ERROR: The penetration rate cannot be smaller than 1.0 when starting as one platoon!")
            if formation_algorithm is not None:
                sys.exit("ERROR: A formation algorithm cannot be used when all starting as one platoon!")
            if depart_flow:
                sys.exit("ERROR: Vehicles can not spawn in a flow when starting as one platoon!")
            if random_depart_position:
                sys.exit("ERROR: Vehicles can not have random departure positions when starting as one platoon!")
            if random_depart_lane:
                sys.exit("ERROR: Vehicles can not have random departure lanes when starting as one platoon!")
            if random_arrival_position:
                sys.exit("ERROR: Vehicles can not have random arrival position when starting as one platoon!")
        if maximum_teleport_distance == -1:
            self._maximum_teleport_distance = self._road_length
            LOG.warning("No maximum teleport distance configured! The vehicle behavior may be unrealistic!")
        else:
            if maximum_teleport_distance >= self._ramp_interval:
                LOG.warning(f"A maximum teleport distance of {maximum_teleport_distance}m allows teleports beyond the next highway ramp! ")
            if maximum_teleport_distance >= minimum_trip_length and minimum_trip_length > 0:
                LOG.warning(f"A maximum teleport distance of {maximum_teleport_distance}m allows teleports beyond the minimum trip length!")
            self._maximum_teleport_distance = maximum_teleport_distance  # maximum teleport distance

        self._formation_algorithm = formation_algorithm  # the formation algorithm to use
        if formation_strategy == "centralized" and number_of_infrastructures == 0:
            sys.exit("ERROR: When using a centralized strategy at least 1 infrastructure is needed!")
        self._formation_strategy = formation_strategy  # the formation strategy to use
        self._formation_centralized_kind = formation_centralized_kind  # the kind of the centralized formation

        # formation properties
        # TODO find a different solution for algorithm specific parameters
        self._execution_interval = execution_interval  # the interval between two iterations of a formation algorithm
        if execution_interval <= 0:
            sys.exit("ERROR: Execution interval has to be at least 1 second!")
        self._alpha = alpha  # the weight of the speed deviation
        self._speed_deviation_threshold = speed_deviation_threshold  # the maximum deviation from the desired driving speed
        self._position_deviation_threshold = position_deviation_threshold  # the maximum deviation from the current position

        # infrastructure properties
        self._infrastructures = {}  # the list (dict) of infrastructures in the simulation

        # simulation properties
        self._step = 0  # the current simulation step in s
        assert(step_length > 0)
        if step_length != 1:
            LOG.warning("Values for step length other than 1s are not yet properly implemented and tested!")
        if step_length < 1.0:
            LOG.warning("Values for step length small than 1s are not recommended in order to avoid crashes!")
        self._step_length = step_length  # the length of a simulation step
        self._max_step = max_step
        self._running = False  # whether the simulation is running
        self._actions = actions  # whether to enable actions
        self._lane_changes = lane_changes  # whether to enable lane changes
        self._collisions = collisions  # whether to check for collisions
        if random_seed >= 0:
            self._random_seed = random_seed  # the random.seed to use for the RNG
            LOG.info(f"Using random seed {random_seed}")
            random.seed(random_seed)

        # gui properties
        self._gui = gui  # whether to show a live sumo-gui
        if gui:
            if 'SUMO_HOME' not in os.environ:
                sys.exit("ERROR: Environment variable 'SUMO_HOME' was not declared!")
            tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
            sys.path.append(tools)

        self._gui_delay = gui_delay  # the delay in every simulation step for the gui
        self._gui_track_vehicle = gui_track_vehicle  # the id of a vehicle to track in the gui

        # result recording properties
        self._result_base_filename = result_base_filename  # the base filename of the result files
        self._record_end_trace = record_end_trace  # whether to record another trace item at the trip end
        self._record_vehicle_trips = record_vehicle_trips  # whether to record vehicles trips
        self._record_vehicle_emissions = record_vehicle_emissions  # whether to record vehicle emissions
        self._record_vehicle_traces = record_vehicle_traces  # whether to record vehicle traces
        self._record_vehicle_changes = record_vehicle_changes  # whether to record vehicle lane changes
        self._record_emission_traces = record_emission_traces  # whether to record emission traces
        self._record_platoon_trips = record_platoon_trips  # whether to record platoon trips
        self._record_platoon_maneuvers = record_platoon_maneuvers  # whether to record platoon maneuvers
        self._record_platoon_formation = record_platoon_formation  # whether to record platoon formation
        self._record_platoon_traces = record_platoon_traces  # whether to record platoon traces
        self._record_platoon_changes = record_platoon_changes  # whether to record platoon lane changes
        self._record_infrastructure_assignments = record_infrastructure_assignments  # whether to record infrastructure assignments
        if record_prefilled:
            LOG.warning("Recording results for pre-filled vehicles is not recommended to avoid broken statistics!")
        self._record_prefilled = record_prefilled  # whether to record results for pre-filled vehicles

        # statistics
        self._avg_number_vehicles = 0
        self._values_in_avg_number_vehicles = 0

        # TODO log generation parameters
        self._pre_fill = pre_fill
        if pre_fill:
            self._generate_vehicles()

        self._generate_infrastructures(number_of_infrastructures)

    @property
    def road_length(self) -> int:
        """Returns the road length."""

        return self._road_length

    @property
    def number_of_lanes(self) -> int:
        """Returns the number of lanes."""

        return self._number_of_lanes

    @property
    def step_length(self) -> int:
        """Returns the length of a simulation step."""

        return self._step_length

    @property
    def step(self) -> int:
        """Returns the current simulation step."""

        return self._step

    def _call_vehicle_actions(self):
        """Triggers actions on all vehicles in the simulation."""

        for vehicle in self._vehicles.values():
            vehicle.action(self.step)

    def _call_infrastructure_actions(self):
        """Triggers actions on all infrastructures in the simulation."""

        for infrastructure in self._infrastructures.values():
            infrastructure.action(self.step)

    def _get_predecessor(self, vehicle: Vehicle, lane: int = -1) -> Vehicle:
        """
        Returns the preceding (i.e., front) vehicle for a given vehicle on a given lane.

        Parameters
        ----------
        vehicle : Vehicle
            The vehicle to consider
        lane : int, optional
            The lane to consider.
            A lane of -1 indicates the vehicle's current lane.
        """

        if lane == -1:
            # implicitly search on current lane of vehicle
            lane = vehicle.lane
        predecessor = None  # there is no predecessor so far
        candidates = [
            v for v in self._vehicles.values() if
            v is not vehicle and  # not this vehicle
            v.lane == lane and  # correct lane
            v.position >= vehicle.position  # in front this vehicle
        ]
        for other_vehicle in candidates:
            # we do not check for collisions here because this method is also called within an update step
            if predecessor is None or other_vehicle.rear_position < predecessor.rear_position:
                # the current vehicle is closer to us than the previous predecessor
                predecessor = other_vehicle
        return predecessor

    def _get_successor(self, vehicle: Vehicle, lane: int = -1) -> Vehicle:
        """
        Returns the succeeding (i.e., back) vehicle for a given vehicle on a given lane.

        Parameters
        ----------
        vehicle : Vehicle
            The vehicle to consider
        lane : int, optional
            The lane to consider.
            A lane of -1 indicates the vehicle's current lane.
        """

        if lane == -1:
            # implicitly search on current lane of vehicle
            lane = vehicle.lane
        successor = None  # there is no successor so far
        candidates = [
            v for v in self._vehicles.values() if
            v is not vehicle and  # not this vehicle
            v.lane == lane and  # correct lane
            v.position <= vehicle.position  # behind this vehicle
        ]
        for other_vehicle in candidates:
            # we do not check for collisions here because this method is also called within an update step
            if successor is None or other_vehicle.position > successor.position:
                # the current vehicle is closer to us than the previous successor
                successor = other_vehicle
        return successor

    def _get_predecessor_rear_position(self, vehicle: Vehicle, lane: int = -1) -> float:
        """
        Returns the rear position of the preceding (i.e., front) vehicle for a given vehicle on a given lane.

        Parameters
        ----------
        vehicle : Vehicle
            The vehicle to consider
        lane : int, optional
            The lane to consider.
            A lane of -1 indicates the vehicle's current lane.
        """

        p = self._get_predecessor(vehicle, lane)
        if p is None:
            return -1
        else:
            return p.rear_position

    def _get_predecessor_speed(self, vehicle: Vehicle, lane: int = -1) -> float:
        """
        Returns the speed of the preceding (i.e., front) vehicle for a given vehicle on a given lane.

        Parameters
        ----------
        vehicle : Vehicle
            The vehicle to consider
        lane : int, optional
            The lane to consider.
            A lane of -1 indicates the vehicle's current lane.
        """

        p = self._get_predecessor(vehicle, lane)
        if p is None:
            return -1
        else:
            return p.speed

    def is_lane_change_safe(self, vehicle: Vehicle, target_lane: int) -> bool:
        """
        Determines whether a lane change for a given vehicle is safe.

        Parameters
        ----------
        vehicle :  Vehicle
            The vehicle to change lanes
        target_lane : int
            The target lane for the lane change
        """

        if vehicle.lane == target_lane:
            return True

        # check predecessor on target lane
        p = self._get_predecessor(vehicle, target_lane)
        if p is not None:
            gap_to_predecessor_on_target_lane = p.rear_position - vehicle.position
            if gap_to_predecessor_on_target_lane < 0:
                LOG.debug(f"{vehicle.vid}'s lane change is not safe because of its predecessor")
                return False
            if vehicle.speed >= vehicle._safe_speed(p.speed, gap_to_predecessor_on_target_lane, vehicle.desired_gap, vehicle.min_gap):
                LOG.debug(f"{vehicle.vid}'s lane change is not safe because of its predecessor")
                return False

        # check successor on target lane
        s = self._get_successor(vehicle, target_lane)
        if s is not None:
            gap_to_successor_on_target_lane = vehicle.rear_position - s.position
            if gap_to_successor_on_target_lane < 0:
                LOG.debug(f"{vehicle.vid}'s lane change is not safe because of its successor")
                return False
            if s.speed >= s._safe_speed(vehicle.speed, gap_to_successor_on_target_lane, s.desired_gap, s.min_gap):
                LOG.debug(f"{vehicle.vid}'s lane change is not safe because of its successor")
                return False

        # safe
        return True

    # TODO move to vehicle?
    def _change_lane(self, vehicle: Vehicle, target_lane: int, reason: str) -> bool:
        """
        Performs a lane change for a given vehicle.

        This is based on Krauss' multi lane traffic:
        lane-change
        congested = (v_safe < v_thresh) and (v^0_safe < v_thresh)
        favorable(right->left) = (v_safe < v_max) and (not congested)
        favorable(left->right) = (v_safe >= v_max) and (v^0_safe >= v_max)
        if ((favorable(i->j) or (rand < p_change)) and safe(i->j)) then change(i->j)
        for vehicles on the right lane:
        if (v > v^0_safe) and (not congested) then v <- v^0_safe

        Parameters
        ----------
        vehicle :  Vehicle
            The vehicle to change lanes
        target_lane : int
            The target lane for the lane change
        reason : str
            The reason for the lane change
        """

        source_lane = vehicle.lane
        if source_lane == target_lane:
            return True
        LOG.debug(f"{vehicle.vid} wants to change from lane {source_lane} to lane {target_lane} ({reason})")

        lane_diff = target_lane - source_lane
        if abs(lane_diff) > 1:
            LOG.warning(f"{vehicle.vid} can only change to adjacent lane!")
            old_target_lane = target_lane
            target_lane = source_lane + copysign(1, lane_diff)
            LOG.info(f"Adjusted target lane of {vehicle.vid} to {target_lane} (from {old_target_lane})")

        if isinstance(vehicle, PlatooningVehicle) and vehicle.is_in_platoon():
            # followers are not allowed to change the lane on their one
            assert(vehicle.platoon_role is not PlatoonRole.FOLLOWER)

            # leaders are allowed to change the lane
            if vehicle.platoon_role == PlatoonRole.LEADER:
                assert(reason == "speedGain" or reason == "keepRight")

                LOG.debug(f"{vehicle.vid} needs to check all its platoon members")

                can_change = True
                for member in vehicle.platoon.formation:
                    can_change = can_change and self.is_lane_change_safe(member, target_lane)
                    if not can_change:
                        LOG.debug(f"lane change is not safe for member {member.vid}")

                if can_change:
                    # perform lane change for all vehicles in this platoon
                    for member in vehicle.platoon.formation:
                        assert(member.lane == source_lane)
                        LOG.debug(f"{member.vid} is switching lanes: {source_lane} -> {target_lane} ({reason})")

                        # switch to adjacent lane
                        member._lane = target_lane

                        if self._record_platoon_changes:
                            # log lane change
                            with open(f'{self._result_base_filename}_platoon_changes.csv', 'a') as f:
                                f.write(
                                    f"{self.step},"
                                    f"{member.vid},"
                                    f"{member.position},"
                                    f"{source_lane},"
                                    f"{target_lane},"
                                    f"{member.speed},"
                                    f"{reason}"
                                    "\n"
                                )

                    return abs(lane_diff) <= 1
                LOG.debug(f"{vehicle.vid}'s lane change is not safe")
                return False

        # we are just a regular vehicle or we are not (yet) in a platoon

        # check adjacent lane is free
        if self.is_lane_change_safe(vehicle, target_lane):
            LOG.debug(f"{vehicle.vid} is switching lanes: {source_lane} -> {target_lane} ({reason})")

            # switch to adjacent lane
            vehicle._lane = target_lane

            if self._record_vehicle_changes:
                # log lane change
                with open(f'{self._result_base_filename}_vehicle_changes.csv', 'a') as f:
                    f.write(
                        f"{self.step},"
                        f"{vehicle.vid},"
                        f"{vehicle.position},"
                        f"{source_lane},"
                        f"{target_lane},"
                        f"{vehicle.speed},"
                        f"{reason}"
                        "\n"
                    )

            return abs(lane_diff) <= 1
        LOG.debug(f"{vehicle.vid}'s lane change is not safe")
        return False

    def _change_lanes(self):
        """
        Does lane changes for all vehicles in the simulation.

        This is based on Krauss' multi lane traffic:
        lane-change
        congested = (v_safe < v_thresh) and (v^0_safe < v_thresh)
        favorable(right->left) = (v_safe < v_max) and (not congested)
        favorable(left->right) = (v_safe >= v_max) and (v^0_safe >= v_max)
        if ((favorable(i->j) or (rand < p_change)) and safe(i->j)) then change(i->j)
        for vehicles on the right lane:
        if (v > v^0_safe) and (not congested) then v <- v^0_safe
        """

        for vehicle in self._vehicles.values():
            self._adjust_lane(vehicle)

    def _adjust_lane(self, vehicle: Vehicle):
        """
        Decides upon and performs a lane change for a given vehicle.

        This is based on Krauss' multi lane traffic:
        lane-change
        congested = (v_safe < v_thresh) and (v^0_safe < v_thresh)
        favorable(right->left) = (v_safe < v_max) and (not congested)
        favorable(left->right) = (v_safe >= v_max) and (v^0_safe >= v_max)
        if ((favorable(i->j) or (rand < p_change)) and safe(i->j)) then change(i->j)
        for vehicles on the right lane:
        if (v > v^0_safe) and (not congested) then v <- v^0_safe

        Parameters
        ----------
        vehicle :  Vehicle
            The vehicle to be adjusted
        """

        if vehicle.blocked_front:
            if vehicle.lane < self.number_of_lanes - 1:
                target_lane = vehicle.lane + 1
                # TODO determine whether it is useful to overtake
                self._change_lane(vehicle, target_lane, "speedGain")
        else:
            if isinstance(vehicle, PlatooningVehicle) and vehicle.platoon_role == PlatoonRole.FOLLOWER:
                # followers are not allowed to change the lane on their own
                return
            if vehicle.lane > 0:
                target_lane = vehicle.lane - 1
                self._change_lane(vehicle, target_lane, "keepRight")

    def _adjust_speeds(self):
        """
        Updates the speed (i.e., acceleration & speed) of all vehicles in the simulation.

        This does not (yet) use a vectorized approach.
        """

        for vehicle in sorted(self._vehicles.values(), key=lambda x: x.position, reverse=True):
            self._adjust_speed(vehicle)

    def _adjust_speed(self, vehicle: Vehicle):
        """
        Updates the speed (i.e., acceleration & speed) of a given vehicle.

        Parameters
        ----------
        vehicle: Vehicle
            The vehicle to be updated
        """

        LOG.debug(f"{vehicle.vid}'s current acceleration: {vehicle.acceleration}")
        LOG.debug(f"{vehicle.vid}'s current speed {vehicle.speed}")
        predecessor = self._get_predecessor(vehicle)
        new_speed = vehicle.new_speed(predecessor.speed if predecessor else -1, predecessor.rear_position if predecessor else -1)
        vehicle._acceleration = new_speed - vehicle.speed
        vehicle._speed = new_speed
        LOG.debug(f"{vehicle.vid}'s new acceleration: {vehicle.acceleration}")
        LOG.debug(f"{vehicle.vid}'s new speed {vehicle.speed}")

    def _remove_arrived_vehicles(self, arrived_vehicles: list):
        """
        Removes arrived vehicles from the simulation.

        Parameters
        ----------
        arrived_vehicles : list
            The ids of arrived vehicles
        """

        for vid in arrived_vehicles:
            # call finish on arrived vehicle
            self._vehicles[vid].finish()
            # remove arrived vehicle from gui
            if self._gui:
                import traci
                traci.vehicle.remove(str(vid), 2)
            # remove from vehicles
            del self._vehicles[vid]

    @staticmethod
    def _check_collisions(vdf: pd.DataFrame):
        """Does collision checks for all vehicles in the simulation."""

        if vdf.empty:
            return

        crashed_vehicles = get_crashed_vehicles(vdf)
        if crashed_vehicles:
            for v in crashed_vehicles:
                print(f"{v}: {vdf.loc[v].position}:{vdf.loc[v].length},{vdf.loc[v].lane}")
            sys.exit(f"ERROR: There were collisions with the following vehicles {crashed_vehicles}!")

    @staticmethod
    def has_collision(vehicle1: TV, vehicle2: TV) -> bool:
        """
        Checks for a collision between two vehicles.

        Parameters
        ----------
        vehicle1 : TV(position, rear_position, lane)
        vehicle2 : TV(position, rear_position, lane)
        """
        assert(vehicle1 is not vehicle2)
        assert(vehicle1.lane == vehicle2.lane)
        return min(vehicle1.position, vehicle2.position) - max(vehicle1.rear_position, vehicle2.rear_position) >= 0

    def _generate_vehicles(self):
        """Adds pre-filled vehicles to the simulation."""

        if self._vehicle_density > 0:
            number_of_vehicles = round(self._vehicle_density * int(self._road_length / 1000) * self._number_of_lanes)
        else:
            number_of_vehicles = self._number_of_vehicles

        LOG.info(f"Pre-filling the road network with {number_of_vehicles} vehicles")

        for num in tqdm(range(0, number_of_vehicles), desc="Generated vehicles"):

            vid = self._last_vehicle_id + 1
            depart_time = -1  # since this is a pre-filled vehicle, we cannot say when it departed

            # TODO remove duplicated code
            if self._start_as_platoon:
                depart_position = (number_of_vehicles - vid) * (vtype.length + self._cacc_spacing)
                depart_lane = 0
            else:
                # assume we have a collision to check at least once
                collision = True
                while collision:
                    collision = False
                    # actual calculation of position and lane
                    # always use random position for pre-filled vehicle
                    # we do not consider depart interval here since this is supposed to be a snapshot from an earlier point of simulation
                    # make sure to also include the end of the road itself
                    # consider length, equal to departPos="base" in SUMO
                    depart_position = random.uniform(vtype.length, self.road_length)
                    # always use random lane for pre-filled vehicle
                    depart_lane = random.randrange(0, self.number_of_lanes, 1)

                    LOG.debug(f"Generated random depart position ({depart_position},{depart_lane}) for vehicle {vid}")

                    if not self._vehicles:
                        continue

                    # avoid a collision with an existing vehicle
                    for other_vehicle in self._vehicles.values():
                        if other_vehicle.lane != depart_lane:
                            # we do not care about other lanes
                            continue
                        tv = TV(depart_position + vtype.min_gap, depart_position - vtype.length, depart_lane)
                        otv = TV(other_vehicle.position + other_vehicle.min_gap, other_vehicle.rear_position, other_vehicle.lane)
                        collision = collision or self.has_collision(tv, otv)

            desired_speed = self._get_desired_speed()

            # always use desired speed for pre-fill vehicles
            depart_speed = desired_speed

            arrival_position = self._get_arrival_position(depart_position, prefill=True)

            self._add_vehicle(
                vid,
                vtype,
                depart_position,
                arrival_position,
                desired_speed,
                depart_lane,
                depart_speed,
                depart_time,
                self._communication_range
            )

            LOG.debug(f"Generated vehicle {vid} at {depart_position}-{depart_position - vtype.length},{depart_lane} with {depart_speed}")

    def _get_desired_speed(self) -> float:
        """Returns a (random) depart speed."""

        if self._random_desired_speed:
            # normal distribution
            desired_speed = self._desired_speed * random.normalvariate(1.0, self._speed_variation)
            desired_speed = max(desired_speed, self._min_desired_speed)
            desired_speed = min(desired_speed, self._max_desired_speed)
        else:
            desired_speed = self._desired_speed

        return desired_speed

    def _get_depart_position(self) -> int:
        """
        Returns a (random) depart position for a given depart position.

        This considers the ramp interval, road length, and minimum trip length.
        """

        # NOTE: this should only be called for non-pre-filled vehicles
        if self._random_depart_position:
            # set maximum theoretical depart position
            # make sure that the vehicles can drive for at least the minimum length of a trip
            # and at least for one ramp
            max_depart = self._road_length - max(self._minimum_trip_length, self._ramp_interval)
            max_depart_ramp = max_depart - (self._ramp_interval + max_depart) % self._ramp_interval
            assert(max_depart_ramp <= self._road_length)
            assert(max_depart_ramp >= 0)
            if max_depart_ramp == 0:
                # start at beginning
                depart_position = 0
            else:
                depart_position = random.randrange(0, max_depart_ramp, self._ramp_interval)
            assert(depart_position <= max_depart_ramp)
        else:
            # simply start at beginning
            depart_position = 0

        assert(depart_position >= 0)
        assert(depart_position <= self.road_length)

        return depart_position

    def _get_arrival_position(self, depart_position: int, prefill: bool = False) -> int:
        """
        Returns a (random) arrival position for a given depart position.

        This considers the ramp interval, road length, and minimum trip length.

        Parameters
        ----------
        depart_position : int
            The depart position to consider
        prefill : bool, optional
            Whether the trip is for a pre-filled vehicle
        """

        if self._random_arrival_position:
            # set minimum theoretical arrival position
            if prefill:
                # We cannot use the minimum trip time here,
                # since the pre-generation is supposed to produce a snapshot of a realistic simulation.
                # But we can assume that a vehicle has to drive at least 1m
                min_arrival = depart_position + 1
            else:
                # make sure that the vehicles drive at least for the minimum length of a trip
                # and at least for one ramp
                min_arrival = depart_position + max(self._minimum_trip_length, self._ramp_interval)
            min_arrival_ramp = min_arrival + (self._ramp_interval - min_arrival) % self._ramp_interval
            assert(min_arrival_ramp >= 0)
            assert(min_arrival_ramp <= self._road_length)
            if min_arrival % self._ramp_interval == 0:
                assert(min_arrival == min_arrival_ramp)
            if min_arrival_ramp == self._road_length:
                # avoid empty randrange
                # exit at end
                arrival_position = self._road_length
            else:
                # make sure to also include the end of the road itself
                arrival_position = random.randrange(min_arrival_ramp, self._road_length + 1, self._ramp_interval)
            assert(arrival_position >= min_arrival_ramp)
        else:
            # simply drive until the end
            arrival_position = self._road_length

        assert(arrival_position <= self._road_length)
        assert(arrival_position > depart_position)

        return arrival_position

    def _spawn_vehicle(self):
        """Spawns a vehicle within the simulation."""

        if self._vehicle_density > 0:
            number_of_vehicles = self._vehicle_density * int(self._road_length / 1000) * self._number_of_lanes
        else:
            number_of_vehicles = self._number_of_vehicles

        if len(self._vehicles) >= number_of_vehicles:
            LOG.debug(f"Number of vehicles {number_of_vehicles} is reached already")
            return

        if not self._depart_flow and self._last_vehicle_id >= number_of_vehicles - 1:
            LOG.debug(f"All {number_of_vehicles} vehicles have been spawned already")
            return

        spawn = False  # should we spawn a new vehicle in this timestep?
        if self._depart_method == "interval":
            # spawn interval
            spawn = self.step % self._depart_time_interval == 0  # is the time step correct?
            if self._last_vehicle_id in self._vehicles.keys():
                spawn = spawn and self._vehicles[self._last_vehicle_id].depart_time != self.step
        elif self._depart_method == "probability":
            # spawn probability per time step
            spawn = random.random() <= self._depart_probability
        elif self._depart_method == "rate":
            # spawn #vehicles per hour
            spawn_interval = 3600 / self.step_length / self._depart_rate
            spawn = self.step % round(spawn_interval) == 0
        elif self._depart_method == "fixed" and self.step == self._depart_fixed_time:
            # we create all of the vehicles now
            sys.exit("ERROR: depart method fixed is not yet implemented!")
        else:
            sys.exit("ERROR: Unknown depart method!")

        if spawn:
            depart_time = self.step
        else:
            # we do not spawn a vehicle in this timestep
            return

        vid = self._last_vehicle_id + 1

        if self._random_depart_lane:
            depart_lane = random.randrange(0, self.number_of_lanes, 1)
        else:
            depart_lane = 0

        depart_position = self._get_depart_position()
        arrival_position = self._get_arrival_position(depart_position)
        depart_position += vtype.length  # equal to departPos="base" in SUMO

        desired_speed = self._get_desired_speed()

        if self._random_depart_speed:
            # make sure to also include the desired speed itself
            depart_speed = random.randrange(0, self._desired_speed + 1, 1)
        else:
            depart_speed = 0

        if self._depart_desired:
            depart_speed = desired_speed

        LOG.debug(f"Spawning vehicle {vid} at {depart_position}-{depart_position - vtype.length},{depart_lane} with {depart_speed}")

        # TODO remove duplicated code
        # check whether the vehicle can actually be inserted safely
        # assume we have a collision to check at least once
        collision = bool(self._vehicles)
        while collision:
            collision = False  # so far we do not have a collision
            LOG.debug(f"Checking for a collision with an existing vehicle for new vehicle {vid}")
            # avoid a collision with an existing vehicle
            # check all vehicles
            for other_vehicle in self._vehicles.values():
                if other_vehicle.lane != depart_lane:
                    # we do not care about other lanes
                    continue
                # do we have a collision?
                # avoid being inserted in between two platoon members by also considering the min gap
                # TODO use the desired headway time for safe insertion on another lane
                tv = TV(
                    depart_position + vtype.min_gap,  # front collider
                    depart_position - vtype.length,  # rear collider
                    depart_lane
                )
                otv = TV(
                    other_vehicle.position + other_vehicle.min_gap,  # front collider
                    other_vehicle.rear_position,  # rear collider
                    other_vehicle.lane
                )
                collision = collision or self.has_collision(tv, otv)

            if collision:
                # can we avoid the collision by switching the departure lane?
                if depart_lane == self.number_of_lanes - 1:
                    # reached maximum number of lanes already
                    # TODO delay insertion of vehicle
                    LOG.warning(f"Could not further increase depart lane ({depart_lane}) for vehicle {vid}! You might want to reduce the number of vehicles to reduce the traffic. Vehicle {vid} will not be spawned, since delaying insertion of vehicles is not (yet) implemented!")
                    return  # do not insert this vehicle but also do not abort the simulation
                depart_lane = depart_lane + 1
                LOG.info(f"Increased depart lane for {vid} to avoid a collision (now lane {depart_lane})")
                # we need to check again

        vehicle = self._add_vehicle(
            vid,
            vtype,
            depart_position,
            arrival_position,
            desired_speed,
            depart_lane,
            depart_speed,
            depart_time,
            self._communication_range
        )

        if self._gui:
            self._add_gui_vehicle(vehicle)

        LOG.info(f"Spawned vehicle {vid} ({depart_position}-{vehicle.rear_position},{depart_lane})")

        if self._start_as_platoon and vid > 0:
            vehicle._join(0, 0)

    def _add_vehicle(
            self,
            vid,
            vtype,
            depart_position,
            arrival_position,
            desired_speed,
            depart_lane,
            depart_speed,
            depart_time,
            communication_range
    ):
        """
        Adds a vehicle to the simulation based on the given parameters.

        Parameters
        ----------
        vid : int
            The id of the vehicle
        vtype : VehicleType
            The vehicle type of the vehicle
        depart_position : int
            The depart position of the vehicle
        arrival_position : int
            The arrival position of the vehicle
        desired_speed : float
            The desired driving speed of the vehicle
        depart_lane : int
            The depart lane of the vehicle
        depart_speed : float
            The depart speed of the vehicle
        depart_time : int
            The depart time of the vehicle
        communication_range : int
            The maximum communication range of the vehicle
        """

        # choose vehicle "type" depending on the penetration rate
        if random.random() <= self._penetration_rate:
            vehicle = PlatooningVehicle(
                self,
                vid,
                vtype,
                depart_position,
                arrival_position,
                desired_speed,
                depart_lane,
                depart_speed,
                depart_time,
                self._communication_range,
                self._acc_headway_time,
                self._cacc_spacing,
                self._formation_algorithm if self._formation_strategy == "distributed" else None,
                self._execution_interval,
                self._alpha,
                self._speed_deviation_threshold,
                self._position_deviation_threshold)
        else:
            vehicle = Vehicle(
                self,
                vid,
                vtype,
                depart_position,
                arrival_position,
                desired_speed,
                depart_lane,
                depart_speed,
                depart_time,
                self._communication_range,
            )

        # add instance
        self._vehicles[vid] = vehicle
        self._last_vehicle_id = vid

        return vehicle

    def _generate_infrastructures(self, number_of_infrastructures: int):
        """Generates infrastructures for the simulation."""

        if number_of_infrastructures <= 0:
            return

        last_infrastructure_id = -1

        placement_interval = self.road_length / number_of_infrastructures

        for num in tqdm(range(0, number_of_infrastructures), desc="Generated infrastructures"):
            iid = last_infrastructure_id + 1
            position = (iid + 0.5) * placement_interval

            infrastructure = Infrastructure(self,
                                            iid,
                                            position,
                                            self._formation_algorithm if self._formation_strategy == "centralized" else None,
                                            self._formation_centralized_kind,
                                            self._execution_interval,
                                            self._alpha,
                                            self._speed_deviation_threshold,
                                            self._position_deviation_threshold)
            self._infrastructures[iid] = infrastructure

            LOG.info(f"Generated infrastructure {infrastructure} at {position}")

            last_infrastructure_id = iid

    def _initialize_result_recording(self):
        """Creates output files for all (enabled) statistics and writes the headers."""

        # write some general information about the simulation
        with open(f'{self._result_base_filename}_general.out', 'w') as f:
            f.write(f"simulation start: {time.asctime(time.localtime(time.time()))}\n")
            f.write(f"parameters {str(self)}\n")

        if self._record_vehicle_trips:
            # create output file for vehicle trips
            with open(f'{self._result_base_filename}_vehicle_trips.csv', 'w') as f:
                f.write(
                    "id,"
                    "depart,"
                    "departLane,"
                    "departPos,"
                    "departSpeed,"
                    "arrival,"
                    "arrivalLane,"
                    "arrivalPos,"
                    "arrivalSpeed,"
                    "duration,"
                    "routeLength,"
                    "timeLoss,"
                    "desiredSpeed,"
                    "expectedTravelTime,"
                    "travelTimeRatio,"
                    "avgDrivingSpeed,"
                    "avgDeviationDesiredSpeed"
                    "\n"
                )

        if self._record_vehicle_emissions:
            # create output file for vehicle emissions
            with open(f'{self._result_base_filename}_vehicle_emissions.csv', 'w') as f:
                f.write(
                    "id,"
                    "CO,"
                    "CO2,"
                    "HC,"
                    "PMx,"
                    "NOx,"
                    "fuel"
                    "\n"
                )

        if self._record_vehicle_traces:
            # create output file for vehicle traces
            with open(f'{self._result_base_filename}_vehicle_traces.csv', 'w') as f:
                f.write(
                    "step,"
                    "id,"
                    "position,"
                    "lane,"
                    "speed,"
                    "duration,"
                    "routeLength,"
                    "desiredSpeed"
                    "\n"
                )

        if self._record_vehicle_changes:
            # create output file for vehicle lane changes
            with open(f'{self._result_base_filename}_vehicle_changes.csv', 'w') as f:
                f.write(
                    "step,"
                    "id,"
                    "position,"
                    "from,"
                    "to,"
                    "speed,"
                    "reason"
                    "\n"
                )

        if self._record_emission_traces:
            # create output file for emission traces
            with open(f'{self._result_base_filename}_emission_traces.csv', 'w') as f:
                f.write(
                    "step,"
                    "id,"
                    "CO,"
                    "CO2,"
                    "HC,"
                    "PMx,"
                    "NOx,"
                    "fuel"
                    "\n"
                )

        if self._record_platoon_trips:
            # create output file for platoon trips
            with open(f'{self._result_base_filename}_platoon_trips.csv', 'w') as f:
                f.write(
                    "id,"
                    "timeInPlatoon,"
                    "distanceInPlatoon,"
                    "platoonTimeRatio,"
                    "platoonDistanceRatio,"
                    "numberOfPlatoons,"
                    "timeUntilFirstPlatoon,"
                    "distanceUntilFirstPlatoon"
                    "\n"
                )

        if self._record_platoon_maneuvers:
            # create output file for platoon maneuvers
            with open(f'{self._result_base_filename}_platoon_maneuvers.csv', 'w') as f:
                f.write(
                    "id,"
                    "joinsAttempted,"
                    "joinsSuccessful,"
                    "joinsAborted,"
                    "joinsAbortedFront,"
                    "joinsAbortedArbitrary,"
                    "joinsAbortedRoadBegin,"
                    "joinsAbortedTripBegin,"
                    "joinsAbortedTripEnd,"
                    "joinsAbortedLeaderManeuver,"
                    "joinsAbortedTeleportThreshold,"
                    "joinsAbortedNoSpace,"
                    "joinsFront,"
                    "joinsArbitrary,"
                    "joinsBack,"
                    "joinsTeleportPosition,"
                    "joinsTeleportLane,"
                    "joinsTeleportSpeed,"
                    "joinsCorrectPosition,"
                    "leavesAttempted,"
                    "leavesSuccessful,"
                    "leavesAborted,"
                    "leavesFront,"
                    "leavesArbitrary,"
                    "leavesBack"
                    "\n"
                )

        if self._record_platoon_formation:
            # create output file for platoon formation
            with open(f'{self._result_base_filename}_platoon_formation.csv', 'w') as f:
                f.write(
                    "id,"
                    "candidatesFound,"
                    "candidatesFiltered,"
                    "candidatesFilteredFollower,"
                    "candidatesFilteredManeuver"
                    "\n"
                )

        if self._record_platoon_traces:
            # create output file for platoon traces
            with open(f'{self._result_base_filename}_platoon_traces.csv', 'w') as f:
                f.write(
                    "step,"
                    "id,"
                    "platoon,"
                    "leader,"
                    "position,"
                    "rearPosition,"
                    "lane,"
                    "speed,"
                    "size,"
                    "length,"
                    "desiredSpeed,"
                    "platoonRole,"
                    "platoonPosition"
                    "\n"
                )

        if self._record_platoon_changes:
            # create output file for platoon lane changes
            with open(f'{self._result_base_filename}_platoon_changes.csv', 'w') as f:
                f.write(
                    "step,"
                    "id,"
                    "position,"
                    "from,"
                    "to,"
                    "speed,"
                    "reason"
                    "\n"
                )

        if self._record_infrastructure_assignments:
            # create output file for infrastructure assignments
            with open(f'{self._result_base_filename}_infrastructure_assignments.csv', 'w') as f:
                f.write(
                    "id,"
                    "assignmentsSolved,"
                    "assignmentsNotSolvable,"
                    "assignmentsNone,"
                    "assignmentsSelf,"
                    "assignmentsCandidateJoinedAlready,"
                    "assignmentsVehicleBecameLeader,"
                    "assignmentsSuccessful,"
                    "\n"
                )

    def _initialize_gui(self):
        """Initializes the GUI via TraCI."""

        sumoBinary = os.path.join(os.environ['SUMO_HOME'], 'bin/sumo-gui')
        sumoCmd = [sumoBinary, "-Q", "-c", "sumocfg/freeway.sumo.cfg", '--collision.action', 'none']

        import traci
        traci.start(sumoCmd)

        # draw ramps
        y = 241
        color = (0, 0, 0)
        width = 4
        height = 150
        for x in range(0, self._road_length + 1, self._ramp_interval):
            traci.polygon.add(f"ramp-{x}", [
                (x - width / 2, y),  # top left
                (x + width / 2, y),  # top right
                (x + width / 2, y - height),  # bottom right
                (x - width / 2, y - height)   # bottom left
            ], color, fill=True)
            traci.poi.add(f"Ramp at {x}m", x=x, y=y - height - 10, color=(51, 128, 51))

        # draw road end
        y_top = 340
        y_bottom = 241
        width = 4
        color = (255, 0, 0)
        traci.polygon.add("road-end", [
            (self._road_length - width / 2, y_bottom),  # bottom left
            (self._road_length + width / 2, y_bottom),  # bottom right
            (self._road_length + width / 2, y_top),  # top right
            (self._road_length - width / 2, y_top)  # top left
        ], color, fill=True, layer=3)
        traci.poi.add("Road End", x=self._road_length + 50, y=300, color=(51, 128, 51))

        # draw infrastructures
        y = 280
        width = 20
        color = (0, 0, 255)
        for infrastructure in self._infrastructures.values():
            # add infrastructure
            if (str(infrastructure.iid)) not in traci.polygon.getIDList():
                traci.polygon.add(f"rsu-{str(infrastructure.iid)}", [
                    (infrastructure.position - width / 2, y),  # bottom left
                    (infrastructure.position + width / 2, y),  # bottom right
                    (infrastructure.position + width / 2, y + width),  # top right
                    (infrastructure.position - width / 2, y + width)  # top left
                ], color, fill=True)
                traci.poi.add(f"RSU {infrastructure.iid}", x=infrastructure.position, y=y + width + 10, color=(51, 128, 51))

        # draw pre-filled vehicles
        # save internal state of random number generator
        state = random.getstate()
        for vehicle in self._vehicles.values():
            self._add_gui_vehicle(vehicle)
        # restore internal state of random number generator to not influence the determinism of the simulation
        random.setstate(state)

    def _update_gui(self):
        """Updates the GUI via TraCI."""

        import traci
        for vehicle in self._vehicles.values():
            # update vehicles
            traci.vehicle.setSpeed(str(vehicle.vid), vehicle.speed)
            traci.vehicle.moveTo(vehID=str(vehicle.vid), pos=vehicle.position, laneID=f'edge_0_0_{vehicle.lane}')

        # remove vehicles not in simulator
        for vid in traci.vehicle.getIDList():
            if int(vid) not in self._vehicles.keys():
                traci.vehicle.remove(vid, 2)

        # sleep for visualization
        time.sleep(self._gui_delay)

    def _add_gui_vehicle(self, vehicle: Vehicle):
        """
        Adds a vehicle to the GUI via TraCI.

        Parameters
        ----------
        vehicle : Vehicle
            The vehicle to add to the GUI
        """

        import traci
        if str(vehicle.vid) not in traci.vehicle.getIDList():
            traci.vehicle.add(str(vehicle.vid), 'route', departPos=str(vehicle.position), departSpeed=str(vehicle.speed), departLane=str(vehicle.lane), typeID='vehicle')
            # save internal state of random number generator
            state = random.getstate()
            color = (random.randrange(0, 255, 1), random.randrange(0, 255, 1), random.randrange(0, 255, 1))
            traci.vehicle.setColor(str(vehicle.vid), color)
            vehicle._color = color
            # restore internal state of random number generator to not influence the determinism of the simulation
            if not (self._pre_fill and self._step == 0):
                # By disabling the reset of the random state when using pre-fill and in the first time step,
                # we achieve a (deterministic) random color for all pre-filled vehicles.
                # Otherwise, when resetting the state step 0, all pre-filled vehicles did not have a random color
                # (instead they had the same), since the RNG did not pick any other numbers since the last color pick.
                random.setstate(state)
            traci.vehicle.setSpeedMode(str(vehicle.vid), 0)
            traci.vehicle.setLaneChangeMode(str(vehicle.vid), 0)
            # track vehicle
            if vehicle.vid == self._gui_track_vehicle:
                traci.gui.trackVehicle("View #0", str(vehicle.vid))
                traci.gui.setZoom("View #0", 1000000)

    def run(self):
        """
        Main simulation method.
        Runs the simulation with the specified parameters until it is stopped.

        This is based on Krauss' multi lane traffic.
        """

        if not self._running:
            self._running = True
        else:
            LOG.warning("Simulation is already running!")

        self._initialize_result_recording()

        # initialize the GUI
        if self._gui:
            self._initialize_gui()

        # initialize pre-filled vehicles
        for vehicle in self._vehicles.values():
            if self._start_as_platoon and vehicle.vid > 0:
                vehicle._join(0, 0)

        progress_bar = tqdm(desc='Simulation progress', total=self._max_step, unit='step')
        # let the simulator run
        while self._running:
            if self.step >= self._max_step:
                self.stop("Reached step limit")
                continue

            # spawn vehicle based on given parameters
            self._spawn_vehicle()

            if not self._vehicles:
                self.stop("No more vehicles in the simulation")  # do we really want to exit here?
                continue

            # update the GUI
            if self._gui:
                self._update_gui()

            # call regular actions on vehicles
            self._call_vehicle_actions()
            # call regular actions on infrastructure
            self._call_infrastructure_actions()

            # perform lane changes (for all vehicles)
            if self._lane_changes:
                self._change_lanes()

            # adjust speed (of all vehicles)
            self._adjust_speeds()

            # BEGIN VECTORIZATION PART
            # convert dict of vehicles to dataframe
            vdf = self._get_vehicles_df()

            # adjust positions (of all vehicles)
            vdf = update_position(vdf, self._step_length)

            # convert dataframe back to dict of vehicles
            self._write_back_vehicles_df(vdf)

            # get arrived vehicles
            arrived_vehicles = vdf[
                (vdf.position >= vdf.arrival_position)
            ].index.values

            # remove arrived vehicles from dataframe
            vdf = vdf.drop(arrived_vehicles)

            # do collision check (for all vehicles)
            # without arrived vehicles
            if self._collisions:
                self._check_collisions(vdf)

            # remove arrived vehicles from dict and do finish
            self._remove_arrived_vehicles(arrived_vehicles)

            # make sure that everything is correct
            assert(list(vdf.index).sort() == list(self._vehicles.keys()).sort())

            del vdf
            # END VECTORIZATION PART

            # a new step begins
            self._step += self._step_length
            progress_bar.update(self._step_length)
            if self._gui:
                import traci
                traci.simulationStep(self.step)
                assert(traci.simulation.getTime() == float(self.step))

        # We reach this point only by setting self._running to False
        # which is only done by calling self.stop()
        # which already calls self.finish().
        # Hence, we do not have to do anything anymore.
        return self.step

    def _statistics(self):
        """Calculates some period statistics."""

        self._avg_number_vehicles = (
            (self._values_in_avg_number_vehicles * self._avg_number_vehicles + len(self._vehicles)) /
            (self._values_in_avg_number_vehicles + 1)
        )

    def _get_vehicles_df(self) -> pd.DataFrame:
        """Returns a pandas dataframe from the internal data structure."""

        if not self._vehicles:
            return pd.DataFrame()
        return (
            pd.DataFrame([dict(
                **v.__dict__,
                length=v.length
            )
                for v in self._vehicles.values()
            ])
            .rename(columns=lambda x: re.sub('^_', '', x))
            .set_index('vid')
        )

    def _write_back_vehicles_df(self, vdf: pd.DataFrame):
        """
        Writes back the vehicle updates from a given pandas dataframe to the internal data structure.

        Parameters
        ----------
        vdf : pandas.DataFrame
            The dataframe containing the vehicles as rows
            index: vid
            columns: [position, length, lane, ..]
        """

        # make sure that everything is correct
        assert(list(vdf.index).sort() == list(self._vehicles.keys()).sort())

        for row in vdf.itertuples():
            # update all fields within the data that we updated with pandas
            vehicle = self._vehicles[row.Index]
            vehicle._position = row.position

    def stop(self, msg: str):
        """
        Stops the simulation with the given message.

        Parameters
        ----------
        msg : str
            The message to show after stopping the simulation
        """

        self._running = False
        print(f"\n{msg}")
        self.finish()

    def __str__(self) -> str:
        """Returns a str representation of a simulator instance."""

        sim_dict = self.__dict__.copy()
        sim_dict.pop('_vehicles')
        sim_dict.pop('_infrastructures')
        sim_dict.update({'current_number_of_vehicles': len(self._vehicles)})
        sim_dict.update({'current_number_of_infrastructures': len(self._infrastructures)})
        return str(sim_dict)

    def finish(self):
        """Cleans up the simulation."""

        if self._running:
            LOG.warning("Finish called during simulation!")
            return

        # write some general information about the simulation
        with open(f'{self._result_base_filename}_general.out', 'a') as f:
            f.write(f"simulation end: {time.asctime(time.localtime(time.time()))}\n")
            f.write(f"average number of vehicles: {self._avg_number_vehicles}\n")

        # call finish on infrastructures
        for infrastructure in self._infrastructures.values():
            infrastructure.finish()
            if self._gui:
                import traci
                traci.polygon.remove(str(infrastructure.iid))

        # call finish on remaining vehicles?
        for vehicle in self._vehicles.values():
            # we do not want to write statistics for not finished vehicles
            # therefore, we do not call finish here
            if self._gui:
                import traci
                traci.vehicle.remove(str(vehicle.vid), 2)
            # remove from vehicles
            del vehicle

        if self._gui:
            import traci
            traci.close(False)
