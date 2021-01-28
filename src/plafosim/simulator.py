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
import os
import pandas as pd
import random
import re
import sys
import time

from collections import namedtuple
from math import copysign
from tqdm import tqdm

from .infrastructure import Infrastructure
from .platooning_vehicle import PlatooningVehicle
from .platoon_role import PlatoonRole
from .util import get_crashed_vehicles
from .util import update_position
from .vehicle import Vehicle
from .vehicle_type import VehicleType

LOG = logging.getLogger(__name__)

# assumptions
# you just reach your arrival_position
# position is in the middle of the front bumper
# a vehicle ends at position + length
# crash detection does not work with steps greater than 1

# vehicle properties
length = 4  # m # TODO make parameter
max_speed = 55  # m/s # TODO make parameter
max_acceleration = 2.5  # m/s # TODO make parameter
max_deceleration = 15  # m/s # TODO make parameter
min_gap = 2.5  # m # TODO make parameter
desired_headway_time = 1.0  # s # TODO make parameter
vtype = VehicleType("car", length, max_speed, max_acceleration, max_deceleration, min_gap, desired_headway_time)  # TODO support multiple vtypes
TV = namedtuple('TV', ['position', 'rear_position', 'lane'])


class Simulator:
    """A collection of parameters and information of the simulator"""

    def __init__(
            self,
            road_length: int = 100 * 1000,
            number_of_lanes: int = 3,
            depart_interval: int = 1000,
            arrival_interval: int = 1000,
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
            record_vehicle_trips: bool = True,
            record_vehicle_emissions: bool = True,
            record_vehicle_traces: bool = False,
            record_vehicle_changes: bool = False,
            record_emission_traces: bool = False,
            record_platoon_trips: bool = True,
            record_platoon_maneuvers: bool = True,
            record_platoon_formation: bool = True,
            record_platoon_traces: bool = False,
            record_platoon_changes: bool = False,
            record_prefilled: bool = False):
        """Initialize a simulator instance"""

        # TODO add custom filter that prepends the log entry with the step time
        logging.basicConfig(level=log_level, format="%(levelname)s [%(name)s]: %(message)s")

        # road network properties
        self._road_length = road_length  # the length of the road
        self._number_of_lanes = number_of_lanes  # the number of lanes
        self._depart_interval = depart_interval  # the distance between departure positions
        self._arrival_interval = arrival_interval  # the distance between arrival positions

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
                sys.exit("ERROR: Vehicles can not have random arrival posiition when starting as one platoon!")

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
        self._record_prefilled = record_prefilled  # whether to record results for pre-filled vehicles

        # TODO log generation parameters
        self._pre_fill = pre_fill
        if pre_fill:
            self._generate_vehicles()

        self._generate_infrastructures(number_of_infrastructures)

    @property
    def road_length(self) -> int:
        return self._road_length

    @property
    def number_of_lanes(self) -> int:
        return self._number_of_lanes

    @property
    def step_length(self) -> int:
        return self._step_length

    @property
    def step(self) -> int:
        return self._step

    def _call_vehicle_actions(self):
        """Trigger actions of all vehicles"""

        for vehicle in self._vehicles.values():
            vehicle.action(self.step)

    def _call_infrastructure_actions(self):
        """Trigger actions of all instrastructures"""

        for instrastructure in self._infrastructures.values():
            instrastructure.action(self.step)

    def _get_predecessor(self, vehicle: Vehicle, lane: int = -1) -> Vehicle:
        if lane == -1:
            # implicitly search on current lane of vehicle
            lane = vehicle.lane
        predecessor = None  # there is no predecessor so far
        candiates = [
            v for v in self._vehicles.values() if
            v is not vehicle and  # not this vehicle
            v.lane == lane and  # correct lane
            v.position >= vehicle.position  # in front this vehicle
        ]
        for other_vehicle in candiates:
            # we do not check for collisions here because this method is also called within an update step
            if predecessor is None or other_vehicle.rear_position < predecessor.rear_position:
                # the current vehicle is closer to us than the previous predecessor
                predecessor = other_vehicle
        return predecessor

    def _get_successor(self, vehicle: Vehicle, lane: int = -1) -> Vehicle:
        if lane == -1:
            # implicitly search on current lane of vehicle
            lane = vehicle.lane
        successor = None  # there is no successor so far
        candiates = [
            v for v in self._vehicles.values() if
            v is not vehicle and  # not this vehicle
            v.lane == lane and  # correct lane
            v.position <= vehicle.position  # behind this vehicle
        ]
        for other_vehicle in candiates:
            # we do not check for collisions here because this method is also called within an update step
            if successor is None or other_vehicle.position > successor.position:
                # the current vehicle is closer to us than the previous successor
                successor = other_vehicle
        return successor

    def _get_predecessor_rear_position(self, vehicle: Vehicle, lane: int = -1) -> float:
        p = self._get_predecessor(vehicle, lane)
        if p is None:
            return -1
        else:
            return p.rear_position

    def _get_predecessor_speed(self, vehicle: Vehicle, lane: int = -1) -> float:
        p = self._get_predecessor(vehicle, lane)
        if p is None:
            return -1
        else:
            return p.speed

    def is_lane_change_safe(self, vehicle: Vehicle, target_lane: int) -> bool:
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
        source_lane = vehicle.lane
        if source_lane == target_lane:
            return True
        LOG.debug(f"{vehicle.vid} wants to change from lane {source_lane} to lane {target_lane} ({reason})")

        lane_diff = target_lane - source_lane
        if abs(lane_diff) > 1:
            LOG.warning(f"{vehicle.vid} only change to adjacent lane!")
            old_target_lane = target_lane
            target_lane = source_lane + copysign(1, lane_diff)
            LOG.warning(f"Adjusted target lane of {vehicle.vid} to {target_lane} (from {old_target_lane})")

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
                        LOG.info(f"{member.vid} is switching lanes: {source_lane} -> {target_lane} ({reason})")

                        # switch to adjacent lane
                        member._lane = target_lane

                        if self._record_platoon_changes:
                            # log lane change
                            with open(self._result_base_filename + '_platoon_changes.csv', 'a') as f:
                                f.write(
                                    f"{self.step},"
                                    f"{member.vid},"
                                    f"{member.position},"
                                    f"{source_lane},"
                                    f"{target_lane},"
                                    f"{member.speed},"
                                    f"{reason},"
                                    "\n"
                                )

                    return abs(lane_diff) <= 1
                LOG.debug(f"{vehicle.vid}'s lane change is not safe")
                return False

        # we are just a regular vehicle or we are not (yet) in a platoon

        # check adjacent lane is free
        if self.is_lane_change_safe(vehicle, target_lane):
            LOG.info(f"{vehicle.vid} is switching lanes: {source_lane} -> {target_lane} ({reason})")

            # switch to adjacent lane
            vehicle._lane = target_lane

            if self._record_vehicle_changes:
                # log lane change
                with open(self._result_base_filename + '_vehicle_changes.csv', 'a') as f:
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

    # kraus - multi lane traffic
    # lane-change
    # congested = (v_safe < v_thresh) and (v^0_safe < v_thresh)
    # favorable(right->left) = (v_safe < v_max) and (not congested)
    # favorable(left->right) = (v_safe >= v_max) and (v^0_safe >= v_max)
    # if ((favorable(i->j) or (rand < p_change)) and safe(i->j)) then change(i->j)
    # for vehicles on the right lane:
    # if (v > v^0_safe) and (not congested) then v <- v^0_safe
    def _change_lanes(self):
        """Do lane changes for all vehicles"""

        for vehicle in self._vehicles.values():
            self._adjust_lane(vehicle)

    def _adjust_lane(self, vehicle: Vehicle):
        # decide upon and perform a lane change for this vehicle
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
        """Do speed adjustments for all vehicles"""

        for vehicle in sorted(self._vehicles.values(), key=lambda x: x.position, reverse=True):
            self._adjust_speed(vehicle)

    def _adjust_speed(self, vehicle: Vehicle):
        LOG.debug(f"{vehicle.vid}'s current speed {vehicle.speed}")
        predecessor = self._get_predecessor(vehicle)
        new_speed = vehicle.new_speed(predecessor.speed if predecessor else -1, predecessor.rear_position if predecessor else -1)
        vehicle._acceleration = new_speed - vehicle.speed
        LOG.debug(f"{vehicle.vid}'s current acceleration: {vehicle.acceleration}")
        vehicle._speed = new_speed

    def _remove_arrived_vehicles(self, vdf: pd.DataFrame):
        # find arrived vehicles
        arrived_vehicles = vdf[
            (vdf.position >= vdf.arrival_position)
        ].index.values

        for vid in arrived_vehicles:
            # write back the position for this vehicle in order to correctly record trip statistics
            self._vehicles[vid]._position = vdf.loc[vid, 'position']
            # call finish on arrived vehicle
            self._vehicles[vid].finish()
            # remove arrived vehicle from gui
            if self._gui:
                import traci
                traci.vehicle.remove(str(vid), 2)
            # remove from vehicles
            del self._vehicles[vid]

        # remove arrived vehicles from dataframe
        vdf = vdf.drop(arrived_vehicles)

        return vdf

    @staticmethod
    def _check_collisions(vdf: pd.DataFrame):
        """Do collision checks for all vehicles"""

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
        TV(position, rear_position, lane)
        """
        assert(vehicle1 is not vehicle2)
        assert(vehicle1.lane == vehicle2.lane)
        return min(vehicle1.position, vehicle2.position) - max(vehicle1.rear_position, vehicle2.rear_position) >= 0

    def _generate_vehicles(self):

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
                depart_position = (number_of_vehicles - vid) * (length + self._cacc_spacing)
                depart_lane = 0
            else:
                # assume we have a collision to check at least once
                collision = True
                while collision:
                    collision = False
                    # actual calculation of position and lane
                    # always use random position for pre-filled vehicle
                    # we do not consider depart interval here since this is supposed to be a snapshot from an ealier point of simulation
                    depart_position = random.randrange(length, self.road_length, round(length + min_gap))
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
                        tv = TV(depart_position, depart_position - vtype.length, depart_lane)
                        otv = TV(other_vehicle.position, other_vehicle.rear_position, other_vehicle.lane)
                        collision = collision or self.has_collision(tv, otv)

            desired_speed = self._get_desired_speed()

            # always use desired speed for pre-fill vehicles
            depart_speed = desired_speed

            # TODO remove duplicated code
            if self._random_arrival_position:
                # we cannot use the minimum trip time here since, the pre-generation is supposed to produce a snapshot of a realistic simulation
                # but we can assume that a vehicle has to drive a least to the next exit ramp
                min_arrival = min(depart_position + self._arrival_interval, self._road_length)
                min_arrival_ramp = min_arrival + (self._arrival_interval - min_arrival) % self._arrival_interval
                assert(min_arrival_ramp >= 0)
                assert(min_arrival_ramp <= self._road_length)
                if min_arrival_ramp == self._road_length:
                    # exit at end
                    arrival_position = self._road_length
                else:
                    arrival_position = random.randrange(min_arrival_ramp, self._road_length, self._arrival_interval)
                    assert(arrival_position > depart_position)
            else:
                arrival_position = self._road_length

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

            LOG.debug(f"Generated vehicle {vid}")

    def _get_desired_speed(self) -> float:
        if self._random_desired_speed:
            # normal distribution
            desired_speed = self._desired_speed * random.normalvariate(1.0, self._speed_variation)
            desired_speed = max(desired_speed, self._min_desired_speed)
            desired_speed = min(desired_speed, self._max_desired_speed)
        else:
            desired_speed = self._desired_speed

        return desired_speed

    def _spawn_vehicle(self):

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
        LOG.debug(f"Spawning vehicle {vid}")

        if self._random_depart_lane:
            depart_lane = random.randrange(0, self.number_of_lanes, 1)
        else:
            depart_lane = 0

        # TODO remove duplicated code
        if self._random_depart_position:
            max_depart = self._road_length - self._minimum_trip_length
            max_depart_ramp = max_depart - (self._depart_interval + max_depart) % self._depart_interval
            assert(max_depart_ramp <= self._road_length)
            assert(max_depart_ramp >= 0)
            if max_depart_ramp == 0:
                # start at beginning
                depart_position = length  # equql to departPos="base" in SUMO
            else:
                depart_position = random.randrange(0, max_depart_ramp, self._depart_interval)  # TODO length (base)
        else:
            depart_position = length  # equal to departPos="base" in SUMO

        # TODO remove duplicated code
        # check whether the can actually be inserted
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
                tv = TV(depart_position, depart_position - vtype.length, depart_lane)
                otv = TV(other_vehicle.position, other_vehicle.rear_position, other_vehicle.lane)
                collision = collision or self.has_collision(tv, otv)

            if collision:
                # can we avoid the collision by switching the departure lane?
                if depart_lane == self.number_of_lanes - 1:
                    # reached maximum number of lanes already
                    # TODO delay insertion of vehicle
                    sys.exit(f"ERROR: Could not further increase depart lane ({depart_lane}) for vehicle {vid}! You might want to reduce the number of vehicles to reduce the traffic. Delaying insertion of vehicles is not (yet) implemtend!")
                depart_lane = depart_lane + 1
                LOG.warning(f"Increased depart lane for {vid} to avoid a collision (now lane {depart_lane})")
                # we need to check again

        desired_speed = self._get_desired_speed()

        if self._random_depart_speed:
            depart_speed = random.randrange(0, self._desired_speed, 1)
        else:
            depart_speed = 0

        if self._depart_desired:
            depart_speed = desired_speed

        # TODO remove duplicated code
        if self._random_arrival_position:
            min_arrival = depart_position + self._minimum_trip_length
            min_arrival_ramp = min_arrival + (self._arrival_interval - min_arrival) % self._arrival_interval
            assert(min_arrival_ramp >= 0)
            assert(min_arrival_ramp <= self._road_length)
            if min_arrival_ramp == self._road_length:
                # exit at end
                arrival_position = self._road_length
            else:
                arrival_position = random.randrange(min_arrival_ramp, self._road_length, self._arrival_interval)
        else:
            arrival_position = self._road_length

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
        """Generate infrastructures for the simulation"""

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

            LOG.info(f"Generated infrastructure {infrastructure}")

            last_infrastructure_id = iid

    def _initialize_result_recording(self):
        # write some general information about the simulation
        with open(self._result_base_filename + '_general.out', 'w') as f:
            f.write("simulation start: " + time.asctime(time.localtime(time.time())) + '\n')
            f.write("parameters" + str(self) + '\n')

        if self._record_vehicle_trips:
            # create output file for vehicle trips
            with open(self._result_base_filename + '_vehicle_trips.csv', 'w') as f:
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
                    "estimatedTravelTime,"
                    "travelTimeRatio,"
                    "avgDrivingSpeed,"
                    "avgDeviationDesiredSpeed,"
                    "\n"
                )

        if self._record_vehicle_emissions:
            # create output file for vehicle emissions
            with open(self._result_base_filename + '_vehicle_emissions.csv', 'w') as f:
                f.write(
                    "id,"
                    "CO,"
                    "CO2,"
                    "HC,"
                    "PMx,"
                    "NOx,"
                    "fuel,"
                    "\n"
                )

        if self._record_vehicle_traces:
            # create output file for vehicle traces
            with open(self._result_base_filename + '_vehicle_traces.csv', 'w') as f:
                f.write(
                    "step,"
                    "id,"
                    "position,"
                    "lane,"
                    "speed,"
                    "duration,"
                    "routeLength,"
                    "\n"
                )

        if self._record_vehicle_changes:
            # create output file for vehicle lane changes
            with open(self._result_base_filename + '_vehicle_changes.csv', 'w') as f:
                f.write(
                    "step,"
                    "id,"
                    "position,"
                    "from,"
                    "to,"
                    "speed,"
                    "reason,"
                    "\n"
                )

        if self._record_emission_traces:
            # create output file for emission traces
            with open(self._result_base_filename + '_emission_traces.csv', 'w') as f:
                f.write(
                    "step,"
                    "id,"
                    "CO,"
                    "CO2,"
                    "HC,"
                    "PMx,"
                    "NOx,"
                    "fuel,"
                    "\n"
                )

        if self._record_platoon_trips:
            # create output file for platoon trips
            with open(self._result_base_filename + '_platoon_trips.csv', 'w') as f:
                f.write(
                    "id,"
                    "timeInPlatoon,"
                    "distanceInPlatoon,"
                    "platoonTimeRatio,"
                    "platoonDistanceRatio,"
                    "numberOfPlatoons,"
                    "timeUntilFirstPlatoon,"
                    "distanceUntilFirstPlatoon,"
                    "\n"
                )

        if self._record_platoon_maneuvers:
            # create output file for platoon maneuvers
            with open(self._result_base_filename + '_platoon_maneuvers.csv', 'w') as f:
                f.write(
                    "id,"
                    "joins_attempted,"
                    "joins_succesful,"
                    "joins_aborted,"
                    "leaves_attempted,"
                    "leaves_successful,"
                    "leaves_aborted,"
                    "\n"
                )

        if self._record_platoon_formation:
            # create output file for platoon formation
            with open(self._result_base_filename + '_platoon_formation.csv', 'w') as f:
                f.write(
                    "id,"
                    "candidates_found,"
                    "candidates_filtered,"
                    "\n"
                )

        if self._record_platoon_traces:
            # create output file for platoon traces
            with open(self._result_base_filename + '_platoon_traces.csv', 'w') as f:
                f.write(
                    "step,"
                    "id,"
                    "platoon,"
                    "leader,"
                    "position,"
                    "rear_position,"
                    "lane,speed,size,"
                    "length,"
                    "desired_speed,"
                    "platoon_role,"
                    "platoon_position,"
                    "\n"
                )

        if self._record_platoon_changes:
            # create output file for platoon lane changes
            with open(self._result_base_filename + '_platoon_changes.csv', 'w') as f:
                f.write(
                    "step,"
                    "id,"
                    "position,"
                    "from,"
                    "to,"
                    "speed,"
                    "reason,"
                    "\n"
                )

    def _initialize_gui(self):
        sumoBinary = os.path.join(os.environ['SUMO_HOME'], 'bin/sumo-gui')
        sumoCmd = [sumoBinary, "-Q", "-c", "sumocfg/freeway.sumo.cfg", '--collision.action', 'warn']

        import traci
        traci.start(sumoCmd)

        # draw infrastructures
        for infrastructure in self._infrastructures.values():
            # add infrastructure
            if (str(infrastructure.iid)) not in traci.polygon.getIDList():
                y = 230
                width = 10
                color = (255, 126, 0)
                traci.polygon.add(str(infrastructure.iid), [(infrastructure.position, y), (infrastructure.position + width, y), (infrastructure.position + width, y + width), (infrastructure.position, y + width)], color, fill=True)
        # draw pre-filled vehicles
        # save internal state of random number generator
        state = random.getstate()
        for vehicle in self._vehicles.values():
            self._add_gui_vehicle(vehicle)
        # restore internal state of random number generator to not influence the determinsim of the simulation
        random.setstate(state)

    def _update_gui(self):
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
        import traci
        if str(vehicle.vid) not in traci.vehicle.getIDList():
            traci.vehicle.add(str(vehicle.vid), 'route', departPos=str(vehicle.position), departSpeed=str(vehicle.speed), departLane=str(vehicle.lane), typeID='vehicle')
            # save internal state of random number generator
            state = random.getstate()
            color = (random.randrange(0, 255, 1), random.randrange(0, 255, 1), random.randrange(0, 255, 1))
            traci.vehicle.setColor(str(vehicle.vid), color)
            vehicle._color = color
            # restore internal state of random number generator to not influence the determinsim of the simulation
            if not (self._pre_fill and self._step == 0):
                # By disabling the reset of the random state when using pre-fill and in the first time step,
                # we achieve a (deterministic) random color for all pre-filled vehicles.
                # Otherweise, when resetting the state step 0, all pre-filled vehicles did not have a random color
                # (instead they had the same), since the RNG did not pick any other numbers since the last color pick.
                random.setstate(state)
            traci.vehicle.setSpeedMode(str(vehicle.vid), 0)
            traci.vehicle.setLaneChangeMode(str(vehicle.vid), 0)
            # track vehicle
            if vehicle.vid == self._gui_track_vehicle:
                traci.gui.trackVehicle("View #0", str(vehicle.vid))
                traci.gui.setZoom("View #0", 1000000)

    def run(self):
        """Run the simulation with the specified parameters"""

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

            if self._actions:
                # call regular actions on vehicles
                self._call_vehicle_actions()
                # call regular actions on infrastructure
                self._call_infrastructure_actions()

            # perform lane changes (for all vehicles)
            if self._lane_changes:
                self._change_lanes()

            # adjust speed (of all vehicles)
            self._adjust_speeds()

            # BEGIN VECTORIZATION PART - CONVERT LIST OF OBJECTS TO DATAFRAME
            vdf = self._get_vehicles_df()

            # adjust positions (of all vehicles)
            vdf = update_position(vdf, self._step_length)

            # remove arrived vehicles
            vdf = self._remove_arrived_vehicles(vdf)

            # do collision check (for all vehicles)
            if self._collisions:
                self._check_collisions(vdf)

            assert(list(vdf.index).sort() == list(self._vehicles.keys()).sort())

            # CONVERT DATAFRAME BACK TO LIST OF OBJECTS
            self._write_back_vehicles_df(vdf)

            del vdf
            # END VECTORIZATION PART

            # a new step begins
            self._step += self._step_length
            progress_bar.update(self._step_length)
            if self._gui:
                import traci
                traci.simulationStep(self.step)
                assert(traci.simulation.getTime() == float(self.step))

        return self.step

    def _get_vehicles_df(self) -> pd.DataFrame:
        if not self._vehicles:
            return pd.DataFrame()
        return (
            pd.DataFrame([
                dict(
                    **v.__dict__,
                    length=v.length
                )
                for v in self._vehicles.values()
            ])
            .rename(columns=lambda x: re.sub('^_', '', x))
            .set_index('vid')
        )

    def _write_back_vehicles_df(self, vdf: pd.DataFrame):
        for row in vdf.itertuples():
            # update all fields within the data that we updated with pandas
            vehicle = self._vehicles[row.Index]
            vehicle._position = row.position

    def stop(self, msg: str):
        """Stop the simulation with the given message"""

        self._running = False
        print(f"\n{msg}")
        self.finish()

    def __str__(self) -> str:
        """Return a nice string representation of a simulator instance"""

        sim_dict = self.__dict__.copy()
        sim_dict.pop('_vehicles')
        sim_dict.pop('_infrastructures')
        sim_dict.update({'current_number_of_vehicles': len(self._vehicles)})
        sim_dict.update({'current_number_of_infrastructures': len(self._infrastructures)})
        return str(sim_dict)

    def finish(self):
        """Clean up the simulation"""

        if self._running:
            LOG.warning("Finish called during simulation!")
            return

        # write some general information about the simulation
        with open(self._result_base_filename + '_general.out', 'a') as f:
            f.write("simulation end: " + time.asctime(time.localtime(time.time())) + '\n')

        if self._gui:
            import traci
            # remove all infrastructures
            for iid in traci.polygon.getIDList():
                traci.polygon.remove(iid)
            # remove all vehicles
            for vid in traci.vehicle.getIDList():
                traci.vehicle.remove(vid, 2)
            traci.close(False)
