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
import random
import os
import sys
import time

from math import copysign
from tqdm import tqdm

from .vehicle_type import VehicleType
from .vehicle import Vehicle
from .infrastructure import Infrastructure
from .platooning_vehicle import PlatooningVehicle, CF_Mode
from .platoon_role import PlatoonRole
from .platoon import Platoon

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
min_gap = 0  # m # TODO make parameter
vtype = VehicleType("car", length, max_speed, max_acceleration, max_deceleration, min_gap)  # TODO support multiple vtypes


class Simulator:
    """A collection of parameters and information of the simulator"""

    def __init__(
            self,
            road_length: int = 100 * 1000,
            number_of_lanes: int = 4,
            depart_interval: int = 1000,
            arrival_interval: int = 1000,
            pre_fill: bool = False,
            number_of_vehicles: int = 100,
            vehicle_density: int = 0,
            max_speed: float = 55,
            acc_headway_time: float = 1.0,
            cacc_spacing: float = 5.0,
            collisions: bool = True,
            lane_changes: bool = True,
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
            communication_range: float = 1000,
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
            random_seed: int = -1,
            log_level: int = 'warn',
            gui: bool = False,
            gui_delay: int = 0,
            gui_track_vehicle: int = -1,
            result_base_filename: str = 'results',
            record_vehicle_trips: bool = True,
            record_vehicle_emissions: bool = True,
            record_vehicle_traces: bool = False,
            record_vehicle_changes: bool = False,
            record_emission_traces: bool = False,
            record_platoon_trips: bool = True,
            record_platoon_traces: bool = False,
            record_platoon_changes: bool = False):
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
        self._cacc_spacing = cacc_spacing  # the constant spacing for CACC
        self._collisions = collisions  # whether to check for collisions
        self._lane_changes = lane_changes  # whether to enable lane changes
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
            sys.exit("random-depart-position is only possible in conjunction with depart-desired!")
        self._depart_flow = depart_flow  # whether to spawn vehicles in a continuous flow
        self._depart_method = depart_method  # the departure method to use
        self._depart_time_interval = depart_time_interval  # the interval between two vehicle departures
        self._depart_probability = depart_probability  # the departure probability
        self._depart_rate = depart_rate  # the departure rate
        self._depart_fixed_time = depart_fixed_time  # the fixed departure time for all vehicles
        self._random_arrival_position = random_arrival_position  # whether to use random arrival positions
        self._minimum_trip_length = minimum_trip_length  # the minimum trip length

        # communication properties
        self._communication_range = communication_range  # the maximum communication range between two vehicles
        if communication_range == -1:
            self._communication_range = road_length
        if self._communication_range <= 0:
            sys.exit("Communication range has to be > 0!")

        # platoon properties
        self._start_as_platoon = start_as_platoon  # whether vehicles start as one platoon
        if start_as_platoon:
            if penetration_rate < 1.0:
                sys.exit("The penetration rate cannot be smaller than 1.0 when starting as one platoon!")
            if formation_algorithm is not None:
                sys.exit("A formation algorithm cannot be used when all starting as one platoon!")
            if random_depart_position:
                sys.exit("Vehicles can not have random departure positions when starting as one platoon!")
            if random_depart_lane:
                sys.exit("Vehicles can not have random departure lanes when starting as one platoon!")

        self._formation_algorithm = formation_algorithm  # the formation algorithm to use
        if formation_strategy == "centralized" and number_of_infrastructures == 0:
            sys.exit("When using a centralized strategy at least 1 infrastructure is needed!")
        self._formation_strategy = formation_strategy  # the formation strategy to use
        self._formation_centralized_kind = formation_centralized_kind  # the kind of the centralized formation

        # formation properties
        # TODO find a different solution for algorithm specific parameters
        self._execution_interval = execution_interval  # the interval between two iterations of a formation algorithm
        if execution_interval <= 0:
            sys.exit("Execution interval has to be at least 1 second!")
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
        if random_seed >= 0:
            self._random_seed = random_seed  # the random.seed to use for the RNG
            LOG.info(f"Using random seed {random_seed}")
            random.seed(random_seed)

        # gui properties
        self._gui = gui  # whether to show a live sumo-gui
        if gui:
            if 'SUMO_HOME' not in os.environ:
                sys.exit("please declare environment variable 'SUMO_HOME'")
            tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
            sys.path.append(tools)

        self._gui_delay = gui_delay  # the delay in every simulation step for the gui
        self._gui_track_vehicle = gui_track_vehicle  # the id of a vehicle to track in the gui

        # result recording properties
        self._result_base_filename = result_base_filename  # the base filename of the result files
        self._record_vehicle_trips = record_vehicle_trips  # whether to record vehicles trips
        self._record_vehicle_emissions = record_vehicle_emissions  # whether to record vehicle emissions
        self._record_vehicle_traces = record_vehicle_traces  # whether to record vehicle traces
        self._record_vehicle_changes = record_vehicle_changes  # whether to record vehicle lane changes
        self._record_emission_traces = record_emission_traces  # whether to record emission traces
        self._record_platoon_trips = record_platoon_trips  # whether to record platoon trips
        self._record_platoon_traces = record_platoon_traces  # whether to record platoon traces
        self._record_platoon_changes = record_platoon_changes  # whether to record platoon lane changes

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

    @staticmethod
    def speed2distance(speed: float, time_interval: float = 1.0) -> float:
        return speed * time_interval

    @staticmethod
    def distance2speed(distance: float, time_interval: float = 1.0) -> float:
        return distance / time_interval

    @staticmethod
    def acceleration2speed(acceleration: float, time_interval: float = 1.0) -> float:
        return acceleration * time_interval

    @staticmethod
    def speed2acceleration(speed_from: float, speed_to: float, time_interval: float = 1.0) -> float:
        return (speed_to - speed_from) / time_interval

    def _get_predecessor(self, vehicle: Vehicle, lane: int = -1) -> Vehicle:
        if lane == -1:
            # implicitly search on current lane of vehicle
            lane = vehicle.lane
        predecessor = None  # there is no predecessor so far
        for other_vehicle in self._vehicles.values():
            if other_vehicle is vehicle:
                # skip the vehicle
                continue
            if other_vehicle.depart_time > self._step:
                # vehicle did not start yet
                continue
            if other_vehicle.lane != lane:
                # skip other lane
                continue
            if other_vehicle.position < vehicle.position:
                # vehicle is not in front of us
                # this means we consider all vehicles that are at least as far as we are
                continue
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
        for other_vehicle in self._vehicles.values():
            if other_vehicle is vehicle:
                # skip the vehicle
                continue
            if other_vehicle.depart_time > self._step:
                # vehicle did not start yet
                continue
            if other_vehicle.lane != lane:
                # skip other lane
                continue
            if other_vehicle.position > vehicle.position:
                # vehicle is not behind us
                continue
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
            if vehicle.speed > vehicle._safe_speed(p.speed, gap_to_predecessor_on_target_lane, vehicle.desired_gap, vehicle.vehicle_type.min_gap):
                LOG.debug(f"{vehicle.vid}'s lane change is not safe because of its predecessor")
                return False

        # check successor on target lane
        s = self._get_successor(vehicle, target_lane)
        if s is not None:
            gap_to_successor_on_target_lane = vehicle.rear_position - s.position
            if gap_to_successor_on_target_lane < 0:
                LOG.debug(f"{vehicle.vid}'s lane change is not safe because of its successor")
                return False
            if s.speed > s._safe_speed(vehicle.speed, gap_to_successor_on_target_lane):
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
            LOG.warn(f"{vehicle.vid} only change to adjacent lane!")
            old_target_lane = target_lane
            target_lane = source_lane + copysign(1, lane_diff)
            LOG.warn(f"Adjusted target lane of {vehicle.vid} to {target_lane} (from {old_target_lane})")

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
                                f.write(f"{self.step},{member.vid},{member.position},{source_lane},{target_lane},{member.speed},{reason}\n")

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
                    f.write(f"{self.step},{vehicle.vid},{vehicle.position},{source_lane},{target_lane},{vehicle.speed},{reason}\n")

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
        if vehicle.depart_time > self._step:
            # vehicle did not start yet
            return

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

        for vehicle in self._vehicles.values():
            self._adjust_speed(vehicle)

    def _adjust_speed(self, vehicle: Vehicle):
        if vehicle.depart_time > self._step:
            # vehicle did not start yet
            return

        LOG.debug(f"{vehicle.vid}'s current speed {vehicle.speed}")
        predecessor = self._get_predecessor(vehicle)
        new_speed = vehicle.new_speed(predecessor.speed if predecessor else -1, predecessor.rear_position if predecessor else -1)
        vehicle._acceleration = new_speed - vehicle.speed
        LOG.debug(f"{vehicle.vid}'s current acceleration: {vehicle.acceleration}")
        vehicle._speed = new_speed

    # krauss - single lane traffic
    # adjust position (move)
    # x(t + step_size) = x(t) + v(t)*step_size
    def _move_vehicles(self):
        """Do position updates for all vehicles"""

        for vehicle in sorted(self._vehicles.values(), key=lambda x: x.position, reverse=True):
            self._move_vehicle(vehicle)

    def _move_vehicle(self, vehicle: Vehicle):
        if vehicle.depart_time > self._step:
            # vehicle did not start yet
            return
        # increase position according to speed
        position_difference = self.speed2distance(vehicle.speed, self._step_length)
        # TODO add emissions/fuel statistics
        # arrival_position reached?
        if vehicle.position + position_difference >= vehicle.arrival_position:
            # TODO use proper method
            vehicle._position = vehicle.arrival_position
            vehicle.finish()
            if self._gui:
                import traci
                traci.vehicle.remove(str(vehicle.vid), 2)
            del self._vehicles[vehicle.vid]
            return
        else:
            # TODO use proper method
            vehicle._position += position_difference
            LOG.debug(f"{vehicle.vid}'s new position {vehicle.position}-{vehicle.rear_position}")

    def _check_collisions(self):
        """Do collision checks for all vehicles"""

        for vehicle in self._vehicles.values():
            self._check_collision(vehicle)

    def _check_collision(self, vehicle: Vehicle):
        if vehicle.depart_time > self._step:
            # vehicle did not start yet
            return
        # check for crashes of this vehicle with any other vehicle
        for other_vehicle in self._vehicles.values():
            if vehicle is other_vehicle:
                # we do not need to compare us to ourselves
                continue
            if vehicle.lane != other_vehicle.lane:
                # we do not care about other lanes
                continue
            if other_vehicle.depart_time > self._step:
                # other vehicle did not start yet
                continue
            if self.has_collision(vehicle, other_vehicle):
                sys.exit(f"collision between {vehicle.vid} ({vehicle.position}-{vehicle.rear_position},{vehicle.lane}) and {other_vehicle.vid} ({other_vehicle.position}-{other_vehicle.rear_position},{other_vehicle.lane})")

    @staticmethod
    def has_collision(vehicle1: Vehicle, vehicle2: Vehicle) -> bool:
        assert(vehicle1 is not vehicle2)
        assert(vehicle1.lane == vehicle2.lane)
        return min(vehicle1.position, vehicle2.position) - max(vehicle1.rear_position, vehicle2.rear_position) >= 0

    # TODO remove duplicated code
    def _generate_vehicles(self):

        if self._vehicle_density > 0:
            number_of_vehicles = round(self._vehicle_density * int(self._road_length / 1000) * self._number_of_lanes)
        else:
            number_of_vehicles = self._number_of_vehicles

        LOG.info(f"Pre-filling the road network with {number_of_vehicles} vehicles")

        for num in tqdm(range(0, number_of_vehicles), desc="Generated vehicles"):

            vid = self._last_vehicle_id + 1
            depart_time = -1  # since this is a pre-filled vehicle, we cannot say when it departed

            collision = True
            while collision:
                # actual calculation of position and lane
                # always use random position for pre-filled vehicle
                # we do not consider depart interval here since this is supposed to be a snapshot from an ealier point of simulation
                depart_position = random.randrange(length, self.road_length, length + min_gap)
                # always use random lane for pre-filled vehicle
                depart_lane = random.randrange(0, self.number_of_lanes, 1)

                LOG.debug(f"Generated random depart position ({depart_position},{depart_lane}) for vehicle {vid}")

                # avoid a collision with an existing vehicle
                collision = False
                for other_vehicle in self._vehicles.values():
                    if other_vehicle.lane != depart_lane:
                        # we do not care about other lanes
                        continue
                    # TODO HACK for using collision check
                    vehicle = Vehicle(self, vid, vtype, depart_position, -1, -1, depart_lane, -1, depart_time, -1)
                    collision = collision or self.has_collision(vehicle, other_vehicle)

            if self._random_desired_speed:
                # normal distribution
                desired_speed = self._desired_speed * random.normalvariate(1.0, self._speed_variation)
                desired_speed = max(desired_speed, self._min_desired_speed)
                desired_speed = min(desired_speed, self._max_desired_speed)
            else:
                desired_speed = self._desired_speed

            # always use desired speed for pre-fill vehicles
            depart_speed = desired_speed

            if self._random_arrival_position:
                arrival_position = random.randrange(depart_position + max(self._arrival_interval, self._minimum_trip_length), self._road_length, self._arrival_interval)
            else:
                arrival_position = self._road_length

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
                if self._start_as_platoon:
                    if vid == 0:
                        vehicle._cf_mode = CF_Mode.ACC
                        vehicle._platoon_role = PlatoonRole.LEADER
                    else:
                        vehicle._cf_mode = CF_Mode.CACC
                        vehicle._platoon_role = PlatoonRole.FOLLOWER
                    vehicle._platoon = Platoon(0, [vehicle], vehicle.desired_speed)

            else:
                vehicle = Vehicle(self, vid, vtype, depart_position, arrival_position,
                                  desired_speed, depart_lane, depart_speed, depart_time, self._communication_range)

            self._vehicles[vid] = vehicle
            self._last_vehicle_id = vid

            if self._gui:
                self._add_gui_vehicle(vehicle)

            if self._start_as_platoon and vid > 0:
                vehicle._join(0, 0)

            LOG.debug(f"Spawned vehicle {vid}")

    # TODO remove duplicated code
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
            sys.exit("depart method fixed is not yet implemented!")
        else:
            sys.exit("Unknown depart method!")

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

        if self._random_depart_position:
            depart_position = random.randrange(length, self.road_length, self._depart_interval + length + min_gap)
        else:
            depart_position = length  # equal to departPos="base"

        # check whether the can actually be inserted
        collision = True  # assume we have a collision to check at least once
        while collision:
            collision = False  # so far we do not have a collision
            # check all vehicles
            for other_vehicle in self._vehicles.values():
                if other_vehicle.lane != depart_lane:
                    # we do not care about other lanes
                    continue
                # do we have a collision?
                # TODO HACK for using collision check
                vehicle = Vehicle(self, vid, vtype, depart_position, -1, -1, depart_lane, -1, depart_time, -1)
                collision = collision or self.has_collision(vehicle, other_vehicle)
            # can we avoid the collision by switching the departure lane?
            if collision:
                if depart_lane < self.number_of_lanes:
                    depart_lane = depart_lane + 1
                    LOG.debug(f"Increased depart lane for {vid} to avoid a collision")
                else:
                    sys.exit(f"{vid} crashed at start into {vehicle.vid}")

        if self._random_desired_speed:
            # normal distribution
            desired_speed = self._desired_speed * random.normalvariate(1.0, self._speed_variation)
            desired_speed = max(desired_speed, self._min_desired_speed)
            desired_speed = min(desired_speed, self._max_desired_speed)
        else:
            desired_speed = self._desired_speed

        if self._random_depart_speed:
            depart_speed = random.randrange(0, self._desired_speed, 1)
        else:
            depart_speed = 0

        if self._depart_desired:
            depart_speed = desired_speed

        if self._random_arrival_position:
            arrival_position = random.randrange(depart_position + max(self._arrival_interval, self._minimum_trip_length), self._road_length, self._arrival_interval)
        else:
            arrival_position = self._road_length

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
            vehicle = Vehicle(self, vid, vtype, depart_position, arrival_position,
                              desired_speed, depart_lane, depart_speed, depart_time, self._communication_range)

        self._vehicles[vid] = vehicle
        self._last_vehicle_id = vid

        if self._gui:
            self._add_gui_vehicle(vehicle)

        if self._start_as_platoon and vid > 0:
            vehicle._join(0, 0)

        LOG.info(f"Spawned vehicle {vid}")

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
                f.write("id,depart,departLane,departPos,departSpeed,arrival,arrivalLane,arrivalPos,arrivalSpeed,duration,routeLength,timeLoss,desiredSpeed,estimatedTravelTime,travelTimeRatio,avgDrivingSpeed,avgDeviationDesiredSpeed\n")

        if self._record_vehicle_emissions:
            # create output file for vehicle emissions
            with open(self._result_base_filename + '_vehicle_emissions.csv', 'w') as f:
                f.write("id,CO,CO2,HC,PMx,NOx,fuel\n")

        if self._record_vehicle_traces:
            # create output file for vehicle traces
            with open(self._result_base_filename + '_vehicle_traces.csv', 'w') as f:
                f.write("step,id,position,lane,speed,duration,routeLength\n")

        if self._record_vehicle_changes:
            # create output file for vehicle lane changes
            with open(self._result_base_filename + '_vehicle_changes.csv', 'w') as f:
                f.write("step,id,position,from,to,speed,reason\n")

        if self._record_emission_traces:
            # create output file for emission traces
            with open(self._result_base_filename + '_emission_traces.csv', 'w') as f:
                f.write("step,id,CO,CO2,HC,PMx,NOx,fuel\n")

        if self._record_platoon_trips:
            # create output file for platoon trips
            with open(self._result_base_filename + '_platoon_trips.csv', 'w') as f:
                f.write("id,timeInPlatoon,distanceInPlatoon,platoonTimeRatio,platoonDistanceRatio\n")

        if self._record_platoon_traces:
            # create output file for platoon traces
            with open(self._result_base_filename + '_platoon_traces.csv', 'w') as f:
                f.write("step,id,leader,position,rear_position,lane,speed,size,length\n")

        if self._record_platoon_changes:
            # create output file for platoon lane changes
            with open(self._result_base_filename + '_platoon_changes.csv', 'w') as f:
                f.write("step,id,position,from,to,speed,reason\n")

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

    def _update_gui(self):
        import traci
        for vehicle in self._vehicles.values():
            if vehicle.depart_time > self._step:
                # vehicle did not start yet
                continue
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
            LOG.warn("Simulation is already running!")

        self._initialize_result_recording()

        # initialize the GUI
        if self._gui:
            self._initialize_gui()

        progress_bar = tqdm(desc='Simulation progress', total=self._max_step, unit='step')
        # let the simulator run
        while self._running:
            if self.step >= self._max_step:
                self.stop("Reached step limit")
                continue

            # spawn vehicle based on given parameters
            self._spawn_vehicle()

            if len(self._vehicles) == 0:
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

            # adjust positions (of all vehicles)
            self._move_vehicles()

            # do collision check (for all vehicles)
            if self._collisions:
                self._check_collisions()

            # a new step begins
            self._step += self._step_length
            progress_bar.update(self._step_length)
            if self._gui:
                import traci
                traci.simulationStep(self.step)
                assert(traci.simulation.getTime() == float(self.step))

    def stop(self, msg: str):
        """Stop the simulation with the given message"""

        self._running = False
        print(f"\n{msg}")
        self.finish()

    def __str__(self) -> str:
        """Return a nice string representation of a simulator instance"""

        import funcy  # TODO get rid of this dependency
        sim_dict = self.__dict__
        sim_dict = funcy.omit(sim_dict, '_vehicles')
        sim_dict = funcy.omit(sim_dict, '_infrastructures')
        sim_dict.update({'current_number_of_vehicles': len(self._vehicles)})
        sim_dict.update({'current_number_of_infrastructures': len(self._infrastructures)})
        return str(sim_dict)

    def finish(self):
        """Clean up the simulation"""

        if self._running:
            LOG.warn("Finish called during simulation!")
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
