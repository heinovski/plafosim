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
import time

from math import copysign
from random import normalvariate, randrange, random, seed
from tqdm import tqdm
from .vehicle import VehicleType, Vehicle
from .platooning_vehicle import PlatooningVehicle, CF_Mode, PlatoonRole, Platoon

# assumptions
# you just reach your arrival_position
# position is in the middle of the front bumper
# a vehicle ends at position + length
# crash detection does not work with steps greater than 1


class Simulator:
    """A collection of parameters and information of the simulator"""

    def __init__(
            self,
            road_length: int,
            number_of_lanes: int,
            lane_changes: bool,
            collisions: bool,
            step_length: int,
            random_seed: int,
            log_level: int,
            gui: bool,
            gui_delay: int,
            gui_track_vehicle: int,
            result_base_filename: str):
        """Initialize a simulator instance"""

        # TODO add custom filter that prepends the log entry with the step time
        logging.basicConfig(level=log_level, format="%(levelname)s: %(message)s")

        # road network properties
        self._road_length = road_length  # the length of the road
        self._number_of_lanes = number_of_lanes  # the number of lanes

        # vehicle properties
        self._vehicles = {}  # the list (dict) of vehicles in the simulation
        self._lane_changes = lane_changes  # whether to enable lane changes
        self._collisions = collisions  # whether to check for collisions

        # simulation properties
        self._step = 0  # the current simulation step in s
        self._step_length = step_length  # the length of a simulation step
        if random_seed >= 0:
            logging.info("Using random seed %d" % random_seed)
            seed(random_seed)
            self._random_seed = random_seed
        self._running = False  # whether the simulation is running
        self._gui = gui  # whether to show a live sumo-gui
        self._gui_delay = gui_delay  # the delay in every simulation step for the gui
        self._gui_track_vehicle = gui_track_vehicle  # the id of a vehicle to track in the gui
        self._result_base_filename = result_base_filename  # the base filename of the result files

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

    def call_actions(self):
        """Trigger actions of all vehicles"""

        for vehicle in self._vehicles.values():
            vehicle.action()

    def speed2distance(self, speed: float, time_interval: float = 1) -> float:
        return speed * time_interval

    def distance2speed(self, distance: float, time_interval: float = 1) -> float:
        return distance / time_interval

    def acceleration2speed(self, acceleration: float, time_interval: float = 1) -> float:
        return acceleration * time_interval

    def speed2acceleration(self, speed_from: float, speed_to: float, time_interval: float = 1) -> float:
        return (speed_to - speed_from) / time_interval

    def _get_predecessor_id(self, vid: int, lane: int = -1) -> int:
        position = self._vehicles[vid].position
        if lane == -1:
            lane = self._vehicles[vid].lane
        predecessor_id = -1
        for vehicle in self._vehicles.values():
            if vehicle.vid == vid:
                continue
            if vehicle.depart_time > self._step:
                # vehicle did not start yet
                continue
            if vehicle.lane != lane:
                continue
            if vehicle.position < position:
                continue
            if vehicle.position == position:
                # TODO throw error if the vehicles are "interleaved"
                continue
            if predecessor_id == -1 or vehicle.position < self._vehicles[predecessor_id].position:
                predecessor_id = vehicle.vid
            # TODO throw error if predecessor and vehicle are "interleaved"
        return predecessor_id

    def _get_successor_id(self, vid: int, lane: int = -1) -> int:
        position = self._vehicles[vid].position
        if lane == -1:
            lane = self._vehicles[vid].lane
        successor_id = -1
        for vehicle in self._vehicles.values():
            if vehicle.vid is vid:
                continue
            if vehicle.lane is not lane:
                continue
            if vehicle.position > position:
                continue
            if vehicle.position is position:
                # TODO throw error if the vehicles are "interleaved"
                continue
            if successor_id == -1 or vehicle.position > self._vehicles[successor_id].position:
                successor_id = vehicle.vid
            # TODO throw error if successor and vehicle are "interleaved"
        return successor_id

    def _get_predecessor_rear_position(self, vid: int, lane: int = -1) -> int:
        pid = self._get_predecessor_id(vid, lane)
        if pid == -1:
            return -1
        else:
            return self._vehicles[pid].rear_position

    def _get_predecessor_speed(self, vid: int, lane: int = -1) -> int:
        pid = self._get_predecessor_id(vid, lane)
        if pid == -1:
            return -1
        else:
            return self._vehicles[pid].speed

    def is_lane_change_safe(self, vid: int, target_lane: int) -> bool:
        v = self._vehicles[vid]

        if v.lane == target_lane:
            return True

        # check predecessor on target lane
        predecessor_on_target_lane = self._get_predecessor_id(vid, target_lane)
        if predecessor_on_target_lane != -1:
            p = self._vehicles[predecessor_on_target_lane]
            gap_to_predecessor_on_target_lane = p.rear_position - v.position
            if v.speed > v._safe_speed(p.speed, gap_to_predecessor_on_target_lane, v.desired_gap, v.vehicle_type.min_gap):
                return False

        # check successor on target lane
        successor_on_target_lane = self._get_successor_id(vid, target_lane)
        if successor_on_target_lane != -1:
            s = self._vehicles[successor_on_target_lane]
            gap_to_successor_on_target_lane = v.rear_position - s.position
            if s.speed > s._safe_speed(v.speed, gap_to_successor_on_target_lane):
                return False

        # safe
        return True

    # TODO move to vehicle?
    def _change_lane(self, vid: int, target_lane: int, reason: str) -> bool:
        v = self._vehicles[vid]
        source_lane = v.lane
        if source_lane == target_lane:
            return True
        logging.info("%d wants to chage from lane %d to lane %d" % (vid, source_lane, target_lane))

        lane_diff = target_lane - source_lane
        if abs(lane_diff) > 1:
            logging.warn("%d only change to adjacent lane!")
            old_target_lane = target_lane
            target_lane = source_lane + copysign(1, lane_diff)
            logging.info("Adjusted target lane to %d (from %d)" % (target_lane, old_target_lane))

        if isinstance(v, PlatooningVehicle) and v.is_in_platoon():
            # followers are not allowed to change the lane on their one
            assert(v.platoon_role != PlatoonRole.FOLLOWER)

            # leaders are allowed to change the lane
            if v.platoon_role == PlatoonRole.LEADER:
                assert(reason == "speedGain" or reason == "keepRight")

                logging.info("%d needs to check all platoon members" % vid)

                can_change = True
                for member in v.platoon.formation:
                    can_change = can_change and self.is_lane_change_safe(member.vid, target_lane)
                    if not can_change:
                        logging.debug("lane change is not safe for %d")

                if can_change:
                    # perform lane change for all vehicles in this platoon
                    for member in v.platoon.formation:
                        assert(member.lane == source_lane)
                        logging.info("%d is switching lanes: %d -> %d (%s)" % (member.vid, source_lane, target_lane, reason))

                        # switch to adjacent lane
                        member._lane = target_lane

                        # log lane change
                        with open(self._result_base_filename + '_member_changes.csv', 'a') as f:
                            f.write("%d,%d,%f,%d,%d,%f,%s\n" % (self.step, member.vid, member.position, source_lane, target_lane, member.speed, reason))

                    return abs(lane_diff) <= 1
                return False

        # we are just a regular vehicle or we are not (yet) in a platoon

        # check adjacent lane is free
        if self.is_lane_change_safe(vid, target_lane):
            logging.info("%d is switching lanes: %d -> %d (%s)" % (vid, source_lane, target_lane, reason))

            # switch to adjacent lane
            v._lane = target_lane

            # log lane change
            with open(self._result_base_filename + '_vehicle_changes.csv', 'a') as f:
                f.write("%d,%d,%f,%d,%d,%f,%s\n" % (self.step, vid, v.position, source_lane, target_lane, v.speed, reason))

            return abs(lane_diff) <= 1
        return False

    # kraus - multi lane traffic
    # lane-change
    # congested = (v_safe < v_thresh) and (v^0_safe < v_thresh)
    # favorable(right->left) = (v_safe < v_max) and (not congested)
    # favorable(left->right) = (v_safe >= v_max) and (v^0_safe >= v_max)
    # if ((favorable(i->j) or (rand < p_change)) and safe(i->j)) then change(i->j)
    # for vehicles on the right lane:
    # if (v > v^0_safe) and (not congested) then v <- v^0_safe
    def change_lanes(self):
        """Do lane changes for all vehicles"""

        for vehicle in self._vehicles.values():
            if vehicle.depart_time > self._step:
                # vehicle did not start yet
                continue

            # decide upon and perform a lane change for this vehicle
            if vehicle.blocked_front:
                if vehicle.lane < self.number_of_lanes - 1:
                    source_lane = vehicle.lane
                    target_lane = source_lane + 1
                    # TODO determine whether it is useful to overtake
                    self._change_lane(vehicle.vid, target_lane, "speedGain")
            else:
                if isinstance(vehicle, PlatooningVehicle) and vehicle.platoon_role == PlatoonRole.FOLLOWER:
                    # followers are not allowed to change the lane on their own
                    continue
                if vehicle.lane > 0:
                    source_lane = vehicle.lane
                    target_lane = source_lane - 1
                    self._change_lane(vehicle.vid, target_lane, "keepRight")

    def adjust_speeds(self):
        """Do speed adjustments for all vehicles"""

        for vehicle in self._vehicles.values():
            if vehicle.depart_time > self._step:
                # vehicle did not start yet
                continue

            logging.debug("%d's current speed %f" % (vehicle.vid, vehicle.speed))

            new_speed = vehicle.new_speed(self._get_predecessor_speed(vehicle.vid), self._get_predecessor_rear_position(vehicle.vid), vehicle.desired_gap)
            vehicle._acceleration = new_speed - vehicle.speed

            logging.debug("%d's current acceleration: %f" % (vehicle.vid, vehicle.acceleration))

            vehicle._speed = new_speed

    # krauss - single lane traffic
    # adjust position (move)
    # x(t + step_size) = x(t) + v(t)*step_size
    def move_vehicles(self):
        """Do position updates for all vehicles"""

        for vehicle in sorted(self._vehicles.values(), key=lambda x: x.position, reverse=True):
            if vehicle.depart_time > self._step:
                # vehicle did not start yet
                continue
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
                continue
            else:
                # TODO use proper method
                vehicle._position += position_difference

    def check_collisions(self):
        """Do collision checks for all vehicles"""

        for vehicle in self._vehicles.values():
            if vehicle.depart_time > self._step:
                # vehicle did not start yet
                continue
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
                if self.has_collision(vehicle.vid, vehicle.position, vehicle.rear_position, other_vehicle.vid, other_vehicle.position, other_vehicle.rear_position):
                    logging.critical("collision between %d and %d" % (vehicle.vid, other_vehicle.vid))
                    logging.debug("%d (%f-%f)" % (vehicle.vid, vehicle.position, vehicle.rear_position))
                    logging.debug("%d (%f-%f)" % (other_vehicle.vid, other_vehicle.position, other_vehicle.rear_position))
                    exit(1)

    def has_collision(self, vid1: float, pos1: float, rear_pos1: float, vid2: float, pos2: float, rear_pos2: float) -> bool:
        return min(pos1, pos2) - max(rear_pos1, rear_pos2) >= 0

    # TODO move out of simulator class
    # TODO generate while simulation is running
    def generate_vehicles(
            self,
            max_step: int,
            number_of_vehicles: int,
            penetration_rate: float,
            depart_interval: int,
            arrival_interval: int,
            max_speed: float,
            acc_headway_time: float,
            cacc_spacing: float,
            random_depart_position: bool,
            random_depart_lane: bool,
            desired_speed: float,
            random_desired_speed: bool,
            speed_variation: float,
            min_desired_speed: float,
            max_desired_speed: float,
            random_depart_speed: bool,
            depart_desired: bool,
            depart_method: str,
            depart_time_interval: int,
            random_arrival_position: bool,
            start_as_platoon: bool,
            formation_strategy: str,
            alpha: float,
            speed_deviation_threshold: float,
            position_deviation_threshold: int):
        """Generate vehicles for the simulation"""

        last_vehicle_id = -1

        # vehicle properties
        length = 4
        max_acceleration = 2.5  # m/s
        max_deceleration = 15  # m/s
        # imperfection = 0.5  # sigma
        min_gap = 0  # m
        vtype = VehicleType("car", length, max_speed, max_acceleration, max_deceleration, min_gap)  # TODO multiple vtypes

        for num in tqdm(range(0, number_of_vehicles), desc="Generated vehicles"):
            vid = last_vehicle_id + 1

            if random_depart_position:
                if start_as_platoon:
                    logging.warn("Vehicles can not have random departure positions when starting as one platoon!")
                    exit(1)

                depart_position = position = randrange(0, self._road_length, depart_interval)
            else:
                depart_position = 0
            depart_position = depart_position + length  # equal to departPos="base"

            if random_depart_lane:
                if start_as_platoon:
                    logging.warn("Vehicles can not have random departure lanes when starting as one platoon!")
                    exit(1)

                depart_lane = randrange(0, self._number_of_lanes, 1)
            else:
                depart_lane = 0

            if random_desired_speed:
                # normal distribution
                speed = desired_speed * normalvariate(1.0, speed_variation)
                speed = max(speed, min_desired_speed)
                speed = min(speed, max_desired_speed)
            else:
                speed = desired_speed

            if random_depart_speed:
                depart_speed = randrange(0, desired_speed, 1)
            else:
                depart_speed = 0

            if depart_desired:
                depart_speed = desired_speed

            if depart_method == "interval":
                if last_vehicle_id != -1:
                    depart_time = self._vehicles[last_vehicle_id].depart_time + depart_time_interval
                else:
                    depart_time = 0
            #elif spawn_strategy is "rate":
                # TODO
                # vehicles per hour
                # max_step / 3600 --> # hours
                # rate per hour / 3600 --> rate per second
                # random > rate? spawn
            #elif spawn_strategy is "probability":
                # TODO
            else:
                depart_time = randrange(0, max_step, 1 * 60)  # in which minute to start
            # safety_gap = 0  # m

            if random_arrival_position:
                arrival_position = randrange(position + arrival_interval, self._road_length, arrival_interval)
            else:
                arrival_position = self._road_length

            if start_as_platoon:
                if penetration_rate < 1.0:
                    logging.warn("The penetration rate cannot be smaller than 1.0 when starting as one platoon!")
                    exit(1)
                if formation_strategy is not None:
                    logging.warn("A formation strategy cannot be used when all starting as one platoon!")
                    exit(1)

            # choose vehicle "type" depending on the penetration rate
            if random() < penetration_rate:
                vehicle = PlatooningVehicle(
                    self,
                    vid,
                    vtype,
                    depart_position,
                    arrival_position,
                    speed,
                    depart_lane,
                    depart_speed,
                    depart_time,
                    acc_headway_time,
                    cacc_spacing,
                    formation_strategy,
                    alpha,
                    speed_deviation_threshold,
                    position_deviation_threshold)
                if start_as_platoon:
                    if vid == 0:
                        vehicle._cf_mode = CF_Mode.ACC
                        vehicle._platoon_role = PlatoonRole.LEADER
                    else:
                        vehicle._cf_mode = CF_Mode.CACC
                        vehicle._platoon_role = PlatoonRole.FOLLOWER
                    vehicle._platoon = Platoon(0, [vehicle], desired_speed)

            else:
                vehicle = Vehicle(self, vid, vtype, depart_position, arrival_position,
                                  speed, depart_lane, depart_speed, depart_time)

            self._vehicles[vid] = vehicle
            logging.info("Generated vehicle", vehicle)

            last_vehicle_id = vid

        if start_as_platoon:
            platoon = Platoon(0, list(self._vehicles.values()), self._vehicles[0].desired_speed)
            for vehicle in self._vehicles.values():
                vehicle._platoon = platoon

    def run(self, max_step: int):
        """Run the simulation with the specified parameters"""

        if not self._running:
            self._running = True
        else:
            logging.warn("Simulation is already running!")

        # write some general information about the simulation
        with open(self._result_base_filename + '_general.out', 'w') as f:
            f.write("simulation start: " + time.asctime(time.localtime(time.time())) + '\n')
            f.write("parameters" + str(self) + '\n')

        # create output file for vehicle trips
        with open(self._result_base_filename + '_vehicle_trips.csv', 'w') as f:
            f.write("id,depart,departLane,departPos,departSpeed,arrival,arrivalLane,arrivalPos,arrivalSpeed,duration,routeLength,timeLoss,desiredSpeed\n")

        # create output file for vehicle emissions
        with open(self._result_base_filename + '_vehicle_emissions.csv', 'w') as f:
            f.write("id,CO,CO2,HC,PMx,NOx,fuel\n")

        # create output file for vehicle traces
        with open(self._result_base_filename + '_vehicle_traces.csv', 'w') as f:
            f.write("step,id,position,lane,speed,duration,routeLength\n")

        # crieate output file for vehicle lane changes
        with open(self._result_base_filename + '_vehicle_changes.csv', 'w') as f:
            f.write("step,id,position,from,to,speed,reason\n")

        if self._gui:
            import os
            import sys

            if 'SUMO_HOME' not in os.environ:
                sys.exit("please declare environment variable 'SUMO_HOME'")

            tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
            sys.path.append(tools)

            import traci

            sumoBinary = os.path.join(os.environ['SUMO_HOME'], 'bin/sumo-gui')
            sumoCmd = [sumoBinary, "-Q", "-c", "sumocfg/freeway.sumo.cfg", '--collision.action', 'warn']

            traci.start(sumoCmd)
            traci.simulationStep(self._step)
            from random import randrange

        progress_bar = tqdm(desc='Simulation progress', total=max_step, unit='step')
        # let the simulator run
        while self._running:
            if self._step >= max_step:
                self.stop("Reached step limit")
                continue
            if len(self._vehicles) == 0:
                self.stop("No more vehicles in the simulation")  # do we really want to exit here?
                continue

            if self._gui:

                # simulate vehicles from trace file
                for vehicle in self._vehicles.values():
                    if vehicle.depart_time > self._step:
                        # vehicle did not start yet
                        continue
                    # add vehicles
                    if str(vehicle.vid) not in traci.vehicle.getIDList():
                        traci.vehicle.add(str(vehicle.vid), 'route', departPos=str(vehicle.position), departSpeed=str(vehicle.speed), departLane=str(vehicle.lane), typeID='vehicle')
                        traci.vehicle.setColor(str(vehicle.vid), (randrange(0, 255, 1), randrange(0, 255, 1), randrange(0, 255, 1)))
                        traci.vehicle.setSpeedMode(str(vehicle.vid), 0)
                        traci.vehicle.setLaneChangeMode(str(vehicle.vid), 0)
                        # track vehicle
                        if vehicle.vid == self._gui_track_vehicle:
                            traci.gui.trackVehicle("View #0", str(vehicle.vid))
                            traci.gui.setZoom("View #0", 1000000)
                    # update vehicles
                    traci.vehicle.setSpeed(str(vehicle.vid), vehicle.speed)
                    traci.vehicle.moveTo(vehID=str(vehicle.vid), pos=vehicle.position, laneID='edge_0_0_%d' % vehicle.lane)

                traci.simulationStep(self._step)

                # remove vehicles not in simulator
                for vid in traci.vehicle.getIDList():
                    if int(vid) not in self._vehicles.keys():
                        traci.vehicle.remove(vid, 2)

                # sleep for visualization
                time.sleep(self._gui_delay)

            # call regular actions on vehicles
            self.call_actions()

            # perform lane changes (for all vehicles)
            if self._lane_changes:
                self.change_lanes()

            # adjust speed (of all vehicles)
            self.adjust_speeds()

            # adjust positions (of all vehicles)
            self.move_vehicles()

            # do collision check (for all vehicles)
            if self._collisions:
                self.check_collisions()

            self._step += self._step_length
            progress_bar.update(self._step_length)

    def stop(self, msg: str):
        """Stop the simulation with the given message"""

        self._running = False
        print("\n%s" % msg)
        self.finish()

    def __str__(self) -> str:
        """Return a nice string representation of a simulator instance"""

        import funcy  # TODO get rid of this dependency
        return str(funcy.omit(self.__dict__, '_vehicles'))

    def finish(self):
        """Clean up the simulation"""

        # write some general information about the simulation
        with open(self._result_base_filename + '_general.out', 'a') as f:
            f.write("simulation end: " + time.asctime(time.localtime(time.time())) + '\n')

        if self._gui:
            import traci
            # remove all vehicles
            for vid in traci.vehicle.getIDList():
                traci.vehicle.remove(vid, 2)
            traci.close(False)
