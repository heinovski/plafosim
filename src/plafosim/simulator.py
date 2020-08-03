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
import time

from random import normalvariate, randrange, random
from .vehicle import VehicleType, Vehicle, PlatooningVehicle

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
            collisions: bool,
            step_length: int,
            debug: bool,
            gui: bool,
            gui_delay: int,
            result_base_filename: str):
        """Initialize a simulator instance"""

        # road network properties
        self._road_length = road_length  # the length of the road
        self._number_of_lanes = number_of_lanes  # the number of lanes

        # vehicle properties
        self._vehicles = {}  # the list (dict) of vehicles in the simulation
        self._collisions = collisions  # whether to check for collisions

        # simulation properties
        self._step = 0  # the current simulation step in s
        self._step_length = step_length  # the length of a simulation step
        self._running = False  # whether the simulation is running
        self._debug = debug  # whether debugging is enabled
        self._gui = gui  # whether to show a live sumo-gui
        self._gui_delay = gui_delay  # the delay in every simulation step for the gui
        self._result_base_filename = result_base_filename  # the base filename of the result files

    @property
    def road_length(self) -> int:
        return self._road_length

    @property
    def number_of_lanes(self) -> int:
        return self._number_of_lanes

    @property
    def step(self) -> int:
        return self._step

    def call_actions(self):
        """Trigger actions of all vehicles"""

        for vehicle in self._vehicles.values():
            vehicle.action()

    def _get_predecessor_id(self, vid: int, lane: int = -1) -> int:
        position = self._vehicles[vid].position
        if lane == -1:
            lane = self._vehicles[vid].lane
        predecessor_id = -1
        for vehicle in self._vehicles.values():
            if vehicle.vid is vid:
                continue
            if vehicle.lane is not lane:
                continue
            if vehicle.position < position:
                continue
            if vehicle.position is position:
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

    def is_lane_change_safe(self, vid: int, target_lane: int) -> bool:
        if self._vehicles[vid].lane is target_lane:
            return True

        # check predecessor on target lane
        predecessor_position_on_target_lane = self._get_predecessor_rear_position(vid, target_lane)
        if predecessor_position_on_target_lane != -1:
            gap_to_predecessor_on_target_lane = predecessor_position_on_target_lane - self._vehicles[vid].position
            if self._vehicles[vid].speed > (gap_to_predecessor_on_target_lane / self._step_length):
                return False

        # check successor on target lane
        successor_on_target_lane = self._get_successor_id(vid, target_lane)
        if successor_on_target_lane != -1:
            gap_to_successor_on_target_lane = self._vehicles[vid].rear_position - self._vehicles[successor_on_target_lane].position
            if self._vehicles[successor_on_target_lane].speed > (gap_to_successor_on_target_lane / self._step_length):
                return False

        # safe
        return True

    def _change_lane(self, vid: int, target_lane: int):
        # check adjacent lane is free
        if self.is_lane_change_safe(vid, target_lane):
            # switch to adjacent lane
            print("%d switching lanes %d -> %d" % (vid, self._vehicles[vid].lane, target_lane), flush=True)
            self._vehicles[vid]._lane = target_lane

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
                    target_lane = vehicle.lane + 1
                    # TODO determine whether it is useful to overtake
                    self._change_lane(vehicle.vid, target_lane)
            else:
                if vehicle.lane > 0:
                    target_lane = vehicle.lane - 1
                    self._change_lane(vehicle.vid, target_lane)

    def adjust_speeds(self):
        """Do speed adjustments for all vehicles"""

        for vehicle in self._vehicles.values():
            if vehicle.depart_time > self._step:
                # vehicle did not start yet
                continue

            vehicle._speed = vehicle.new_speed(self._get_predecessor_rear_position(vehicle.vid))

    # krauss - single lane traffic
    # adjust position (move)
    # x(t + step_size) = x(t) + v(t)*step_size
    def move_vehicles(self):
        """Do position updates for all vehicles"""

        for vid in list(self._vehicles.keys()):
            vehicle = self._vehicles[vid]
            if vehicle.depart_time > self._step:
                # vehicle did not start yet
                continue
            # increase position according to speed
            position_difference = vehicle.speed * self._step_length
            # TODO add emissions/fuel statistics
            # arrival_position reached?
            if vehicle.position + position_difference >= vehicle.arrival_position:
                # TODO use proper method
                vehicle._position = vehicle.arrival_position
                vehicle.finish()
                del self._vehicles[vid]
                if self._gui:
                    import traci
                    traci.vehicle.remove(str(vid), 2)
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
                    print("%s (%d-%d)" % (vehicle.vid, vehicle.position, vehicle.rear_position))
                    print("%s (%d-%d)" % (other_vehicle.vid, other_vehicle.position, other_vehicle.rear_position))
                    exit(1)

    def has_collision(self, vid1: int, pos1: int, rear_pos1: int, vid2: int, pos2: int, rear_pos2: int) -> bool:

        # front bumper of follower is within back of leader
        if (pos1 < pos2 and rear_pos1 < pos2 and rear_pos1 < rear_pos2 and pos1 == rear_pos2) or (pos2 < pos1 and rear_pos2 < pos1 and rear_pos2 < rear_pos1 and pos2 == rear_pos1):
            print(self._step, "crash (same position)", vid1, pos1, rear_pos1, vid2, pos2, rear_pos2, flush=True)
            return True

        # follower is (partly) in leader
        if (pos1 <= pos2 and rear_pos1 < pos2 and pos1 >= rear_pos2 and rear_pos1 <= rear_pos2) or (pos2 <= pos1 and rear_pos2 < pos1 and pos2 >= rear_pos1 and rear_pos2 <= rear_pos1):
            print(self._step, "crash (regular)", vid1, pos1, rear_pos1, vid2, pos2, rear_pos2, flush=True)
            return True

        # follower is completely within leader
        if (pos1 >= pos2 and rear_pos1 <= rear_pos2 and rear_pos1 < pos2 and pos1 > pos2) or (pos2 >= pos1 and rear_pos2 <= rear_pos1 and rear_pos2 < pos1 and pos2 > pos1):
            print(self._step, "crash (within)", vid1, pos1, rear_pos1, vid2, pos2, rear_pos2, flush=True)
            return True

        # no collision!
        assert(
        (pos1 < rear_pos2 and rear_pos1 < rear_pos2 and pos1 < pos2 and rear_pos1 < pos2) or
        (pos2 < rear_pos1 and rear_pos2 < rear_pos1 and pos2 < pos1 and rear_pos2 < pos1)
            )

        return False

    # TODO move out of simulator class
    # TODO generate while simulation is running
    def generate_vehicles(
            self,
            max_step: int,
            number_of_vehicles: int,
            penetration_rate: float,
            depart_interval: int,
            arrival_interval: int,
            max_speed: int,
            random_depart_position: bool,
            random_depart_lane: bool,
            desired_speed: int,
            random_desired_speed: bool,
            speed_variation: float,
            min_desired_speed: int,
            max_desired_speed: int,
            random_depart_speed: bool,
            depart_desired: bool,
            depart_method: str,
            depart_time_interval: int,
            random_arrival_position: bool):
        """Generate vehicles for the simulation"""

        last_vehicle_id = -1

        # vehicle properties
        length = 4
        max_acceleration = 2.5 # m/s
        max_deceleration = 15  # m/s
        imperfection = 0.5  # sigma
        vtype = VehicleType("car", length, max_speed, max_acceleration, max_deceleration)  # TODO multiple vtypes

        for num in range(0, number_of_vehicles):
            vid = last_vehicle_id + 1

            if random_depart_position:
                depart_position = position = randrange(0, self._road_length, depart_interval)
            else:
                depart_position = 0
            depart_position = depart_position + length  # equal to departPos="base"

            if random_depart_lane:
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
                arrival_position = randrange(position + 1, self._road_length, arrival_interval)
            else:
                arrival_position = self._road_length

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
                    depart_time)
            else:
                vehicle = Vehicle(self, vid, vtype, depart_position, arrival_position,
                                  speed, depart_lane, depart_speed, depart_time)

            self._vehicles[vid] = vehicle

            last_vehicle_id = vid

    def run(self, max_step: int):
        """Run the simulation with the specified parameters"""

        if not self._running:
            self._running = True
        else:
            print("Simulation is already running!")
            exit(1)

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

        if self._gui:
            import os
            import sys

            if 'SUMO_HOME' not in os.environ:
                sys.exit("please declare environment variable 'SUMO_HOME'")

            tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
            sys.path.append(tools)

            import traci

            sumoBinary = os.path.join(os.environ['SUMO_HOME'], 'bin/sumo-gui')
            sumoCmd = [sumoBinary, "-Q", "-c", "cfg/freeway.sumo.cfg", '--collision.action', 'warn']

            traci.start(sumoCmd)
            traci.simulationStep(self._step)
            from random import randrange

        print("Starting simulation", flush=True)

        # let the simulator run
        while self._running:
            if self._step >= max_step:
                self.stop("Reached step limit")
                continue
            if len(self._vehicles) == 0:
                self.stop("No more vehicles in the simulation")  # do we really want to exit here?
                continue

            print("\rCurrent step: %d" % self._step, sep=' ', end='...', flush=True)

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
            self.change_lanes()

            # adjust speed (of all vehicles)
            self.adjust_speeds()

            # adjust positions (of all vehicles)
            self.move_vehicles()

            # do collision check (for all vehicles)
            if self._collisions:
                self.check_collisions()

            self._step += self._step_length

    def stop(self, msg: str):
        """Stop the simulation with the given message"""

        self._running = False
        print("\n%s" % msg, flush=True)
        self.finish()

    def __str__(self) -> str:
        """Return a nice string representation of a simulator instance"""

        import funcy  # TODO get rid of this dependency
        return str(funcy.omit(self.__dict__, '_vehicles'))

    def finish(self):
        """Clean up the simulation"""

        # write some general information about the simulation
        with open(self._result_base_filename + '_general.out', 'w') as f:
            f.write("simulation end: " + time.asctime(time.localtime(time.time())) + '\n')

        if self._gui:
            import traci
            # remove all vehicles
            for vid in traci.vehicle.getIDList():
                traci.vehicle.remove(vid, 2)
            traci.close(False)
