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

from random import randrange, random
from .vehicle import VehicleType, Vehicle, PlatooningVehicle

# assumptions
# you just reach your arrival_position
# position is in the middle of the front bumper
# a vehicle ends at position + length
# crash detection does not work with steps greater than 1


# krauss - single lane traffic
# adjust speed
# v_max, desired speed
# epsilon, dawdling of drives
# g_des = tau*v_lead
# tau, reaction time of drivers
# tau_b = v/b
# v_safe(t) = v_lead(t) + (g(t)-g_des(t)) / (tau_b + tau)
# v_des(t) = min[v_max, v(t)+a(v)*step_size, v_safe(t)]
# v(t + step_size) = max[0, v_des(t) - epsilon]
# TODO move to the vehicle??
def new_speed(current_speed: int, desired_speed: int, max_speed: int, max_acceleration: int, max_deceleration: int, current_position: int, pred_rear_position: int) -> int:
    """Calcuate the new speed for a vehicle using the kraus model"""

    new_speed = -1
    # do we need to adjust our speed?
    diff_to_desired = desired_speed - current_speed
    if diff_to_desired > 0:
        # we need to accelerate
        new_speed = min(current_speed + min(diff_to_desired, max_acceleration), max_speed)
    elif diff_to_desired < 0:
        # we need to decelerate
        new_speed = max(current_speed - max(diff_to_desired, max_deceleration), 0)
    else:
        new_speed = current_speed

    # TODO vsafe
    # TODO this is a simple and dumb calculation for the safe speed of a vehicle based on the positions of the predessor and the vehicle itself
    safe_speed = (pred_rear_position - current_position)  # TODO / self._step_length # - desired_gap + pred_speed
    new_speed = min(safe_speed, new_speed)

    # TODO dawdling?
    # new_speed -= random() * new_speed

    if (new_speed < 0):
        new_speed = 0

    return new_speed


class Simulator:
    """A collection of paramaters and information of the simulator"""

    def __init__(
            self,
            road_length: int,
            number_of_lanes: int,
            collisions: bool,
            step_length: int,
            debug: bool,
            gui: bool,
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
        self._step_length = step_length  # the lengh of a simulation step
        self._debug = debug  # whether debugging is enabled
        self._gui = gui  # whether to show a live sumo-gui
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

    def get_predecessor_id(self, vid: int) -> int:
        position = self._vehicles[vid].position
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
            if predecessor_id is -1 or vehicle.position < self._vehicles[predecessor_id].position:
                predecessor_id = vehicle.vid
            # TODO throw error if precessor and vehicle are "interleaved"
        return predecessor_id

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
                # TODO decide upon and perform a lane change for this vehicle
                # check diff to desired speed
                # check distance to previous
                # determine whether it is useful to overtake
                # check adjacent lane is free

    def adjust_speeds(self):
        """Do speed adjustments for all vehicles"""

        for vehicle in self._vehicles.values():
            if vehicle.depart_time > self._step:
                # vehicle did not start yet
                continue

            pid = self.get_predecessor_id(vehicle.vid)
            if pid is -1:
                pred_pos_rear = self._road_length
            else:
                pred_pos_rear = self._vehicles[pid].rear_position

            vehicle._speed = new_speed(vehicle.speed, vehicle.desired_speed, vehicle.max_speed,
                                       vehicle.max_acceleration, vehicle.max_deceleration, vehicle.position,
                                       pred_pos_rear)

    # krauss - single lane traffic
    # adjust position (move)
    # x(t + step_size) = x(t) + v(t)*step_size
    def move_vehicles(self):
        """Do postion updates for all vehicles"""

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

        # TODO we kind of do not want collisions at all
        # either the cf model shouldn't allow collisions or we should move this to the move part
        for vehicle in self._vehicles.values():
            if vehicle.depart_time > self._step:
                # vehicle did not start yet
                continue
            # check for crashes of this vehicle with any other vehicle
            for other_vehicle in self._vehicles.values():
                if vehicle is other_vehicle:
                    # we do not need to compare us to ourselves
                    continue
                if other_vehicle.depart_time > self._step:
                    # other vehicle did not start yet
                    continue
                if vehicle.lane != other_vehicle.lane:
                    # we do not care about other lanes
                    continue
                if vehicle.position >= (other_vehicle.position - other_vehicle.length) and \
                        other_vehicle.position >= (vehicle.position - vehicle.length):
                    # vehicle is within the back of other_vehicle
                    print(self._step, ": crash", vehicle.vid, vehicle.position, vehicle.length,
                          other_vehicle.vid, other_vehicle.position, other_vehicle.length)
                    exit(1)

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
            random_desired_speed: bool,
            min_desired_speed: int,
            max_desired_speed: int):
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
            depart_position = position = randrange(0, self._road_length, depart_interval)
            depart_position = 0  # FIXME start from beginning for now
            depart_lane = 0
            depart_lane = randrange(0, self._number_of_lanes, 1)  # FIXME start on random lane for now
            if random_desired_speed:
                desired_speed = randrange(min_desired_speed, max_desired_speed, 1)
            else:
                desired_speed = max_speed
            depart_speed = randrange(0, desired_speed, 1)
            depart_speed = 0  # FIXME start with 0 speed for now
            arrival_position = randrange(position + 1, self._road_length, arrival_interval)
            arrival_position = self._road_length  # FIXME go to end for now
            spawn_strategy = "interval"
            if spawn_strategy is "interval":
                depart_time_interval = 1
                if last_vehicle_id is not -1:
                    depart_time = self._vehicles[last_vehicle_id].depart_time + depart_time_interval
                else:
                    depart_time = 0
            #elif spwan_strategy is "rate":
                # TODO
                # vehicles per hour
                # max_step / 3600 --> # hours
                # rate per hour / 3600 --> rate per second
                # random > rate? spawn
            #elif spawn_strategy is "probabilty":
                # TODO
            else:
                depart_time = randrange(0, max_step, 1 * 60)  # in which minute to start
            # safety_gap = 0  # m

            # choose vehicle "type" depending on the penetration rate
            if random() < penetration_rate:
                vehicle = PlatooningVehicle(
                    self,
                    vid,
                    vtype,
                    depart_position,
                    arrival_position,
                    desired_speed,
                    depart_speed,
                    depart_lane,
                    depart_time)
            else:
                vehicle = Vehicle(self, vid, vtype, depart_position, arrival_position,
                                  desired_speed, depart_speed, depart_lane, depart_time)

            self._vehicles[vid] = vehicle

            last_vehicle_id = vid

    def run(self, max_step: int):
        """Run the simulation with the specified parameters"""

        # write some general information about the simulation
        with open(self._result_base_filename + '_general.out', 'w') as f:
            f.write("simulation start: " + time.asctime(time.localtime(time.time())) + '\n')
            f.write("parameters" + str(self) + '\n')

        # create output file for vehicle trips
        with open(self._result_base_filename + '_vehicle_trips.csv', 'w') as f:
            f.write("id,depart,departLane,departPos,departSpeed,arrival,arrivalLane,arrivalPos,arrivalSpeed,duration,routeLength,timeLoss\n")

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
            sumoCmd = [sumoBinary, "-c", "cfg/freeway.sumo.cfg"]

            traci.start(sumoCmd)
            traci.simulationStep(self._step)
            from random import randrange

        # let the simulator run
        while True:
            if self._step >= max_step:
                self.stop("reached step limit")
            if len(self._vehicles) == 0:
                self.stop("no more vehicles in the simulation")  # do we really want to exit here?

            print("\rCurrent step: %d" % self._step, sep=' ', end='', flush=True)

            if self._gui:
                for vehicle in self._vehicles.values():
                    if vehicle.depart_time > self._step:
                        # vehicle did not start yet
                        continue
                    if str(vehicle.vid) not in traci.vehicle.getIDList():
                        traci.vehicle.add(str(vehicle.vid), 'route', departPos=str(vehicle.position), departSpeed=str(vehicle.speed), departLane=str(vehicle.lane), typeID='vehicle')
                        traci.vehicle.setColor(str(vehicle.vid), (randrange(0, 255, 1), randrange(0, 255, 1), randrange(0, 255, 1)))
                        traci.vehicle.setSpeedMode(str(vehicle.vid), 0)
                        traci.vehicle.setLaneChangeMode(str(vehicle.vid), 0)
                    traci.vehicle.setSpeed(str(vehicle.vid), vehicle.speed)
                    traci.vehicle.moveTo(vehID=str(vehicle.vid), pos=vehicle.position, laneID='edge_0_0_%d' % vehicle.lane)
                traci.simulationStep(self._step)

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

        print(self._step, ":", msg, flush=True)
        self.finish()
        exit(0)

    def __str__(self) -> str:
        """Return a nice string representation of a simulator instance"""

        return str(self.__dict__)

    def finish(self):
        """Clean up the simulation"""

        if self._gui:
            import traci
            traci.close(False)
        pass
