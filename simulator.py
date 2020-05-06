import argparse

from random import randrange

# assumptions
# you just reach your arrival_position
# position is in the middle of the front bumper
# a vehicle ends at position + length
# crash detection does not work with steps greater than 1

# parse some parameters
parser = argparse.ArgumentParser(description="This is a simulator.")
# road network properties
parser.add_argument('--length', type=int, default=100, help="The length of the road in km (default 100)")
parser.add_argument('--lanes', type=int, default=4, help="The number of lanes (default 4)")
# vehicles
parser.add_argument('--vehicles', type=int, default=100, help="The number of vehicles (default 100)")
# simulation properties
parser.add_argument('--step', type=int, default=1, help="The step length in s (default 1)")
args = parser.parse_args()


class Vehicle:
    'A vehicle in the simulation'

    def __init__(self, simulator, vid, depart_position, arrival_position, desired_speed, depart_lane,
                 depart_speed, depart_time, length, max_acceleration, max_deceleration):
        '''Initialize a vehicle'''
        self.simulator = simulator

        self.vid = vid
        # trip details
        self.depart_position = depart_position
        self.arrival_position = arrival_position
        self.desired_speed = desired_speed
        self.depart_lane = depart_lane
        self.depart_speed = depart_speed
        self.depart_time = depart_time
        # vehicle details
        self.position = self.depart_position
        self.lane = self.depart_lane
        self.speed = self.depart_speed
        self.length = length
        self.max_acceleration = max_acceleration
        self.max_deceleration = max_deceleration
        # statistics
        self.co = 0
        self.co2 = 0
        self.hc = 0
        self.pmx = 0
        self.npx = 0
        self.fuel = 0

    def travel_distance(self):
        return self.position - self.depart_position

    def travel_time(self):
        return self.simulator.step - self.depart_time

    def info(self):
        '''Print info of a vehicle'''
        e_remaining_travel_time = round((self.arrival_position - self.position) / self.desired_speed)
        print(self.simulator.step, ":", self.vid, "at", self.position, self.lane, "with", self.speed,
              "takes", e_remaining_travel_time)

    def statistics(self):
        # TODO write proper statistics
        return

    def __str__(self):
        return str(self.__dict__)

    def __del__(self):
        if (self.position < self.arrival_position):
            return

        e_travel_time = round((self.arrival_position - self.depart_position) / self.desired_speed)
        time_loss = self.travel_time() - e_travel_time
        try:
            travel_time_ratio = round(self.travel_time() / e_travel_time, 2)  # FIXME prodcues a division by 0 error after a crash happened
        except ZeroDivisionError:
            print("ZeroDivisionError")
            print(self)
            quit(1)

        print(self.simulator.step, ":", self.vid, "arrived", self.position, self.lane, "with", self.speed,
              "took", self.travel_time(), self.travel_distance(), time_loss, travel_time_ratio)

        # TODO write proper statistics
        trip_info = "id=%d depart=%d departLane=%d departPos=%d departSpeed=%d departDelay=%d arrival=%d arrivalLane=%d arrivalPos=%d arrivalSpeed=%d duration=%d routeLength=%d timeLoss=%d" % (self.vid, self.depart_time, self.depart_lane, self.depart_position, self.depart_speed, -1, self.simulator.step, self.lane, self.position, self.speed, self.travel_time(), self.travel_distance(), time_loss)
        print(trip_info)

        emissions = "CO_abs=%d CO2_abs=%d HC_abs=%d PMx_abs=%d NOx_abs=%d fuel_abs=%d" % (self.co, self.co2, self.hc, self.pmx, self.npx, self.fuel)
        print(emissions)


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
def new_speed(current_speed, desired_speed, max_acceleration, max_deceleration):
    new_speed = -1
    # do we need to adjust our speed?
    diff_to_desired = desired_speed - current_speed
    if diff_to_desired > 0:
        # we need to accelerate
        new_speed = current_speed + min(diff_to_desired, max_acceleration)
    elif diff_to_desired < 0:
        # we need to decelerate
        new_speed = current_speed - max(diff_to_desired, max_deceleration)
    else:
        new_speed = current_speed

    # TODO vsafe?

    # TODO dawdling?
    # new_speed -= random() * max_

    if (new_speed < 0):
        new_speed = 0

    return new_speed


class Simulator:

    def __init__(self, road_length, number_of_lanes, number_of_vehicles, step_length, max_step, debug):
        self.road_length = road_length
        self.number_of_lanes = number_of_lanes
        self.number_of_vehicles = number_of_vehicles
        self.step_length = step_length
        self.max_step = max_step
        self.debug = debug

        self.step = 0  # s
        self.vehicles = []

    def record_stats(self):
        for vehicle in self.vehicles:
            if vehicle.depart_time > self.step:
                # vehicle did not start yet
                continue
            elif vehicle.depart_time == self.step:
                vehicle.info()
            elif self.debug is True:
                # the current status of the vehicle
                print(vehicle)

            # log periodic statistics
            vehicle.statistics()

    # kraus - multi lane traffic
    # lane-change
    # congested = (v_safe < v_thresh) and (v^0_safe < v_thresh)
    # favorable(right->left) = (v_safe < v_max) and (not congested)
    # favorable(left->right) = (v_safe >= v_max) and (v^0_safe >= v_max)
    # if ((favorable(i->j) or (rand < p_change)) and safe(i->j)) then change(i->j)
    # for vehicles on the right lane:
    # if (v > v^0_safe) and (not congested) then v <- v^0_safe
    def change_lanes(self):
        for vehicle in self.vehicles:
            if vehicle.depart_time > self.step:
                # vehicle did not start yet
                continue
            # TODO

    def adjust_speeds(self):
        for vehicle in self.vehicles:
            if vehicle.depart_time > self.step:
                # vehicle did not start yet
                continue
            vehicle.speed = new_speed(vehicle.speed, vehicle.desired_speed,
                                      vehicle.max_acceleration, vehicle.max_deceleration)

    # krauss - single lane traffic
    # adjust position (move)
    # x(t + step_size) = x(t) + v(t)*step_size
    def move_vehicles(self):
        for vehicle in self.vehicles:
            if vehicle.depart_time > self.step:
                # vehicle did not start yet
                continue
            # increase position according to speed
            position_difference = vehicle.speed * self.step_length
            # arrival_position reached?
            if vehicle.position + position_difference >= vehicle.arrival_position:
                vehicle.position = vehicle.arrival_position
                self.vehicles.remove(vehicle)
                continue
            else:
                vehicle.position += position_difference

    def check_collisions(self):
        # TODO we kind of do not want collisions at all
        # either the cf model shouldn't allow collisions or we should move this to the move part
        for vehicle in self.vehicles:
            if vehicle.depart_time > self.step:
                # vehicle did not start yet
                continue
            # check for crashes of this vehicle with any other vehicle
            for other_vehicle in self.vehicles:
                if vehicle is other_vehicle:
                    # we do not need to compare us to ourselves
                    continue
                if other_vehicle.depart_time > self.step:
                    # other vehicle did not start yet
                    continue
                if vehicle.lane is not other_vehicle.lane:
                    # we do not care about other lanes
                    continue
                if vehicle.position >= (other_vehicle.position - other_vehicle.length) and \
                        other_vehicle.position >= (vehicle.position - vehicle.length):
                    # vehicle is within the back of other_vehicle
                    print("crash", vehicle.vid, vehicle.position, vehicle.length,
                          other_vehicle.vid, other_vehicle.position, other_vehicle.length)
                    exit(1)

    def generte_vehicles(self):
        last_vehicle_id = -1
        for num in range(0, self.number_of_vehicles):
            vid = last_vehicle_id + 1
            depart_position = position = randrange(0, self.road_length, 1 * 1000)  # on-ramps every 1000 m
            depart_position = 0  # start from beginning for now
            depart_lane = randrange(0, self.number_of_lanes, 1)
        #   depart_lane = 0  # start on lane 0 for now
            depart_speed = randrange(0, 28, 1)
            depart_speed = 0  # start with 0 speed for now
            desired_speed = randrange(22, 28, 1)
            arrival_position = randrange(position + 1, self.road_length, 1 * 1000)  # off-ramps every 1000 m
            depart_time = randrange(0, self.max_step, 1 * 60)  # in which minute to start
            # vehicle properties
            length = randrange(4, 5 + 1, 1)
            max_acceleration = 3  # m/s
            max_deceleration = -5  # m/s
            # safety_gap = 0  # m

            self.vehicles.append(Vehicle(self, vid, depart_position, arrival_position, desired_speed, depart_speed, depart_lane, depart_time, length, max_acceleration, max_deceleration))

            last_vehicle_id = vid

    def run(self):
        # let the simulator run
        while 1:
            if self.step >= self.max_step:
                print("reached step limit")
                exit(0)
            if len(self.vehicles) == 0:
                print("no more vehicles in the simulation")
                exit(0)  # do we really want to exit here?

            # stats
            self.record_stats()

            # perform lane changes (for all vehicles)
            self.change_lanes()

            # adjust speed (of all vehicles)
            self.adjust_speeds()

            # adjust positions (of all vehicles)
            self.move_vehicles()

            # do collision check (for all vehicles)
            self.check_collisions()

            self.step += self.step_length


def main():
    simulator = Simulator(args.length * 1000, args.lanes, args.vehicles, args.step, 100 * 60 * 60, False)
    simulator.generte_vehicles()
    simulator.run()


if __name__ == "__main__":
    main()
