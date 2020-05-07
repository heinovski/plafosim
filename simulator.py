import argparse

from random import randrange
from vehicle import Vehicle, VehicleType

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

    def __init__(self, road_length, number_of_lanes, collisions, step_length, debug):
        self.road_length = road_length
        self.number_of_lanes = number_of_lanes
        self.collisions = collisions
        self.step_length = step_length
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
                                      vehicle.max_acceleration(),
                                      vehicle.max_deceleration())

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
                if vehicle.position >= (other_vehicle.position - other_vehicle.length()) and \
                        other_vehicle.position >= (vehicle.position - vehicle.length()):
                    # vehicle is within the back of other_vehicle
                    print(self.step, ": crash", vehicle.vid, vehicle.position, vehicle.length(),
                          other_vehicle.vid, other_vehicle.position, other_vehicle.length())
                    exit(1)

    def generate_vehicles(self, max_step, number_of_vehicles, depart_interval, arrival_interval,
                          min_desired_speed, max_desired_speed, max_speed):
        last_vehicle_id = -1
        # vehicle properties
        length = randrange(4, 5 + 1, 1)
        max_acceleration = 3  # m/s
        max_deceleration = -5  # m/s
        vtype = VehicleType("car", length, max_speed, max_acceleration, max_deceleration)  # TODO multiple vtypes
        for num in range(0, number_of_vehicles):
            vid = last_vehicle_id + 1
            depart_position = position = randrange(0, self.road_length, depart_interval)
            depart_position = 0  # FIXME start from beginning for now
            depart_lane = 0
            depart_lane = randrange(0, self.number_of_lanes, 1)  # FIXME start on random lane for now
            desired_speed = randrange(min_desired_speed, max_desired_speed, 1)
            depart_speed = randrange(0, desired_speed, 1)
            depart_speed = 0  # FIXME start with 0 speed for now
            arrival_position = randrange(position + 1, self.road_length, arrival_interval)
            depart_time = randrange(0, max_step, 1 * 60)  # in which minute to start
            # safety_gap = 0  # m

            vehicle = Vehicle(self, vid, vtype, depart_position, arrival_position, desired_speed,
                              depart_speed, depart_lane, depart_time)
            self.vehicles.append(vehicle)

            last_vehicle_id = vid

    def run(self, max_step):
        # let the simulator run
        while 1:
            if self.step >= max_step:
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
            if self.collisions:
                self.check_collisions()

            self.step += self.step_length


def main():
    # parse some parameters
    parser = argparse.ArgumentParser(description="This is a simulator.")
    # road network properties
    parser.add_argument('--length', type=int, default=100, help="The length of the road in km (default 100)")
    parser.add_argument('--lanes', type=int, default=4, help="The number of lanes (default 4)")
    parser.add_argument('--depart-interval', type=int, default=1000,
                        help="The distance between departure positions (on-ramps) in m (default 1000)")
    parser.add_argument('--arrival-interval', type=int, default=1000,
                        help="The distance between arrival positions (off-ramps) in m (default 1000)")
    # vehicles
    parser.add_argument('--vehicles', type=int, default=100, help="The number of vehicles (default 100)")
    parser.add_argument('--min-desired-speed', type=int, default=22,
                        help="The minimum desired driving speed im m/s (default 22)")
    parser.add_argument('--max-desired-speed', type=int, default=28,
                        help="The minimum desired driving speed im m/s (default 28)")
    parser.add_argument('--max-speed', type=int, default=36,
                        help="The maximum possible driving speed in m/s (default 36)")
    parser.add_argument('--collisions', type=bool, default=True, help="Enable collision checks (default True)")
    # simulation properties
    parser.add_argument('--step', type=int, default=1, help="The step length in s (default 1)")
    parser.add_argument('--limit', type=int, default=100, help="The simulation limit in h (default 100)")
    parser.add_argument('--debug', type=bool, default=False, help="Enable debug output (default False)")
    args = parser.parse_args()

    simulator = Simulator(args.length * 1000, args.lanes, args.collisions,
                          args.step, args.debug)
    max_step = args.limit * 60 * 60
    simulator.generate_vehicles(max_step, args.vehicles, args.depart_interval, args.arrival_interval,
                                args.min_desired_speed, args.max_desired_speed, args.max_speed)
    simulator.run(max_step)


if __name__ == "__main__":
    main()
