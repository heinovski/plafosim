from random import randrange

# assumptions
# you just reach your destination
# posx is in the middle of the front bumper
# a vehicle ends at posx + length
# crash detection does not work with steps greater than 1

# simulation properties
maxstep = 100 * 60 * 60  # s
step = 0  # s
step_length = 1  # s

# road network properties
road_length = 10 * 1000  # m
number_of_lanes = 1

# vehicles
number_of_vehicles = 10
vehicles = []


def record_stats():
    for vehicle in vehicles:
        if vehicle.start_time == step:
            vehicle.stats()
        if vehicle.start_time > step:
            # vehicle did not start yet
            continue
        # the current status of the vehicle
    #    vehicle.stats()


def change_lanes():
    for vehicle in vehicles:
        if vehicle.start_time > step:
            # vehicle did not start yet
            continue
        # TODO


def adjust_speeds():
    for vehicle in vehicles:
        if vehicle.start_time > step:
            # vehicle did not start yet
            continue
        vehicle.speed = new_speed(vehicle.speed, vehicle.desired_speed,
                                  vehicle.max_acceleration, vehicle.max_deceleration)


def move_vehicles():
    for vehicle in vehicles:
        if vehicle.start_time > step:
            # vehicle did not start yet
            continue
        # increase position according to speed
        vehicle.posx += vehicle.speed * step_length
        # TODO use diff to destination for increasing

        # destination reached?
        if vehicle.posx >= vehicle.destination:
            # posx does not match destination
            travel_distance = vehicle.posx - vehicle.origin
            travel_time = step - vehicle.start_time
            print(step, ":", vehicle.vid, "reached destination", vehicle.destination, travel_distance, travel_time)
            vehicles.remove(vehicle)
            continue


def check_collisions():
    # TODO we kind of do not want collisions at all
    # either the cf model shouldn't allow collisions or we should move this to the move part
    for vehicle in vehicles:
        if vehicle.start_time > step:
            # vehicle did not start yet
            continue
        # check for crashes of this vehicle with any other vehicle
        for other_vehicle in vehicles:
            if vehicle is other_vehicle:
                # we do not need to compare us to ourselves
                continue
            if other_vehicle.start_time > step:
                # other vehicle did not start yet
                continue
            if vehicle.lane is not other_vehicle.lane:
                # we do not vehicle about other lanes
                continue
            if vehicle.posx >= (other_vehicle.posx - other_vehicle.length) and \
                    other_vehicle.posx >= (vehicle.posx - vehicle.length):
                # vehicle is within the back of other_vehicle
                print("crash", vehicle.vid, vehicle.posx, vehicle.length,
                      other_vehicle.vid, other_vehicle.posx, other_vehicle.length)
                exit(1)


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
#
# adjust position (move)
# x(t + step_size) = x(t) + v(t)*step_size


# kraus - multi lane traffic
# lane-change
# congested = (v_safe < v_thresh) and (v^0_safe < v_thresh)
# favorable(right->left) = (v_safe < v_max) and (not congested)
# favorable(left->right) = (v_safe >= v_max) and (v^0_safe >= v_max)
# if ((favorable(i->j) or (rand < p_change)) and safe(i->j)) then change(i->j)
# for vehicles on the right lane:
# if (v > v^0_safe) and (not congested) then v <- v^0_safe
# then adjust
# then move


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

    # TODO vsafe?

    # TODO dawdling?
    # new_speed -= random() * max_

    if (new_speed < 0):
        new_speed = 0

    return new_speed


class Vehicle:
    'A vehicle in the simulation'

    def __init__(self, vid, origin, destination, desired_speed, start_time, length, max_acceleration, max_deceleration):
        '''Initialize a vehicle'''
        self.vid = vid
        # trip details
        self.origin = origin
        self.destination = destination
        self.desired_speed = desired_speed
        self.start_time = start_time
        # vehicle details
        self.posx = self.origin
        self.lane = 0
        self.speed = randrange(0, 28, 1)
        self.speed = 0  # start with 0 speed for now
        self.length = length
        self.max_acceleration = max_acceleration
        self.max_deceleration = max_deceleration

    def stats(self):
        '''Print stats of a vehicle'''
        print(step, ":", self.vid, "is at", self.posx, self.lane, "with", self.speed)


# generate vehicles
last_vehicle_id = -1
for num in range(0, number_of_vehicles):
    vid = last_vehicle_id + 1
    origin = posx = randrange(0, road_length, 1 * 1000)  # on-ramps every 1000 m
    origin = posx = 0  # start from beginning
    desired_speed = randrange(22, 28, 1)
    dest = randrange(posx, road_length, 1 * 1000)  # off-ramps every 1000 m
    start = randrange(0, maxstep, 1 * 60)  # in which minute to start
    # vehicle properties
    length = randrange(4, 5 + 1, 1)
    max_acceleration = 3  # m/s
    max_deceleration = -5  # m/s
    # safety_gap = 0  # m

    vehicles.append(Vehicle(vid, origin, dest, desired_speed, start, length,  max_acceleration, max_deceleration))

    last_vehicle_id = vid


# let the simulator run
while 1:
    if step >= maxstep:
        print("reached step limit")
        exit(0)
    if len(vehicles) == 0:
        print("no more vehicles in the simulation")
        exit(0)  # do we really want to exit here?

    # stats
    record_stats()

    # perform lane changes (for all vehicles)
    change_lanes()

    # adjust speed (of all vehicles)
    adjust_speeds()

    # adjust positions (of all vehicles)
    move_vehicles()

    # do collision check (for all vehicles)
    check_collisions()

    step += step_length
