from random import randrange

# assumptions
# continoius driving speed
# you just reach your destination
# posx is in the middle of the front bumper
# a car ends at posx + length
# crash detection does not work with steps greater than 1
# linear velocity

# simulation properties
maxstep = 100 * 60 * 60  # s
step = 0  # s
step_length = 1  # s

# road network properties
road_length = 10 * 1000  # m
number_of_lanes = 1

# car properties
max_accel = 3  # m/s
max_deccel = -5  # m/s
safety_gap = 0  # m

# cars
number_of_cars = 10
last_vehicle_id = -1
cars = []


def change_lanes():
    for car in cars:
        if car.start_time > step:
            # car did not start yet
            continue
        # TODO


def adjust_speeds():
    for car in cars:
        if car.start_time > step:
            # car did not start yet
            continue
        car.speed = new_speed(car.speed, car.desired_speed, max_accel, max_deccel)


def move_vehicles():
    for car in cars:
        if car.start_time > step:
            # car did not start yet
            continue
        # increase position according to speed
        car.posx += car.speed * step_length
        # TODO use diff to destination for increasing

        # destination reached?
        if car.posx >= car.destination:
            # posx does not match destination
            travelled_distance = car.posx - car.origin
            travelled_time = step - car.start_time
            print(step, ":", car.vid, "reached its destination", car.destination, travelled_distance, travelled_time)
            cars.remove(car)
            continue


def check_collisions():
    # TODO we kind of do not want collissions at all
    # either the cf model shouldn't allow collisions or we should move this to the move part
    for car in cars:
        if car.start_time > step:
            # car did not start yet
            continue
        # check for crashes of this car with any other car
        for other_car in cars:
            if car is other_car:
                # we do not need to compare us to ourselves
                continue
            if other_car.start_time > step:
                # other car did not start yet
                continue
            if car.lane is not other_car.lane:
                # we do not car about other lanes
                continue
            if car.posx >= (other_car.posx - other_car.length) and \
                    other_car.posx >= (car.posx - car.length):
                # car is within the back of other_car
                print("crash", car.vid, car.posx, car.length,
                      other_car.vid, other_car.posx, other_car.length)
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


def new_speed(current_speed, desired_speed, max_accel, max_decel):
    new_speed = -1
    # do we need to adjust our speed?
    diff_to_desired = desired_speed - current_speed
    if diff_to_desired > 0:
        # we need to accelerate
        new_speed = current_speed + min(diff_to_desired, max_accel)
    elif diff_to_desired < 0:
        # we need to deccelerate
        new_speed = current_speed - max(diff_to_desired, max_deccel)

    # TODO vsafe?

    # TODO dawdling?
    # new_speed -= random() * max_

    if (new_speed < 0):
        new_speed = 0

    return new_speed


class Vehicle:
    'A vehicle in the simulation'

    def __init__(self, vid, origin, destination, desired_speed, length, start_time):
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

    def stats(self):
        '''Print stats of a vehicle'''
        print(step, ":", self.vid, "is at", self.posx, self.lane, "with", self.speed)


# generate cars
for num in range(0, number_of_cars):
    vid = last_vehicle_id + 1
    origin = posx = randrange(0, road_length, 1 * 1000)  # on-ramps every 1000 m
    origin = posx = 0  # start from beginning
    desired_speed = randrange(22, 28, 1)
    length = randrange(4, 5 + 1, 1)
    dest = randrange(posx, road_length, 1 * 1000)  # off-ramps every 1000 m
    start = randrange(0, maxstep, 1 * 60)  # in which minute to start

    cars.append(Vehicle(vid, origin, dest, desired_speed, length, start))

    last_vehicle_id = vid

while 1:
    if step >= maxstep:
        print("reached step limit")
        exit(0)
    if len(cars) == 0:
        print("no more cars in the simulation")
        exit(0)  # do we really want to exit here?

    # stats
    for car in cars:
        if car.start_time == step:
            car.stats()
        if car.start_time > step:
            # car did not start yet
            continue
        # the current status of the car
    #    car.stats()

    # perform lane changes (for all vehicles)
    change_lanes()

    # adjust speed (of all cars)
    adjust_speeds()

    # adjust positions (of all cars)
    move_vehicles()

    # do collision check (for all cars)
    check_collisions()

    step += step_length
