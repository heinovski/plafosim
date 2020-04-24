from random import randrange

# assumptions
# continoius driving speed
# you just reach your destination
# posx is in the middle of the front bumper
# a car ends at posx + length
# crash detection does not work with steps greater than 1
# linear velocity

# simulation properties
maxstep = 2 * 60 * 60  # s
step = 0  # s
step_length = 1  # s

# road network properties
road_length = 10 * 1000  # m
number_of_lanes = 4

# car properties
max_accel = 3  # m/s
max_deccel = -5  # m/s
safety_gap = 0  # m

# cars
number_of_cars = 100
last_vehicle_id = -1
cars = []


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
        print(self.vid, "is at", self.posx, self.lane, "with", self.speed)


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

    for car in cars:
        if car.start_time > step:
            # car did not start yet
            continue
        # destination reached?
        if car.posx >= car.destination:
            # posx does not match destination
            travelled_distance = car.posx - car.origin
            travelled_time = step - car.start_time
            print(step, ":", car.vid, "reached its destination", car.destination, travelled_distance, travelled_time)
            cars.remove(car)
            continue

        # the current status of the car
        car.stats()

        # TODO lane change

        # TODO adjust

        # do we need to adjust our speed?
        diff_to_desired = car.desired_speed - car.speed
        if diff_to_desired > 0:
            # we need to accelerate
            diff = min(diff_to_desired, max_accel)
        elif diff_to_desired < 0:
            # we need to deccelerate
            diff = max(diff_to_desired, max_deccel)
        else:
            # we are good
            diff = 0
        car.speed += diff

        # move

        # increase position according to speed
        car.posx += car.speed * step_length
        # TODO put destination check here?
        # TODO use diff to destination for increasing

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

    step += step_length
