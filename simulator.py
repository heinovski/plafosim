cars = [
    {
        'vid': 0,
        'posx': 0,  # m
        'lane': 0,
        'speed': 5,  # m/s
        'length': 4,
        'pid': 0,
        'dest': 6 * 1000,  # m
        'start': 0  # s
    },
    {
        'vid': 1,
        'posx': 0,  # m
        'lane': 0,
        'speed': 8,  # m/s
        'length': 5,
        'pid': 0,
        'dest': 4 * 1000,  # m
        'start': 10  # s
    }
]

maxstep = 1 * 60 * 60  # s
step = 0  # s
step_length = 1  # s
road_length = 100000  # m
number_of_lanes = 2
safety_gap = 0  # m

# assumptions
# continoius driving speed
# you just reach your destination
# posx is in the middle of the front bumper
# a car ends at posx + length
# crash detection does not work with steps greater than 1

while 1:
    if step >= maxstep:
        print("reached step limit")
        exit(0)
    if len(cars) == 0:
        print("no more cars in the simulation")
#        exit(0)  # do we really want to exit here?
    for car in cars:
        if car['start'] > step:
            # car did not start yet
            continue
        if car['posx'] >= car['dest']:
            print(step, ":", car['vid'], "reached its destination", car['dest'])
            cars.remove(car)
            continue
        print(step, ":", car['vid'], "is at", car['posx'], car['posx']-car['length'], car['lane'], "with", car['speed'])
        # increase position according to speed
        car['posx'] += car['speed'] * step_length
        # check for crashes of this car with any other car
        for other_car in cars:
            if car is other_car:
                # we do not need to compare us to ourselves
                continue
            if other_car['start'] > step:
                # other car did not start yet
                continue
            if car['lane'] is not other_car['lane']:
                # we do not car about other lanes
                continue
            if car['posx'] >= (other_car['posx'] - other_car['length']) and \
                    other_car['posx'] >= (car['posx'] - car['length']):
                # car is within the back of other_car
                print("crash", car['vid'], car['posx'], car['length'],
                      other_car['vid'], other_car['posx'], other_car['length'])
                exit(1)
    step += step_length
