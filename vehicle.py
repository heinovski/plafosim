class VehicleType:

    def __init__(self, name, length, max_speed, max_acceleration, max_deceleration):
        self.name = name
        self.length = length
        self.max_speed = max_speed
        self.max_acceleration = max_acceleration
        self.max_deceleration = max_deceleration


class Vehicle:
    'A vehicle in the simulation'

    simulator = None
    vid = -1
    vehicle_type = None
    depart_position = -1
    arrival_position = -1
    desired_speed = -1
    depart_lane = -1
    depart_speed = -1
    depart_time = -1
    position = -1
    lane = -1
    speed = -1
    # statistics
    co = 0
    co2 = 0
    hc = 0
    pmx = 0
    npx = 0
    fuel = 0

    def __init__(self, simulator, vid, vehicle_type, depart_position, arrival_position, desired_speed, depart_lane,
                 depart_speed, depart_time):
        '''Initialize a vehicle'''
        self.simulator = simulator

        self.vid = vid
        self.vehicle_type = vehicle_type
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

    def length(self):
        return self.vehicle_type.length

    def max_speed(self):
        return self.vehicle_type.max_speed

    def max_acceleration(self):
        return self.vehicle_type.max_acceleration

    def max_deceleration(self):
        return self.vehicle_type.max_deceleration

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

        e_travel_time = (self.arrival_position - self.depart_position) / self.desired_speed
        time_loss = self.travel_time() - round(e_travel_time)
        travel_time_ratio = round(self.travel_time() / e_travel_time, 2)

        print(self.simulator.step, ":", self.vid, "arrived", self.position, self.lane, "with", self.speed,
              "took", self.travel_time(), self.travel_distance(), time_loss, travel_time_ratio)

        # TODO write proper statistics
        trip_info = "id=%d depart=%d departLane=%d departPos=%d departSpeed=%d arrival=%d arrivalLane=%d arrivalPos=%d arrivalSpeed=%d duration=%d routeLength=%d timeLoss=%d" % (
            self.vid, self.depart_time, self.depart_lane, self.depart_position, self.depart_speed, self.simulator.step,
            self.lane, self.position, self.speed, self.travel_time(), self.travel_distance(), time_loss)
        emissions = "id=%d CO_abs=%d CO2_abs=%d HC_abs=%d PMx_abs=%d NOx_abs=%d fuel_abs=%d" % (
            self.vid, self.co, self.co2, self.hc, self.pmx, self.npx, self.fuel)

        with open("vehicles.out", 'a') as f:
            f.write(trip_info)
            f.write("\n")
            f.write(emissions)
            f.write("\n")
