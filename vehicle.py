class VehicleType:

    def __init__(self, name, length, max_speed, max_acceleration, max_deceleration):
        self.name = name
        self.length = length
        self.max_speed = max_speed
        self.max_acceleration = max_acceleration
        self.max_deceleration = max_deceleration


class Vehicle:
    'A vehicle in the simulation'

    __simulator = None
    __vid = -1
    __vehicle_type = None
    __depart_position = -1
    __arrival_position = -1
    __desired_speed = -1
    __depart_lane = -1
    __depart_speed = -1
    __depart_time = -1
    __position = -1
    __lane = -1
    __speed = -1
    # statistics
    __co = 0
    __co2 = 0
    __hc = 0
    __pmx = 0
    __npx = 0
    __fuel = 0

    def __init__(self, simulator, vid, vehicle_type, depart_position, arrival_position, desired_speed, depart_lane,
                 depart_speed, depart_time):
        '''Initialize a vehicle'''
        self.__simulator = simulator

        self.__vid = vid
        self.__vehicle_type = vehicle_type
        # trip details
        self.__depart_position = depart_position
        self.__arrival_position = arrival_position
        self.__desired_speed = desired_speed
        self.__depart_lane = depart_lane
        self.__depart_speed = depart_speed
        self.__depart_time = depart_time
        # vehicle details
        self.__position = self.__depart_position
        self.__lane = self.__depart_lane
        self.__speed = self.__depart_speed

    def vid(self):
        return self.__vid

    def length(self):
        return self.__vehicle_type.length

    def max_speed(self):
        return self.__vehicle_type.max_speed

    def max_acceleration(self):
        return self.__vehicle_type.max_acceleration

    def max_deceleration(self):
        return self.__vehicle_type.max_deceleration

    def depart_position(self):
        return self.__depart_position

    def arrival_position(self):
        return self.__arrival_position

    def desired_speed(self):
        return self.__desired_speed

    def depart_lane(self):
        return self.__depart_lane

    def depart_speed(self):
        return self.__depart_speed

    def depart_time(self):
        return self.__depart_time

    def position(self):
        return self.__position

    def lane(self):
        return self.__lane

    def speed(self):
        return self.__speed

    def travel_distance(self):
        return self.__position - self.__depart_position

    def travel_time(self):
        return self.__simulator.step - self.__depart_time

    def info(self):
        '''Print info of a vehicle'''
        e_remaining_travel_time = round((self.__arrival_position - self.__position) / self.__desired_speed)
        print(self.__simulator.step, ":", self.__vid, "at", self.__position, self.__lane, "with", self.__speed,
              "takes", e_remaining_travel_time)

    def statistics(self):
        # TODO write proper statistics
        return

    def __str__(self):
        return str(self.__dict__)

    def __del__(self):
        if (self.__position < self.__arrival_position):
            return

        e_travel_time = (self.__arrival_position - self.__depart_position) / self.__desired_speed
        time_loss = self.travel_time() - round(e_travel_time)
        travel_time_ratio = round(self.travel_time() / e_travel_time, 2)

        print(self.__simulator.step, ":", self.__vid, "arrived", self.__position, self.__lane, "with", self.__speed,
              "took", self.travel_time(), self.travel_distance(), time_loss, travel_time_ratio)

        # TODO write proper statistics
        trip_info = "id=%d depart=%d departLane=%d departPos=%d departSpeed=%d arrival=%d arrivalLane=%d arrivalPos=%d arrivalSpeed=%d duration=%d routeLength=%d timeLoss=%d" % (
            self.__vid, self.__depart_time, self.__depart_lane, self.__depart_position, self.__depart_speed, self.__simulator.step,
            self.__lane, self.__position, self.__speed, self.travel_time(), self.travel_distance(), time_loss)
        emissions = "id=%d CO_abs=%d CO2_abs=%d HC_abs=%d PMx_abs=%d NOx_abs=%d fuel_abs=%d" % (
            self.__vid, self.__co, self.__co2, self.__hc, self.__pmx, self.__npx, self.__fuel)

        with open("vehicles.out", 'a') as f:
            f.write(trip_info)
            f.write("\n")
            f.write(emissions)
            f.write("\n")
