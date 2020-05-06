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
