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
class VehicleType:

    def __init__(self, name, length, max_speed, max_acceleration, max_deceleration):
        self._name = name  # the name of a vehicle type
        self._length = length  # the length of a vehicle type
        self._max_speed = max_speed  # the maximum speed of a vehicle type
        self._max_acceleration = max_acceleration  # the maximum acceleration of a vehicle type
        self._max_deceleration = max_deceleration  # the maximum deceleration of a vehicle type

    @property
    def length(self):
        return self._length

    @property
    def max_speed(self):
        return self._max_speed

    @property
    def max_acceleration(self):
        return self._max_acceleration

    @property
    def max_deceleration(self):
        return self._max_deceleration


class Vehicle:
    'A vehicle in the simulation'

    def __init__(self, simulator, vid, vehicle_type, depart_position, arrival_position, desired_speed, depart_lane,
                 depart_speed, depart_time):
        '''Initialize a vehicle'''

        # TODO documentation
        self._simulator = simulator

        self._vid = vid
        self._vehicle_type = vehicle_type
        # trip details
        self._depart_position = depart_position
        self._arrival_position = arrival_position
        self._desired_speed = desired_speed
        self._depart_lane = depart_lane
        self._depart_speed = depart_speed
        self._depart_time = depart_time
        # vehicle details
        self._position = self._depart_position
        self._lane = self._depart_lane
        self._speed = self._depart_speed
        # statistics
        self._co = 0
        self._co2 = 0
        self._hc = 0
        self._pmx = 0
        self._npx = 0
        self._fuel = 0

    @property
    def vid(self):
        return self._vid

    @property
    def length(self):
        return self._vehicle_type.length

    @property
    def max_speed(self):
        return self._vehicle_type.max_speed

    @property
    def max_acceleration(self):
        return self._vehicle_type.max_acceleration

    @property
    def max_deceleration(self):
        return self._vehicle_type.max_deceleration

    @property
    def depart_position(self):
        return self._depart_position

    @property
    def arrival_position(self):
        return self._arrival_position

    @property
    def desired_speed(self):
        return self._desired_speed

    @property
    def depart_lane(self):
        return self._depart_lane

    @property
    def depart_speed(self):
        return self._depart_speed

    @property
    def depart_time(self):
        return self._depart_time

    @property
    def position(self):
        return self._position

    @property
    def lane(self):
        return self._lane

    @property
    def speed(self):
        return self._speed

    @property
    def travel_distance(self):
        return self._position - self._depart_position

    @property
    def travel_time(self):
        return self._simulator.step - self._depart_time

    def info(self):
        '''Print info of a vehicle'''
        e_remaining_travel_time = round((self._arrival_position - self._position) / self._desired_speed)
        print(self._simulator.step, ":", self._vid, "at", self._position, self._lane, "with", self._speed,
              "takes", e_remaining_travel_time)

    def statistics(self):
        # TODO write proper statistics
        return

    def finish(self):
        """Record final statistics and do finishing stuff"""
        if (self._position < self._arrival_position):
            return

        e_travel_time = (self._arrival_position - self._depart_position) / self._desired_speed
        time_loss = self.travel_time - round(e_travel_time)
        travel_time_ratio = round(self.travel_time / e_travel_time, 2)

        print(self._simulator.step, ":", self._vid, "arrived", self._position, self._lane, "with", self._speed,
              "took", self.travel_time, self.travel_distance, time_loss, travel_time_ratio)

        # TODO write proper statistics
        trip_info = "id=%d depart=%d departLane=%d departPos=%d departSpeed=%d arrival=%d arrivalLane=%d arrivalPos=%d arrivalSpeed=%d duration=%d routeLength=%d timeLoss=%d" % (
            self._vid, self._depart_time, self._depart_lane, self._depart_position, self._depart_speed, self._simulator.step,
            self._lane, self._position, self._speed, self.travel_time, self.travel_distance, time_loss)
        emissions = "id=%d CO_abs=%d CO2_abs=%d HC_abs=%d PMx_abs=%d NOx_abs=%d fuel_abs=%d" % (
            self._vid, self._co, self._co2, self._hc, self._pmx, self._npx, self._fuel)

        with open("vehicles.out", 'a') as f:
            f.write(trip_info)
            f.write("\n")
            f.write(emissions)
            f.write("\n")

    def __str__(self):
        return str(self._dict_)

    def transmit(self, destination_vid, message):
        if isinstance(message, Message):
            # TODO use threading?
            if destination_vid == -1:
                # TODO make proper
                for vehicle in self._simulator._vehicles:
                    vehicle.receive(message)
            else:
                # TODO make proper
                self._simulator._vehicles[destination_vid].receive(message)
            return True
        else:
            # TODO raise exception
            print("error transmit")
            exit(1)

    def receive(self, message):
        if self._simulator.step < self._depart_time:
            # we cannot receive anything since we did not start yet
            return False
        if isinstance(message, Message):
            if message.destination == self._vid or message.destination == -1:
                print(message)
                return True
            # we cannot receive this message since it was not for us
            return False
        else:
            # TODO raise exception
            print("error receive")
            exit(1)


class PlatooningVehicle(Vehicle):
    """A vehicle that has platooning functionality enabled"""

    def __init__(self, simulator, vid, vehicle_type, depart_position, arrival_position, desired_speed, depart_lane,
                 depart_speed, depart_time):
        super().__init__(simulator, vid, vehicle_type, depart_position, arrival_position, desired_speed, depart_lane,
                         desired_speed, depart_time)
