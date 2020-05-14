#
# Copyright (c) 2020 Julian Heinovski <heinovski@ccs-labs.org>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
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
        self.name = name
        self.length = length
        self.max_speed = max_speed
        self.max_acceleration = max_acceleration
        self.max_deceleration = max_deceleration


class Vehicle:
    'A vehicle in the simulation'

    _simulator = None  # invalid
    _vid = -1  # invalid
    _vehicle_type = None  # invalid
    _depart_position = -1  # invalid
    _arrival_position = -1  # invalid
    _desired_speed = -1  # invalid
    _depart_lane = -1  # invalid
    _depart_speed = -1  # invalid
    _depart_time = -1  # invalid
    _position = -1  # invalid
    _lane = -1  # invalid
    _speed = -1  # invalid
    # statistics
    _co = 0
    _co2 = 0
    _hc = 0
    _pmx = 0
    _npx = 0
    _fuel = 0

    def __init__(self, simulator, vid, vehicle_type, depart_position, arrival_position, desired_speed, depart_lane,
                 depart_speed, depart_time):
        '''Initialize a vehicle'''
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

    def vid(self):
        return self._vid

    def length(self):
        return self._vehicle_type.length

    def max_speed(self):
        return self._vehicle_type.max_speed

    def max_acceleration(self):
        return self._vehicle_type.max_acceleration

    def max_deceleration(self):
        return self._vehicle_type.max_deceleration

    def depart_position(self):
        return self._depart_position

    def arrival_position(self):
        return self._arrival_position

    def desired_speed(self):
        return self._desired_speed

    def depart_lane(self):
        return self._depart_lane

    def depart_speed(self):
        return self._depart_speed

    def depart_time(self):
        return self._depart_time

    def position(self):
        return self._position

    def lane(self):
        return self._lane

    def speed(self):
        return self._speed

    def travel_distance(self):
        return self._position - self._depart_position

    def travel_time(self):
        return self._simulator.step() - self._depart_time

    def info(self):
        '''Print info of a vehicle'''
        e_remaining_travel_time = round((self._arrival_position - self._position) / self._desired_speed)
        print(self._simulator.step(), ":", self._vid, "at", self._position, self._lane, "with", self._speed,
              "takes", e_remaining_travel_time)

    def statistics(self):
        # TODO write proper statistics
        return

    def __str__(self):
        return str(self._dict_)

    def __del__(self):
        if (self._position < self._arrival_position):
            return

        e_travel_time = (self._arrival_position - self._depart_position) / self._desired_speed
        time_loss = self.travel_time() - round(e_travel_time)
        travel_time_ratio = round(self.travel_time() / e_travel_time, 2)

        print(self._simulator.step(), ":", self._vid, "arrived", self._position, self._lane, "with", self._speed,
              "took", self.travel_time(), self.travel_distance(), time_loss, travel_time_ratio)

        # TODO write proper statistics
        trip_info = "id=%d depart=%d departLane=%d departPos=%d departSpeed=%d arrival=%d arrivalLane=%d arrivalPos=%d arrivalSpeed=%d duration=%d routeLength=%d timeLoss=%d" % (
            self._vid, self._depart_time, self._depart_lane, self._depart_position, self._depart_speed, self._simulator.step(),
            self._lane, self._position, self._speed, self.travel_time(), self.travel_distance(), time_loss)
        emissions = "id=%d CO_abs=%d CO2_abs=%d HC_abs=%d PMx_abs=%d NOx_abs=%d fuel_abs=%d" % (
            self._vid, self._co, self._co2, self._hc, self._pmx, self._npx, self._fuel)

        with open("vehicles.out", 'a') as f:
            f.write(trip_info)
            f.write("\n")
            f.write(emissions)
            f.write("\n")
