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

import logging

from .vehicle_type import VehicleType
from .message import Message

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .simulator import Simulator

LOG = logging.getLogger(__name__)


class Vehicle:
    """A collection of state information for a vehicle in the simulation

    A vehicle can really be anything that can move and can be defined by a vehicle type.
    It does not necessarily be driven by a computer (i.e., autonomous).
    However, by default it does have V2X functionality."""

    def __init__(
            self,
            simulator: 'Simulator',
            vid: int,
            vehicle_type: VehicleType,
            depart_position: int,
            arrival_position: int,
            desired_speed: float,
            depart_lane: int,
            depart_speed: float,
            depart_time: int,
            communication_range: float):
        """Initialize a vehicle instance"""

        self._simulator = simulator  # the simulator
        self._started = False  # flag indicating whether the vehicles has actually started

        self._vid = vid  # the id of the vehicle
        self._vehicle_type = vehicle_type  # the vehicle type of the vehicle
        # trip details
        self._depart_position = depart_position  # the departure position of the vehicle
        self._arrival_position = arrival_position  # the arrival position of the vehicle
        self._desired_speed = desired_speed  # the desired driving speed of the vehicle
        self._depart_lane = depart_lane  # the departure lane of the vehicle
        self._depart_speed = depart_speed  # the departure speed of the vehicle
        self._depart_time = depart_time  # the departure time of the vehicle
        # vehicle details
        self._position = self._depart_position  # the current position of the vehicle
        self._lane = self._depart_lane  # the current lane of the vehicle
        self._speed = self._depart_speed  # the current speed of the vehicle
        self._blocked_front = False  # whether the vehicle is blocked by a slower vehicle in front
        self._acceleration = 0  # the current acceleration of the vehicle

        # communication properties
        # TODO move to platooning vehicle
        self._communication_range = communication_range  # the maximum communication range between two vehicles

        # statistics
        self._emissions = {
            'co': 0,  # the total co emission in mg
            'co2': 0,  # the total co2 emission in mg
            'hc': 0,  # the total hc emission in mg
            'pmx': 0,  # the total pmx emission in mg
            'nox': 0,  # the total nox emission in mg
            'fuel': 0  # the total fuel consumption in ml
        }

    @property
    def vid(self) -> int:
        return self._vid

    @property
    def vehicle_type(self) -> VehicleType:
        return self._vehicle_type

    @property
    def length(self) -> int:
        return self._vehicle_type.length

    @property
    def max_speed(self) -> float:
        return self._vehicle_type.max_speed

    @property
    def max_acceleration(self) -> float:
        return self._vehicle_type.max_acceleration

    @property
    def max_deceleration(self) -> float:
        return self._vehicle_type.max_deceleration

    @property
    def min_gap(self) -> float:
        return self._vehicle_type.min_gap

    @property
    def desired_headway_time(self) -> float:
        return self._vehicle_type.desired_headway_time

    @property
    def depart_position(self) -> int:
        return self._depart_position

    @property
    def arrival_position(self) -> int:
        return self._arrival_position

    @property
    def desired_speed(self) -> float:
        return self._desired_speed

    @property
    def desired_gap(self) -> float:
        return self._simulator.speed2distance(self.desired_headway_time * self.speed, self._simulator._step_length)

    @property
    def depart_lane(self) -> int:
        return self._depart_lane

    @property
    def depart_speed(self) -> float:
        return self._depart_speed

    @property
    def depart_time(self) -> int:
        return self._depart_time

    @property
    def position(self) -> float:
        return self._position

    @property
    def rear_position(self) -> int:
        # return max(self.position - self.length, 0)  # is slower than if-else
        position = self.position - self.length
        return 0 if position < 0 else position

    @property
    def lane(self) -> int:
        return self._lane

    @property
    def speed(self) -> float:
        return self._speed

    @property
    def acceleration(self) -> int:
        return self._acceleration

    @property
    def travel_distance(self) -> float:
        return self.position - self.depart_position

    @property
    def travel_time(self) -> float:
        return self._simulator.step - self.depart_time

    @property
    def blocked_front(self) -> bool:
        return self._blocked_front

    # krauss - single lane traffic
    # v_safe(t) = v_lead(t) + (g(t)-g_des(t)) / (tau_b + tau)
    # this is a simple and dumb calculation for the safe speed of a vehicle based on the positions of the predecessor and the vehicle itself
    def _safe_speed(self, speed_predecessor: float, gap_to_predecessor: float, desired_gap: float = 0, min_gap: float = 0) -> float:
        speed_diff_to_use = speed_predecessor - self.speed  # use to drive the same speed
        position_diff_to_use = gap_to_predecessor - max(desired_gap, min_gap)  # use to close the gap
        return speed_diff_to_use + self._simulator.distance2speed(position_diff_to_use, self.desired_headway_time)

    # krauss - single lane traffic
    # adjust speed
    # v_max, desired speed
    # epsilon, dawdling of drives
    # g_des = tau*v_lead
    # tau, reaction time of drivers
    # tau_b = v/b
    # v_des(t) = min[v_max, v(t)+a(v)*step_size, v_safe(t)]
    # v(t + step_size) = max[0, v_des(t) - epsilon]
    def new_speed(self, speed_predecessor: float, predecessor_rear_position: float) -> float:
        """Calculate the new speed for a vehicle using the kraus model"""

        LOG.debug(f"{self.vid}'s desired speed is {self.desired_speed}")

        new_speed = -1
        # do we need to adjust our speed?
        diff_to_desired = self.desired_speed - self.speed
        if diff_to_desired > 0:
            # we need to accelerate
            new_speed = min(self.speed + min(diff_to_desired, self._simulator.acceleration2speed(self.max_acceleration, self._simulator.step_length)), self.max_speed)
            LOG.debug(f"{self.vid} wants to accelerate to {new_speed}")
        elif diff_to_desired < 0:
            # we need to decelerate
            new_speed = max(self.speed + max(diff_to_desired, -self._simulator.acceleration2speed(self.max_deceleration, self._simulator.step_length)), 0)
            LOG.debug(f"{self.vid} wants to decelerate to {new_speed}")
        else:
            new_speed = self.speed
            LOG.debug(f"{self.vid} keeps the speed of {new_speed}")

        # vsafe
        if speed_predecessor >= 0 and predecessor_rear_position >= 0:
            # we have a predecessor
            gap_to_predecessor = predecessor_rear_position - self.position
            LOG.debug(f"{self.vid}'s front gap {gap_to_predecessor}")
            if gap_to_predecessor < 0:
                LOG.warn(f"{self.vid}'s front gap is negative")
            LOG.debug(f"{self.vid}'s predecessor speed {speed_predecessor}")
            LOG.debug(f"{self.vid}'s desired gap {self.desired_gap}")
            safe_speed = self._safe_speed(speed_predecessor, gap_to_predecessor, self.desired_gap, self.min_gap)
            LOG.debug(f"{self.vid}'s safe speed {safe_speed}")

            if safe_speed < new_speed:
                LOG.info(f"{self.vid} is blocked by a slow vehicle!")
                self._blocked_front = True

                new_speed = max(safe_speed, self.speed - self._simulator.acceleration2speed(self.max_deceleration, self._simulator.step_length))  # we cannot brake stronger than we actually can
                LOG.debug(f"{self.vid}'s new speed after safe speed is {new_speed}")
            else:
                self._blocked_front = False
        else:
            # we have no predecessor
            self._blocked_front = False

        # TODO dawdling? we do not support dawdling at the moment (sigma == 0.0)
        # new_speed -= random() * new_speed

        if (new_speed < 0):
            new_speed = 0

        LOG.debug(f"{self.vid}'s new speed is {new_speed}")

        return new_speed

    def action(self, step: int):
        """Trigger actions of a vehicle"""

        if step < self.depart_time:
            # we did not start yet
            return
        else:
            # we started (right now)
            self._start()

            LOG.info(self.info())

            # log periodic statistics
            self._statistics()

            # What has to be triggered periodically?
            self._action(step)

    def _action(self, step: int):
        """Trigger concrete actions of a Vehicle"""

        pass  # this vehicle has no application running

    def _start(self):
        """Start this Vehicle"""

        if self._started:
            return

        self._started = True

    def info(self):
        """Return info of a Vehicle"""

        e_remaining_travel_time = round((self.arrival_position - self.position) / self.desired_speed)
        return f"{self.vid} at {self.position}-{self.rear_position}, {self.lane} with {self.speed}, takes {e_remaining_travel_time}"

    def _statistics(self):
        """Write continuous statistics"""

        if self._simulator._record_vehicle_traces:
            # mobility/trip statistics
            with open(self._simulator._result_base_filename + '_vehicle_traces.csv', 'a') as f:
                f.write(f"{self._simulator.step},{self.vid},{self.position},{self.lane},{self.speed},{self.travel_time},{self.travel_distance}\n")

        self._calculate_emissions()

        # TODO current gap to front

    def _calculate_emissions(self):
        # Computes the emitted pollutant amount using the given speed and acceleration
        #
        # As the functions are defining emissions in g/hour, the function's result is normed
        # by 3.6 (seconds in an hour/1000) yielding mg/s. For fuel ml/s is returned.
        # Negative acceleration results directly in zero emission.
        #
        # The amount emitted by the given emission class when moving with the given velocity and acceleration [mg/s or ml/s]
        #
        # SUMO: The current default model is HBEFA3/PC_G_EU4 (a gasoline powered Euro norm 4 passenger car modeled using the HBEFA3 based model).
        emission_factors = {
            'co': [593.2, 19.32, 0.0, -73.25, 2.086, 0.0],
            'co2': [9449, 938.4, 0.0, -467.1, 28.26, 0.0],
            'hc': [2.923, 0.1113, 0.0, -0.3476, 0.01032, 0.0],
            'pmx': [0.2375, 0.0245, 0.0, -0.03251, 0.001325, 0.0],
            'nox': [4.336, 0.4428, 0.0, -0.3204, 0.01371, 0.0],
            'fuel': [3014, 299.3, 0.0, -149, 9.014, 0.0]
        }
        diesel = False  # TODO make paramemter of vehicle type

        if self._simulator._record_emission_traces:
            with open(self._simulator._result_base_filename + '_emission_traces.csv', 'a') as f:
                f.write(f"{self._simulator.step},{self.vid}")

        for variable in self._emissions.keys():
            scale = 3.6
            if variable == 'fuel':
                if diesel:
                    scale *= 836.0
                else:
                    scale *= 742.0
            value = self._calculate_emission(self.acceleration, self.speed, emission_factors[variable], scale) * self._simulator.step_length
            self._emissions[variable] += value

            if self._simulator._record_emission_traces:
                with open(self._simulator._result_base_filename + '_emission_traces.csv', 'a') as f:
                    f.write(f",{round(value, 2)}")

        if self._simulator._record_emission_traces:
            with open(self._simulator._result_base_filename + '_emission_traces.csv', 'a') as f:
                f.write("\n")

    def _calculate_emission(self, a: float, v: float, f: list, scale: float) -> float:
        if a < 0:
            return 0
        return max((f[0] + f[1] * a * v + f[2] * a * a * v + f[3] * v + f[4] * v * v + f[5] * v * v * v) / scale, 0.0)

    def finish(self):
        """Clean up the instance of the vehicle"""

        if (self.position < self.arrival_position):
            LOG.warn(f"{self.vid}'s finish method was called even though it did not arrive yet!")
            return

        e_travel_time = (self.arrival_position - self.depart_position) / self.desired_speed
        time_loss = self.travel_time - round(e_travel_time)
        travel_time_ratio = round(self.travel_time / e_travel_time, 2)
        average_driving_speed = round(self.travel_distance / self.travel_time, 1)
        average_deviation_desired_speed = round(self._desired_speed - average_driving_speed, 1)

        LOG.info(f"{self.vid} arrived at {self.position}, {self.lane} with {self.speed}, took {self.travel_time}, {self.travel_distance}, {time_loss} {travel_time_ratio * 100}")

        if self._simulator._record_vehicle_trips:
            with open(self._simulator._result_base_filename + '_vehicle_trips.csv', 'a') as f:
                f.write(f"{self.vid},{self.depart_time},{self.depart_lane},{self.depart_position},{self.depart_speed},{self._simulator.step},{self.lane},{self.position},{self.speed},{self.travel_time},{self.travel_distance},{time_loss},{self.desired_speed},{e_travel_time},{travel_time_ratio},{average_driving_speed},{average_deviation_desired_speed}\n")

        if self._simulator._record_vehicle_emissions:
            with open(self._simulator._result_base_filename + '_vehicle_emissions.csv', 'a') as f:
                # TODO log estimated emissions?
                f.write(f"{self.vid},{self._emissions['co']},{self._emissions['co2']},{self._emissions['hc']},{self._emissions['pmx']},{self._emissions['nox']},{self._emissions['fuel']}\n")

    def __str__(self) -> str:
        """Return a nice string representation of a vehicle instance"""

        return str(self.__dict__)

    def _transmit(self, destination_vid: int, message: Message) -> bool:
        """Transmit a message of type Message"""

        if isinstance(message, Message):
            if destination_vid == -1:
                # TODO do we really want access the private field of the vehicles here (i.e., within this class)?
                for vehicle in self._simulator._vehicles.values():
                    vehicle.receive(message)
            else:
                # TODO do we really want access the private field of the vehicles here (i.e., within this class)?
                self._simulator._vehicles[destination_vid].receive(message)

            return True  # this should always be true, at least currently
        raise RuntimeError("Message is not an instance of type Message")

    def receive(self, message) -> bool:
        """Receive a message of arbitrary type"""

        if self._simulator.step < self.depart_time:
            # we cannot receive anything since we did not start yet
            return False
        if isinstance(message, Message):
            if message.destination == self.vid or message.destination == -1:
                self._handle_message(message)
            # we cannot receive this message since it was not for us
            return False
        raise RuntimeError("Message is not an instance of type Message")

    def _handle_message(self, message: Message):
        """Handle a message of arbitrary type Message"""

        func = self.__class__.__dict__.get('_receive_' + message.__class__.__name__,
                                           lambda v, m: print("cannot handle message", m))
        return func(self, message)

    def _receive_Message(self, message: Message):
        """Handle a message of concrete type Message"""

        LOG.warn(f"{self.vid} received non-sense message {message}")
