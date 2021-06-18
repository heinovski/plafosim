#
# Copyright (c) 2020-2021 Julian Heinovski <heinovski@ccs-labs.org>
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
import math
from typing import TYPE_CHECKING

from .cf_model import CF_Model
from .message import Message
from .util import acceleration2speed, distance2speed, speed2distance
from .vehicle_type import VehicleType

if TYPE_CHECKING:
    from .simulator import Simulator  # noqa 401

LOG = logging.getLogger(__name__)


class Vehicle:
    """
    A collection of state information for a vehicle in the simulation.

    A vehicle can really be anything that can move and can be defined by a vehicle type.
    It does not necessarily be driven by a computer (i.e., autonomous).
    However, by default it does have V2X functionality.
    """

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
            communication_range: int):
        """
        Initializes a vehicle instance.

        Parameters
        ----------
        simulator : Simulator
            The global simulator object
        vid : int
            The id of the vehicle
        vehicle_type : VehicleType
            The vehicle type of the vehicle
        depart_position : int
            The depart position of the vehicle
        arrival_position : int
            The arrival position of the vehicle
        desired_speed : float
            The desired driving speed of the vehicle
        depart_lane : int
            The depart lane of the vehicle
        depart_speed : float
            The depart speed of the vehicle
        depart_time : int
            The depart time of the vehicle
        communication_range : int
            The maximum communication range of the vehicle
        """

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
        self._cf_model = CF_Model.CC  # the current car following model
        self._cc_target_speed = desired_speed  # the target speed for CC

        # communication properties
        # TODO move to platooning vehicle
        self._communication_range = communication_range  # the maximum communication range between two vehicles

        # statistics
        self._emissions = {
            'CO': 0,  # the total CO (Kohlenmonoxid) emission in mg
            'CO2': 0,  # the total CO2 (Kohlendioxid) emission in mg
            'HC': 0,  # the total HC (Kohlenwasserstoffe) emission in mg
            'NOx': 0,  # the total NOx (Stickoxide) emission in mg
            'PMx': 0,  # the total PM (Partikel) emission in mg
            'fuel': 0,  # the total mKr (Kraftstoffverbrauch) emission in mg
        }

    @property
    def vid(self) -> int:
        """Return the id of the vehicle."""

        return self._vid

    @property
    def vehicle_type(self) -> VehicleType:
        """Returns the VehicleType of the vehicle."""

        return self._vehicle_type

    @property
    def length(self) -> int:
        """
        Returns the length of the vehicle.

        This is based on the vehicle type.
        """

        return self._vehicle_type._length

    @property
    def max_speed(self) -> float:
        """
        Returns the maximum speed of the vehicle.

        This is based on the vehicle type.
        """

        return self._vehicle_type._max_speed

    @property
    def max_acceleration(self) -> float:
        """
        Returns the maximum acceleration of the vehicle.

        This is based on the vehicle type.
        """

        return self._vehicle_type._max_acceleration

    @property
    def max_deceleration(self) -> float:
        """
        Returns the maximum deceleration of the vehicle.

        This is based on the vehicle type.
        """

        return self._vehicle_type._max_deceleration

    @property
    def min_gap(self) -> float:
        """
        Returns the minimum safety gap to the vehicle in front of the vehicle.

        This is based on the vehicle type.
        """

        return self._vehicle_type._min_gap

    @property
    def cc_headway_time(self) -> float:
        """
        Returns the CC headway time of the vehicle.

        This is based on the vehicle type.
        """

        return self._vehicle_type._cc_headway_time

    @property
    def desired_headway_time(self) -> float:
        """Returns the desired headway time of the vehicle."""

        return self._vehicle_type._cc_headway_time

    @property
    def depart_position(self) -> int:
        """Returns the depart position of the vehicle."""

        return self._depart_position

    @property
    def arrival_position(self) -> int:
        """Returns the arrival position of the vehicle."""

        return self._arrival_position

    @property
    def desired_speed(self) -> float:
        """Returns the desired driving speed of the vehicle."""

        return self._desired_speed

    @property
    def desired_gap(self) -> float:
        """
        Returns the desired gap to the vehicle in front of the vehicle.

        This is based on the desired headway time and the current driving speed.
        """

        # use potential other desired headway time
        # TODO should this be target speed?
        return speed2distance(self.desired_headway_time * self._speed, self._simulator._step_length)

    @property
    def depart_lane(self) -> int:
        """Returns the depart lane of the vehicle."""

        return self._depart_lane

    @property
    def depart_speed(self) -> float:
        """Returns the depart speed of the vehicle."""

        return self._depart_speed

    @property
    def depart_time(self) -> int:
        """Returns the depart time of the vehicle."""

        return self._depart_time

    @property
    def position(self) -> float:
        """Returns the current position of the vehicle."""

        return self._position

    @property
    def rear_position(self) -> int:
        """Returns the current rear position of the vehicle."""

        position = self._position - self._vehicle_type._length
        assert(position >= 0)
        return position

    @property
    def lane(self) -> int:
        """Returns the current lane of the vehicle."""

        return self._lane

    @property
    def speed(self) -> float:
        """Returns the current driving speed of the vehicle."""

        return self._speed

    @property
    def acceleration(self) -> int:
        """Returns the current acceleration of the vehicle."""

        return self._acceleration

    @property
    def cf_model(self) -> CF_Model:
        """Returns the currently activated car following model of the vehicle."""

        return self._cf_model

    @property
    def travel_distance(self) -> float:
        """Returns the current traveled distance of the vehicle."""

        return self._position - self._depart_position

    @property
    def travel_time(self) -> float:
        """Returns the current traveled time of the vehicle."""

        return self._simulator.step - self._depart_time

    @property
    def blocked_front(self) -> bool:
        """Returns whether the vehicle is currently blocked by a slow vehicle in the front."""

        return self._blocked_front

    def _safe_speed(
        self,
        speed_predecessor: float,
        gap_to_predecessor: float,
        desired_gap: float = 0,
        min_gap: float = 0
    ) -> float:
        """
        Returns the speed which is still safe without a collision.

        This is a simple and dumb calculation for the safe speed of a vehicle.
        The calculation is is based on Krauss' single lane traffic:
        v_safe(t) = v_lead(t) + (g(t)-g_des(t)) / (tau_b + tau)

        Parameters
        ----------
        speed_predecessor : float
            The driving speed of the vehicle in the front
        gap_to_predecessor : float
            The gap to the vehicle in the front
        desired_gap : float, optional
            The desired gap
        min_gap : float, optional
            The minimum safety gap
        """

        gap_to_close = gap_to_predecessor - max(desired_gap, min_gap)  # use to close the gap
        return speed_predecessor + distance2speed(gap_to_close, self.desired_headway_time)

    def new_speed(self, speed_predecessor: float, predecessor_rear_position: float, predecessor_id: int) -> float:
        """
        Calculates the new speed for a vehicle using the kraus model.

        This is a simple and dumb calculation for the safe speed of a vehicle.
        The calculation is is based on Krauss' single lane traffic:
        adjust speed
        v_max, desired speed
        epsilon, dawdling of drives
        g_des = tau*v_lead
        tau, reaction time of drivers
        tau_b = v/b
        v_des(t) = min[v_max, v(t)+a(v)*step_size, v_safe(t)]
        v(t + step_size) = max[0, v_des(t) - epsilon]

        Parameters
        ----------
        speed_predecessor : float
            The driving speed of the vehicle in the front
        predecessor_rear_position : float
            The rear position of the vehicle in the front
        predecessor_id : int
            The id of the vehicle in the front. This should only be used for debugging.
        """

        LOG.trace(f"{self._vid}'s target speed is {self._cc_target_speed}m/s")

        new_speed = -1
        # do we need to adjust our speed?
        diff_to_target = self._cc_target_speed - self._speed
        if diff_to_target > 0:
            # we need to accelerate
            new_speed = min(
                self._speed +
                min(diff_to_target, acceleration2speed(self.max_acceleration, self._simulator.step_length)),
                self.max_speed
            )
            LOG.trace(f"{self._vid} wants to accelerate to {new_speed}m/s")
        elif diff_to_target < 0:
            # we need to decelerate
            new_speed = max(
                self._speed +
                max(diff_to_target, -acceleration2speed(self.max_deceleration, self._simulator.step_length)),
                0
            )
            LOG.trace(f"{self._vid} wants to decelerate to {new_speed}m/s")
        else:
            new_speed = self._speed
            LOG.trace(f"{self._vid} wants to keep the speed of {new_speed}m/s")

        # vsafe
        if speed_predecessor >= 0 and predecessor_rear_position >= 0:
            # we have a predecessor
            gap_to_predecessor = predecessor_rear_position - self._position
            LOG.trace(f"{self._vid}'s front gap {gap_to_predecessor}m")
            if gap_to_predecessor < 0:
                LOG.warning(f"{self._vid}'s front gap is negative ({gap_to_predecessor}m)")
            LOG.trace(f"{self._vid}'s predecessor speed {speed_predecessor}m/s")
            LOG.trace(f"{self._vid}'s desired gap {self.desired_gap}m")
            safe_speed = self._safe_speed(speed_predecessor, gap_to_predecessor, self.desired_gap, self.min_gap)
            LOG.trace(f"{self._vid}'s safe speed {safe_speed}m/s")

            if safe_speed < new_speed:
                LOG.debug(f"{self._vid} is blocked by a slow vehicle!")
                self._blocked_front = True

                # we cannot brake stronger than we actually can
                new_speed = max(
                    safe_speed,
                    self._speed - acceleration2speed(self.max_deceleration, self._simulator.step_length)
                )
                LOG.trace(f"{self._vid}'s new speed after safe speed is {new_speed}m/s")
                if safe_speed < new_speed:
                    LOG.warn(f"{self._vid}'s is performing an emergency braking! Its new speed ({new_speed}m/s) is still faster than its safe speed ({safe_speed}m/s)! This will probably lead to a crash!")
            else:
                self._blocked_front = False
        else:
            # we have no predecessor
            self._blocked_front = False

        # TODO dawdling? we do not support dawdling at the moment (sigma == 0.0)
        # new_speed -= random() * new_speed

        # make sure we do not drive backwards
        if (new_speed < 0):
            new_speed = 0

        # avoid issues due to floating point precision
        if math.isclose(new_speed, self._speed):
            new_speed = self._speed

        LOG.debug(f"{self._vid}'s new speed is {new_speed}m/s")

        return new_speed

    def action(self, step: int):
        """
        Triggers actions of a vehicle.

        Parameters
        ----------
        step : int
            The current simulation step
        """

        # we started (right now)
        self._start()

        # log status information
        LOG.info(self.info())

        # record periodic statistics
        self._statistics()

        # What has to be triggered periodically?
        if self._simulator._actions:
            self._action(step)

    def _action(self, step: int):
        """
        Triggers specific actions of a vehicle.

        Parameters
        ----------
        step : int
            The current simulation step
        """

        pass  # this vehicle has no application running

    # TODO: obsolete?
    def _start(self):
        """Starts this Vehicle"""

        if self._started:
            return

        self._started = True

    def info(self) -> str:
        """Returns information about the vehicle."""

        estimated_remaining_travel_time = (
            (self._arrival_position - self._position) / self._speed
            if self._speed > 0
            else self.desired_speed  # use potential other desired driving speed
        )
        return f"{self._vid} at {self._position}-{self.rear_position}, {self._lane} with {self._speed}, takes {estimated_remaining_travel_time}s to reach {self._arrival_position}"

    def _statistics(self):
        """Writes continuous statistics for the vehicle."""

        if not self._simulator._record_prefilled and self._depart_time == -1:
            # we do not record statistics for pre-filled vehicles
            return

        if self._simulator._record_vehicle_traces:
            # mobility/trip statistics
            with open(f'{self._simulator._result_base_filename}_vehicle_traces.csv', 'a') as f:
                f.write(
                    f"{self._simulator.step},"
                    f"{self._vid},"
                    f"{self._position},"
                    f"{self._lane},"
                    f"{self._speed},"
                    f"{self.travel_time},"
                    f"{self.travel_distance},"
                    f"{self.desired_speed},"  # use potential other desired driving speed
                    f"{self._cc_target_speed}"
                    "\n"
                )

        self._calculate_emissions()

        # TODO current gap to front

    def _calculate_emissions(self):
        """
        Calculates the emitted pollutant amount using the given speed and acceleration.

        As the functions are defining emissions in g/hour, the function's result is normed
        by 3.6 (seconds in an hour/1000) yielding mg/s. For fuel ml/s is returned.
        Negative acceleration results directly in zero emission.

        The amount emitted by the given emission class when moving with the given velocity and acceleration [mg/s or ml/s]

        SUMO: The current default model is HBEFA3/PC_G_EU4 (a gasoline powered Euro norm 4 passenger car modeled using the HBEFA3 based model).
        """

        emission_factors = {
            'CO': [593.2, 19.32, 0.0, -73.25, 2.086, 0.0],
            'CO2': [9449, 938.4, 0.0, -467.1, 28.26, 0.0],
            'HC': [2.923, 0.1113, 0.0, -0.3476, 0.01032, 0.0],
            'NOx': [4.336, 0.4428, 0.0, -0.3204, 0.01371, 0.0],
            'PMx': [0.2375, 0.0245, 0.0, -0.03251, 0.001325, 0.0],
            'fuel': [3014, 299.3, 0.0, -149, 9.014, 0.0],
        }
        diesel = False  # TODO make parameter of vehicle type

        if not self._simulator._record_prefilled and self._depart_time == -1:
            # we do not record statistics for pre-filled vehicles
            return

        if self._simulator._record_emission_traces:
            with open(f'{self._simulator._result_base_filename}_emission_traces.csv', 'a') as f:
                f.write(
                    f"{self._simulator.step},"
                    f"{self._vid}"
                )

        for variable in self._emissions.keys():
            scale = 3.6
            if variable == 'fuel':
                if diesel:
                    scale *= 836.0
                else:
                    scale *= 742.0
            value = (self._calculate_emission(
                self._acceleration,
                self._speed,
                emission_factors[variable],
                scale
            ) * self._simulator.step_length)
            self._emissions[variable] += value

            if self._simulator._record_emission_traces:
                with open(f'{self._simulator._result_base_filename}_emission_traces.csv', 'a') as f:
                    f.write(
                        f",{value}"
                    )

        if self._simulator._record_emission_traces:
            with open(f'{self._simulator._result_base_filename}_emission_traces.csv', 'a') as f:
                f.write("\n")

    def _calculate_emission(self, a: float, v: float, f: list, scale: float) -> float:
        """
        Calculates the actual emission of the vehicle.

        Parameters
        ----------
        a : float
            The current acceleration
        v : float
            The current speed
        f : list
            The emission factors to use for current emission variable to be calculated
        scale : float
            The scale to normalize the calculated value
        """

        if a < 0:
            return 0
        return max((f[0] + f[1] * a * v + f[2] * a * a * v + f[3] * v + f[4] * v * v + f[5] * v * v * v) / scale, 0.0)

    def finish(self):
        """
        Cleans up the instance of the vehicle.

        This includes mostly statistic recording.
        """

        if (self._position < self._arrival_position):
            LOG.warning(f"{self._vid}'s finish method was called even though vehicle did not arrive yet!")
            return

        expected_travel_time = (self._arrival_position - self._depart_position) / self._desired_speed  # use explicit individual desired speed
        assert(self.travel_time != 0)
        time_loss = self.travel_time - expected_travel_time
        assert(expected_travel_time != 0)
        travel_time_ratio = self.travel_time / expected_travel_time
        # NOTE: this also contains teleports
        average_driving_speed = self.travel_distance / self.travel_time
        average_deviation_desired_speed = average_driving_speed - self._desired_speed  # use explicit individual desired speed

        LOG.info(f"{self._vid} arrived at {self._position}m,{self._lane} with {self._speed}m/s, took {self.travel_time}s, {self.travel_distance}m, loss: {time_loss}s, {travel_time_ratio * 100}% of expected duration")

        # statistic recording

        if not self._simulator._record_prefilled and self._depart_time == -1:
            # we do not record statistics for pre-filled vehicles
            return

        # by this check, we should also already avoid logging if the minimum trip length has not been fulfilled
        # HACK: adding length here to cope for departPos="base"
        # TODO we might need to travel 'length' more than arrival position
        if self.travel_distance < (self._simulator._minimum_trip_length - self.length):
            # we are only interested in vehicles that did complete the minimum trip length
            # this should only be the case for pre-filled vehicles
            assert(self._depart_time == -1)
            return
        # we could still have pre-filled vehicles that drove at least for the minimum trip length

        assert(travel_time_ratio >= 0)
        assert(average_driving_speed >= 0)

        if self._simulator._record_end_trace:
            # call trace recording once again
            self._statistics()

        if self._simulator._record_vehicle_trips:
            with open(f'{self._simulator._result_base_filename}_vehicle_trips.csv', 'a') as f:
                f.write(
                    f"{self._vid},"
                    f"{self._vehicle_type.name},"
                    "HBEFA3/PC_G_EU4,"  # TODO make parameter
                    f"{self.__class__.__name__},"
                    f"{self._depart_time},"
                    f"{self._depart_lane},"
                    f"{self._depart_position},"
                    f"{self._depart_speed},"
                    f"{self._simulator.step},"
                    f"{self._lane},"
                    f"{self._position},"
                    f"{self._speed},"
                    f"{self.travel_time},"
                    f"{self.travel_distance},"
                    f"{time_loss},"
                    f"{self._desired_speed},"  # use explicit individual desired speed
                    f"{expected_travel_time},"
                    f"{travel_time_ratio},"
                    f"{average_driving_speed},"
                    f"{average_deviation_desired_speed}"
                    "\n"
                )

        if self._simulator._record_vehicle_emissions:
            with open(f'{self._simulator._result_base_filename}_vehicle_emissions.csv', 'a') as f:
                # TODO log estimated emissions?
                f.write(
                    f"{self._vid},"
                    f"{self._emissions['CO']},"
                    f"{self._emissions['CO2']},"
                    f"{self._emissions['HC']},"
                    f"{self._emissions['NOx']},"
                    f"{self._emissions['PMx']},"
                    f"{self._emissions['fuel']}"
                    "\n"
                )

    def __str__(self) -> str:
        """Returns the str representation of the vehicle."""

        self_dict = self.__dict__.copy()
        self_dict.update({'_vehicle_type': str(self._vehicle_type)})  # use str representation of vehicle type
        return str(self_dict)

    def _transmit(self, destination_vid: int, message: Message) -> bool:
        """
        Transmits a message of type Message.

        Messages are in general not used at the moment.

        Parameters
        ----------
        destination_vid : int
            The id of the destination vehicle
        message : Message
            The message to transmit

        THIS IS DEPRECATED AT THE MOMENT!!!
        """

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
        """
        Receives a message of arbitrary type.

        Messages are in general not used at the moment.

        Parameters
        ----------
        message :
            The message to be received

        Raises
        ------
        RuntimeError
            If the message to be received is not an instance of the Message type.

        THIS IS DEPRECATED AT THE MOMENT!!!
        """

        if self._simulator.step < self._depart_time:
            # we cannot receive anything since we did not start yet
            return False
        if isinstance(message, Message):
            if message.destination == self._vid or message.destination == -1:
                self._handle_message(message)
            # we cannot receive this message since it was not for us
            return False
        raise RuntimeError("Message is not an instance of type Message")

    def _handle_message(self, message: Message):
        """
        Handles a message of arbitrary type Message.

        Messages are in general not used at the moment.

        Parameters
        ----------
        message : Message
            The message to be handled

        THIS IS DEPRECATED AT THE MOMENT!!!
        """

        func = self.__class__.__dict__.get('_receive_' + message.__class__.__name__,
                                           lambda v, m: print("cannot handle message", m))
        return func(self, message)

    def _receive_Message(self, message: Message):
        """
        Handles a message of the specific type Message.

        Messages are in general not used at the moment.

        Parameters
        ----------
        message : Message
            The message to be received

        THIS IS DEPRECATED AT THE MOMENT!!!
        """

        LOG.warning(f"{self._vid} received non-sense message {message}")
