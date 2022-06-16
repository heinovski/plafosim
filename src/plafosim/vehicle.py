#
# Copyright (c) 2020-2022 Julian Heinovski <heinovski@ccs-labs.org>
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
from typing import TYPE_CHECKING

from .message import Message
from .mobility import CF_Model
from .statistics import (
    record_emission_trace_prefix,
    record_emission_trace_suffix,
    record_emission_trace_value,
    record_vehicle_emission,
    record_vehicle_trace,
    record_vehicle_trip,
)
from .util import speed2distance
from .vehicle_type import VehicleType

if TYPE_CHECKING:
    from numpy.typing import ArrayLike

    from .simulator import Simulator  # noqa 401
else:
    ArrayLike = float

LOG = logging.getLogger(__name__)

SPEED_NO_PREDECESSOR = 1e15
REARPOSITION_NO_PREDECESSOR = 1e15


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
            depart_delay: int,
            communication_range: int,
            pre_filled: bool = False,
    ):
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
        depart_delay : int
            The time the vehicle had to wait before starting its trip
        communication_range : int
            The maximum communication range of the vehicle
        pre_filled : bool
            Whether this vehicle was pre-filled
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
        self._depart_delay = depart_delay  # the depart delay of the vehicle
        self._pre_filled = pre_filled  # whether this vehicle was pre-filled
        # vehicle details
        self._position = self._depart_position  # the current position of the vehicle
        self._lane = self._depart_lane  # the current lane of the vehicle
        self._speed = self._depart_speed  # the current speed of the vehicle
        self._blocked_front = False  # whether the vehicle is blocked by a slower vehicle in front
        self._acceleration = 0  # the current acceleration of the vehicle, used (only) for the emission model
        self._cf_model = CF_Model.Human  # the current car following model
        self._cf_target_speed = desired_speed  # the target speed for the car following

        # communication properties
        # TODO move to platooning vehicle
        self._communication_range = communication_range  # the maximum communication range between two vehicles

        # statistics
        self._time_loss = 0
        self._emissions = {
            'CO': 0,  # the total CO (Kohlenmonoxid) emission in mg
            'CO2': 0,  # the total CO2 (Kohlendioxid) emission in mg
            'HC': 0,  # the total HC (Kohlenwasserstoffe) emission in mg
            'NOx': 0,  # the total NOx (Stickoxide) emission in mg
            'PMx': 0,  # the total PM (Partikel) emission in mg
            'fuel': 0,  # the total mKr (Kraftstoffverbrauch) emission in mg
        }

        # gui properties
        self._color = (
            self._simulator._rng.randrange(0, 255, 1),
            self._simulator._rng.randrange(0, 255, 1),
            self._simulator._rng.randrange(0, 255, 1),
        )

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
    def headway_time(self) -> float:
        """
        Returns the human headway time of the vehicle.

        This is based on the vehicle type.
        """

        return self._vehicle_type._headway_time

    @property
    def desired_headway_time(self) -> float:
        """Returns the desired headway time of the vehicle."""

        return self._vehicle_type._headway_time

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
        assert position >= 0
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

    @property
    def color(self) -> tuple:
        """Return the current color of the vehicle."""
        return self._color

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
        if LOG.getEffectiveLevel() <= logging.TRACE:
            LOG.trace(self.info())

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

        # calculate time loss
        # SUMO: "The time lost due to driving below the ideal speed."
        # can also use higher layer desired speed
        # FIXME: should this the cf target speed?
        if self._speed < self.desired_speed:
            self._time_loss += self._simulator.step_length

        if self._simulator._record_vehicle_traces:
            # mobility/trip statistics
            record_vehicle_trace(basename=self._simulator._result_base_filename, step=self._simulator.step, vehicle=self)

        self._calculate_emissions()

        # TODO current gap to front

    def _calculate_emissions(self):
        """
        Calculates the emitted pollutant amount using the given speed and acceleration based on the HBEFA3 model.

        As the functions are defining emissions in g/hour, the function's result is normed
        by 3.6 (seconds in an hour/1000) yielding mg/s. For fuel ml/s is returned.
        Negative acceleration results directly in zero emission.

        The amount emitted by the given emission class when moving with the given velocity and acceleration [mg/s or ml/s]
        """

        if not self._simulator._record_prefilled and self._depart_time == -1:
            # we do not record statistics for pre-filled vehicles
            return

        if self._simulator._record_emission_traces:
            record_emission_trace_prefix(
                basename=self._simulator._result_base_filename,
                step=self._simulator.step,
                vid=self._vid,
            )

        ec = self._vehicle_type.emission_class
        for variable in self._emissions.keys():
            scale = 3.6
            if variable == 'fuel':
                if ec.is_diesel:
                    scale *= 836.0
                else:
                    scale *= 742.0
            value = (self._calculate_emission(
                self._acceleration,
                self._speed,
                ec.emission_factors[variable],
                scale
            ) * self._simulator.step_length)
            self._emissions[variable] += value

            if self._simulator._record_emission_traces:
                record_emission_trace_value(basename=self._simulator._result_base_filename, value=value)

        if self._simulator._record_emission_traces:
            record_emission_trace_suffix(basename=self._simulator._result_base_filename)

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
        assert self.travel_time != 0
        assert expected_travel_time != 0
        travel_time_ratio = self.travel_time / expected_travel_time
        # NOTE: this also contains teleports
        average_driving_speed = self.travel_distance / self.travel_time
        average_deviation_desired_speed = average_driving_speed - self._desired_speed  # use explicit individual desired speed

        LOG.info(f"{self._vid} arrived at {self._position}m,{self._lane} with {self._speed}m/s, took {self.travel_time}s, {self.travel_distance}m, loss: {self._time_loss}s, {travel_time_ratio * 100}% of expected duration")

        # statistic recording

        # TODO use pre_filled flag
        if not self._simulator._record_prefilled and self._depart_time == -1:
            # we do not record statistics for pre-filled vehicles
            LOG.debug(f"Not recording statistics for pre-filled vehicle {self._vid}")
            return

        # by this check, we should also already avoid logging if the minimum trip length has not been fulfilled
        # HACK: adding length here to cope for departPos="base"
        # TODO we might need to travel 'length' more than arrival position
        if self.travel_distance < (self._simulator._minimum_trip_length - self.length):
            # we are only interested in vehicles that did complete the minimum trip length
            # this should only be the case for pre-filled vehicles or if started as platoon
            # TODO use pre_filled flag
            assert (
                (self._simulator._record_prefilled and self._depart_time == -1)
                or self._simulator._start_as_platoon
            )

        assert travel_time_ratio >= 0
        assert average_driving_speed >= 0

        if self._simulator._record_end_trace:
            # call trace recording once again
            self._statistics()

        if self._simulator._record_vehicle_trips:
            record_vehicle_trip(
                basename=self._simulator._result_base_filename,
                vehicle=self,
                time_loss=self._time_loss,
                depart_delay=self._depart_delay,
                expected_travel_time=expected_travel_time,
                travel_time_ratio=travel_time_ratio,
                average_driving_speed=average_driving_speed,
                average_deviation_desired_speed=average_deviation_desired_speed,
            )

        if self._simulator._record_vehicle_emissions:
            record_vehicle_emission(basename=self._simulator._result_base_filename, vehicle=self)

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
