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
from enum import Enum
from .message import Message, PlatoonAdvertisement
# from .simulator import Simulator # TODO fix circular import


class VehicleType:
    """A collection of parameters for a concrete vehicle type"""

    def __init__(self, name: str, length: int, max_speed: float, max_acceleration: float, max_deceleration: float, min_gap: float):
        self._name = name  # the name of a vehicle type
        self._length = length  # the length of a vehicle type
        self._max_speed = max_speed  # the maximum speed of a vehicle type
        self._max_acceleration = max_acceleration  # the maximum acceleration of the vehicle type
        self._max_deceleration = max_deceleration  # the maximum deceleration of the vehicle type
        self._min_gap = min_gap  # the minimum gap to the vehicle in front

    @property
    def name(self) -> str:
        return self._name

    @property
    def length(self) -> int:
        return self._length

    @property
    def max_speed(self) -> float:
        return self._max_speed

    @property
    def max_acceleration(self) -> float:
        return self._max_acceleration

    @property
    def max_deceleration(self) -> float:
        return self._max_deceleration

    @property
    def min_gap(self) -> float:
        return self._min_gap


class Vehicle:
    """A collection of state information for a vehicle in the simulation

    A vehicle can really be anything that can move and can be defined by a vehicle type.
    It does not necessarily be driven by a computer (i.e., autonomous).
    However, by default it does have V2X functionality."""

    def __init__(
            self,
            simulator,  # TODO add type hint
            vid: int,
            vehicle_type: VehicleType,
            depart_position: int,
            arrival_position: int,
            desired_speed: float,
            depart_lane: int,
            depart_speed: float,
            depart_time: int):
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
        # statistics
        self._co = 0  # the total co emission in g
        self._co2 = 0  # the total co2 emission in g
        self._hc = 0  # the total hc emission in g
        self._pmx = 0  # the total pmx emission in g
        self._npx = 0  # the total npx emission in g
        self._fuel = 0  # the total fuel consumption in ml

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
    def depart_position(self) -> int:
        return self._depart_position

    @property
    def arrival_position(self) -> int:
        return self._arrival_position

    @property
    def desired_speed(self) -> float:
        return self._desired_speed

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
        return max(self.position - self.length, 0)

    @property
    def lane(self) -> int:
        return self._lane

    @property
    def speed(self) -> float:
        return self._speed

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
        return speed_diff_to_use + self._simulator.distance2speed(position_diff_to_use, self._simulator._step_length)

    # krauss - single lane traffic
    # adjust speed
    # v_max, desired speed
    # epsilon, dawdling of drives
    # g_des = tau*v_lead
    # tau, reaction time of drivers
    # tau_b = v/b
    # v_des(t) = min[v_max, v(t)+a(v)*step_size, v_safe(t)]
    # v(t + step_size) = max[0, v_des(t) - epsilon]
    def new_speed(self, speed_predecessor: float, predecessor_rear_position: float, desired_gap: float = 0) -> float:
        """Calculate the new speed for a vehicle using the kraus model"""

        new_speed = -1
        # do we need to adjust our speed?
        diff_to_desired = self.desired_speed - self.speed
        if diff_to_desired > 0:
            # we need to accelerate
            new_speed = min(self.speed + min(diff_to_desired, self._simulator.acceleration2speed(self.max_acceleration, self._simulator._step_length)), self.max_speed)
            if self._simulator._debug:
                print("%d we need to accelerate %f" % (self.vid, new_speed), flush=True)
        elif diff_to_desired < 0:
            # we need to decelerate
            new_speed = max(self.speed + max(diff_to_desired, -self._simulator.acceleration2speed(self.max_deceleration, self._simulator._step_length)), 0)
            if self._simulator._debug:
                print("%d we need to decelerate %f" % (self.vid, new_speed), flush=True)
        else:
            new_speed = self.speed
            if self._simulator._debug:
                print("%d we keep the speed %f" % (self.vid, new_speed), flush=True)

        # vsafe
        if speed_predecessor >= 0 and predecessor_rear_position >= 0:
            # we have a predecessor
            gap_to_predecessor = predecessor_rear_position - self.position
            if self._simulator._debug:
                print("%d my front gap %f" % (self.vid, gap_to_predecessor))
                print("%d my predecessor speed %f" % (self.vid, speed_predecessor))
            safe_speed = self._safe_speed(speed_predecessor, gap_to_predecessor, desired_gap, self.min_gap)
            if self._simulator._debug:
                print("%d my safe speed %f" % (self.vid, safe_speed))

            if safe_speed < new_speed:
                if self._simulator._debug:
                    print("%d blocked by slow vehicle!" % self.vid, flush=True)
                self._blocked_front = True

                new_speed = max(safe_speed, self.speed - self._simulator.acceleration2speed(self.max_deceleration, self._simulator._step_length))  # we cannot brake stronger than we actually can
                if self._simulator._debug:
                    print("%d new speed after safe speed %f" % (self.vid, new_speed))
            else:
                self._blocked_front = False
        else:
            # we have no predecessor
            self._blocked_front = False

        # TODO dawdling?
        # new_speed -= random() * new_speed

        if (new_speed < 0):
            new_speed = 0

        if self._simulator._debug:
            print("%d new speed %f" % (self.vid, new_speed))

        return new_speed

    def action(self):
        """Trigger actions of a Vehicle"""

        if self._simulator.step < self.depart_time:
            # we did not start yet
            return
        else:
            # we started (right now)
            self._start()

            if self._simulator._debug:
                self.info()

            # log periodic statistics
            self._statistics()

            # What has to be triggered periodically?
            self._action()

    def _action(self):
        """Trigger concrete actions of a Vehicle"""

        pass  # this vehicle has no application running

    def _start(self):
        """Start this Vehicle"""

        if self._started:
            return

        # check whether we can actually be inserted? this should be done within the simulator though
        for vehicle in self._simulator._vehicles.values():
            if vehicle is self:
                # we do not need to compare us to ourselves
                continue
            if vehicle.lane != self.lane:
                # we do not care about other lanes
                continue
            if vehicle.depart_time > self._simulator._step:
                # vehicle did not start yet
                continue
            if self._simulator.has_collision(self.vid, self.position, self.rear_position, vehicle.vid, vehicle.position, vehicle.rear_position):
                print(self.vid, "collision at start with", vehicle.vid)
                exit(1)

        self._started = True

    def info(self):
        """Print info of a Vehicle"""

        e_remaining_travel_time = round((self.arrival_position - self.position) / self.desired_speed)
        print(self._simulator.step, ":", self.vid, "at", self.position, self.rear_position, self.lane, "with", self.speed,
              "takes", e_remaining_travel_time, flush=True)

    def _statistics(self):
        """Write continuous statistics"""

        # mobility/trip statistics
        with open(self._simulator._result_base_filename + '_vehicle_traces.csv', 'a') as f:
            f.write("%d,%d,%f,%d,%f,%d,%f\n" % (self._simulator.step, self.vid, self.position, self.lane, self.speed, self.travel_time, self.travel_distance))

        # TODO emission statistics?

    def finish(self):
        """Clean up the instance of the vehicle"""

        if (self.position < self.arrival_position):
            print("Finish was called even though we did not arrive yet!")
            return

        e_travel_time = (self.arrival_position - self.depart_position) / self.desired_speed
        time_loss = self.travel_time - round(e_travel_time)
        travel_time_ratio = round(self.travel_time / e_travel_time, 2)

        if self._simulator._debug:
            print(self._simulator.step, ":", self.vid, "arrived", self.position, self.lane, "with", self.speed, "took", self.travel_time, self.travel_distance, time_loss, travel_time_ratio, flush=True)

        with open(self._simulator._result_base_filename + '_vehicle_trips.csv', 'a') as f:
            f.write("%d,%d,%d,%d,%f,%d,%d,%f,%f,%d,%f,%f,%f\n" % (self.vid, self.depart_time, self.depart_lane, self.depart_position, self.depart_speed, self._simulator.step, self.lane, self.position, self.speed, self.travel_time, self.travel_distance, time_loss, self.desired_speed))

        with open(self._simulator._result_base_filename + '_vehicle_emissions.csv', 'a') as f:
            # TODO emissions model not yet implemented
            self._co = -1
            self._co2 = -1
            self._hc = -1
            self._pmx = -1
            self._npx = -1
            self._fuel = -1
            f.write("%d,%d,%d,%d,%d,%d,%d\n" % (self.vid, self._co, self._co2, self._hc, self._pmx, self._npx, self._fuel))

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

        print("Received non-sense message", message)


class PlatoonRole(Enum):
    """A collection of available platoon roles"""

    NONE = 0  # corresponds to driving individually
    LEADER = 1  # corresponds to being the leader of a platoon
    FOLLOWER = 2  # corresponds to being a follow of a platoon
    JOINER = 3  # corresponds to be in the process of joining a platoon
    LEAVER = 4  # corresponds to be in the process of leaving a platoon


class Platoon:
    """A collection of parameters for a concrete platoon"""

    def __init__(
            self,
            pid: int,
            lid: int,
            formation: list,
            speed: float,
            lane: int,
            max_speed: float,
            max_acceleration: float,
            max_deceleration: float):
        self._pid = pid  # the id of the platoon
        self._lid = lid  # the current leader id of the platoon
        self._formation = formation  # the current formation of the platoon
        self._speed = speed  # the current (desired) speed of the platoon
        self._lane = lane  # the current (desired) lane of the platoon
        self._max_speed = max_speed  # the current maximum speed of the platoon
        self._max_acceleration = max_acceleration  # the current maximum acceleration of the platoon
        self._max_deceleration = max_deceleration  # the current maximum deceleration of the platoon

    @property
    def pid(self) -> int:
        return self._pid

    @property
    def lid(self) -> int:
        return self._lid

    @property
    def formation(self) -> list:
        return self._formation

    @property
    def speed(self) -> float:
        return self._speed

    @property
    def lane(self) -> int:
        return self._lane

    @property
    def max_speed(self) -> float:
        return self._max_speed

    @property
    def max_acceleration(self) -> float:
        return self._max_acceleration

    @property
    def max_deceleration(self) -> float:
        return self._max_deceleration


class PlatooningVehicle(Vehicle):
    """A vehicle that has platooning functionality enabled"""

    def __init__(
            self,
            simulator,  # TODO add type hint
            vid: int,
            vehicle_type: VehicleType,
            depart_position: int,
            arrival_position: int,
            desired_speed: float,
            depart_lane: int,
            depart_speed: float,
            depart_time: int):
        super().__init__(simulator, vid, vehicle_type, depart_position, arrival_position, desired_speed, depart_lane,
                         depart_speed, depart_time)

        self._platoon_role = PlatoonRole.NONE  # the current platoon role
        self._platoon = Platoon(self.vid, self.vid, [self.vid], self.desired_speed, self.depart_lane, self.max_speed, self.max_acceleration, self.max_deceleration)

        # initialize timer
        self._last_advertisement_step = None

    @property
    def platoon_role(self) -> PlatoonRole:
        return self._platoon_role

    @property
    def platoon(self) -> Platoon:
        return self._platoon

    def is_in_platoon(self) -> bool:
        return self.platoon_role is not PlatoonRole.NONE

    def _action(self):
        """Trigger concrete actions of a PlatooningVehicle"""

        super()._action()

        # transmit regular platoon advertisements
        self._advertise()

        # search for a platoon (depending on the strategy)
        self._do_formation()

    def _do_formation(self):
        """Run platoon formation algorithms to search for a platooning opportunity and perform corresponding maneuvers"""

        return  # TODO

    def _advertise(self):
        """Maintain regular sending of platoon advertisements"""

        return  # TODO this is not necessary as a perfect communication guarantees information

        advertisement_interval = 600  # in s # TODO make parameter
        if self._last_advertisement_step is None or self._last_advertisement_step + advertisement_interval <= self._simulator.step:
            self._send_advertisements()
            self._last_advertisement_step = self._simulator.step

    def _send_advertisements(self):
        """Transmit a broadcast to advertise as platoon"""

        for vehicle in self._simulator._vehicles.values():
            self._transmit(-1, PlatoonAdvertisement(
                self.vid,
                vehicle.vid,
                self.vid,
                self.vid,
                self.speed,
                self.lane,
                self.vid,
                self.position,
                self.position + self.length
            ))

    def _handle_message(self, message: Message):
        """Handle a message of arbitrary type Message"""

        func = self.__class__.__dict__.get(
            '_receive_' + message.__class__.__name__,
            super().__dict__.get('_handle_message'))
        return func(self, message)

    def _receive_PlatoonAdvertisement(self, advertisement: PlatoonAdvertisement):
        """Handle a message of concrete type PlatoonAdvertisement"""

        # TODO add contents to the neighbor table
        print("advertisement from", advertisement.origin)
