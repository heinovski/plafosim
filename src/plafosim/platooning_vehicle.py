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
from .vehicle import VehicleType, Vehicle
# from .simulator import Simulator # TODO fix circular import


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
            platoon_id: int,
            formation: list,
            speed: float,
            lane: int,
            max_speed: float,
            max_acceleration: float,
            max_deceleration: float):
        self._platoon_id = platoon_id  # the id of the platoon
        self._formation = formation  # the current formation of the platoon
        self._speed = speed  # the current (desired) speed of the platoon
        self._lane = lane  # the current (desired) lane of the platoon
        self._max_speed = max_speed  # the current maximum speed of the platoon
        self._max_acceleration = max_acceleration  # the current maximum acceleration of the platoon
        self._max_deceleration = max_deceleration  # the current maximum deceleration of the platoon

    @property
    def platoon_id(self) -> int:
        return self._platoon_id

    @property
    def leader_id(self) -> int:
        return self.formation[0]

    @property
    def last_id(self) -> int:
        return self.formation[-1]

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

    @property
    def length(self) -> int:
        return len(self.formation)

    def get_member_index(self, vid: int) -> int:
        return self.formation.index(vid)

    def get_front_id(self, vid: int) -> int:
        if vid != self.leader_id:
            return self.formation[self.get_member_index(vid) - 1]
        else:
            return -1


class CF_Mode(Enum):

    CC = 0  # safe speed; aka human
    ACC = 1  # fixed time gap
    CACC = 2  # small fixed distance


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
            depart_time: int,
            acc_headway_time: float,
            cacc_spacing: float):
        super().__init__(simulator, vid, vehicle_type, depart_position, arrival_position, desired_speed, depart_lane,
                         depart_speed, depart_time)

        self._cf_mode = CF_Mode.ACC
        self._acc_headway_time = acc_headway_time
        if self.acc_headway_time < 1.0:
            print("Warning: values for ACC headway time lower 1.0s are not recommended to avoid crashes!")
        self._acc_lambda = 0.1  # see Eq. 6.18 of R. Rajamani, Vehicle Dynamics and Control, 2nd. Springer, 2012.
        self._cacc_spacing = cacc_spacing
        if self.cacc_spacing < 5.0:
            print("Warning: values for CACC spacing lower than 5.0m are not recommended to avoid crashes!")
        self._platoon_role = PlatoonRole.NONE  # the current platoon role
        self._platoon = Platoon(self.vid, [self.vid], self.desired_speed, self.depart_lane, self.max_speed, self.max_acceleration, self.max_deceleration)

        # initialize timer
        self._last_advertisement_step = None

    @property
    def cf_mode(self) -> CF_Mode:
        return self._cf_mode

    @property
    def acc_headway_time(self) -> float:
        return self._acc_headway_time

    @property
    def acc_lambda(self) -> float:
        return self._acc_lambda

    @property
    def cacc_spacing(self) -> float:
        return self._cacc_spacing

    @property
    def desired_gap(self) -> float:
        if self.cf_mode is CF_Mode.ACC:
            return self.acc_headway_time * self.speed
        else:
            return self.cacc_spacing

    @property
    def platoon_role(self) -> PlatoonRole:
        return self._platoon_role

    @property
    def platoon(self) -> Platoon:
        return self._platoon

    def is_in_platoon(self) -> bool:
        return self.platoon_role is not PlatoonRole.NONE

    def get_front_gap(self) -> float:
        return self._simulator._get_predecessor_rear_position(self.vid) - self.position

    def get_front_speed(self) -> float:
        return self._simulator._get_predecessor_speed(self.vid)

    def _acc_acceleration(self, desired_speed: float, gap_to_predecessor: float, desired_gap: float) -> float:
        """Helper method to calcucate the ACC acceleration based on the given parameters"""

        # Eq. 6.18 of R. Rajamani, Vehicle Dynamics and Control, 2nd. Springer, 2012.
        return -1.0 / self.acc_headway_time * (self.speed - desired_speed + self.acc_lambda * (-gap_to_predecessor + desired_gap))

    def new_speed(self, speed_predecessor: float, predecessor_rear_position: float, desired_gap: float = 0) -> float:
        if self._cf_mode is CF_Mode.ACC:
            del desired_gap  # delete passed variable to avoid misuse
            # TODO we should use different maximum accelerations/decelerations and headway times/gaps for different modes
            if speed_predecessor >= 0 and predecessor_rear_position >= 0:
                gap_to_predecessor = predecessor_rear_position - self.position
                if self._simulator._debug:
                    print("%d my front gap %f" % (self.vid, gap_to_predecessor))
                    print("%d my desired gap %f" % (self.vid, self.desired_gap))
                    print("%d my predecessor speed %f" % (self.vid, speed_predecessor))

                u = self._acc_acceleration(speed_predecessor, gap_to_predecessor, self.acc_headway_time * self.speed)

                if self._simulator._debug:
                    print("%d ACC safe speed %f" % (self.vid, self.speed + u))

                u = min(self.max_acceleration, u)  # we cannot accelerate stronger than we actually can
                u = max(-self.max_deceleration, u)  # we cannot decelerate stronger than we actually can

                new_speed = self.speed + u
                new_speed = min(self.max_speed, new_speed)  # only drive as fast as possible
                if self._simulator._debug:
                    print("%d ACC possible speed %f" % (self.vid, new_speed))

                new_speed = min(self.desired_speed, new_speed)  # only drive as fast as desired
                if self._simulator._debug:
                    print("%d ACC desired speed %f" % (self.vid, new_speed))

                if new_speed < self.desired_speed and u <= 0:
                    if self._simulator._debug:
                        print("%d blocked by slow vehicle!" % self.vid, flush=True)
                    self._blocked_front = True
                else:
                    self._blocked_front = False

                if self._simulator._debug:
                    print("%d ACC new speed %f" % (self.vid, new_speed))

                return new_speed

        elif self._cf_mode is CF_Mode.CACC:
            assert(self.is_in_platoon())
            assert(self.platoon_role is PlatoonRole.FOLLOWER)  # only followers can use CACC

            # TODO make sure that the leader uses the platoon's parameters for ACC

            # sanity checks for front vehicle in platoon
            assert(speed_predecessor >= 0 and predecessor_rear_position >= 0)
            # check whether there is a vehicle between us and our front vehicle
            assert(self.platoon.get_front_id(self.vid) == self._simulator._get_predecessor_id(self.vid))

            gap_to_predecessor = predecessor_rear_position - self.position
            if self._simulator._debug:
                print("%d my front gap %f" % (self.vid, gap_to_predecessor))
                print("%d my desired gap %f" % (self.vid, self.desired_gap))
                print("%d my predecessor speed %f" % (self.vid, speed_predecessor))

            ### HACK FOR AVOIDING COMMUNICATION ###
            # acceleration_predecessor = self._simulator._vehicles[self.platoon.get_front_id(self.vid)].acceleration
            # acceleration_leader = self._simulator._vehicles[self.platoon.leader_id].acceleration
            speed_leader = self._simulator._vehicles[self.platoon.leader_id].speed
            #######################################

            ### HACK FOR CACC ###
            u = self._acc_acceleration(speed_leader, gap_to_predecessor, self.desired_gap)
            #####################

            if self._simulator._debug:
                print("%d CACC safe speed %f" % (self.vid, self.speed + u))

            u = min(self.max_acceleration, u)  # we cannot accelerate stronger than we actually can
            u = max(-self.max_deceleration, u)  # we cannot decelerate stronger than we actually can

            new_speed = self.speed + u
            new_speed = min(self.max_speed, new_speed)  # only drive as fast as possible
            if self._simulator._debug:
                print("%d CACC possible speed %f" % (self.vid, new_speed))

            if self._simulator._debug:
                print("%d CACC new speed %f" % (self.vid, new_speed))

            return new_speed

        # default: drive freely
        return super().new_speed(speed_predecessor, predecessor_rear_position, super().desired_gap)

    def finish(self):
        # clean up platoon
        if self.is_in_platoon():
            ### FIXME HACK FOR DESTROYING THE PLATOON AT THE END OF THE TRIP ###
            for v in self.platoon.formation:
                if v == self.vid:
                    continue
                vehicle = self._simulator._vehicles[v]
                if self.vid == self.platoon.leader_id:
                    vehicle._platoon = Platoon(v, [v], vehicle.desired_speed, vehicle.lane, vehicle.max_speed, vehicle.max_acceleration, vehicle.max_deceleration)
                    vehicle._cf_mode = CF_Mode.ACC
                else:
                    vehicle.platoon._formation.remove(self.vid)
            ####################################################################

        super().finish()

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
