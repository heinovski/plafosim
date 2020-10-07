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

from enum import Enum

from .message import Message, PlatoonAdvertisement
from .platoon import Platoon
from .platoon_role import PlatoonRole
from .vehicle_type import VehicleType
from .vehicle import Vehicle
from .formation_algorithm import SpeedPosition

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .simulator import Simulator


class CF_Mode(Enum):

    CC = 0  # safe speed; aka human
    ACC = 1  # fixed time gap
    CACC = 2  # small fixed distance


class PlatooningVehicle(Vehicle):
    """A vehicle that has platooning functionality enabled"""

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
            acc_headway_time: float,
            cacc_spacing: float,
            formation_algorithm: str,
            alpha: float,
            speed_deviation_threshold: float,
            position_deviation_threshold: int):
        super().__init__(simulator, vid, vehicle_type, depart_position, arrival_position, desired_speed, depart_lane,
                         depart_speed, depart_time)

        self._cf_mode = CF_Mode.ACC
        self._acc_headway_time = acc_headway_time
        if self.acc_headway_time < 1.0:
            logging.warn("Values for ACC headway time lower 1.0s are not recommended to avoid crashes!")
        self._acc_lambda = 0.1  # see Eq. 6.18 of R. Rajamani, Vehicle Dynamics and Control, 2nd. Springer, 2012.
        self._cacc_spacing = cacc_spacing
        if self.cacc_spacing < 5.0:
            logging.warn("Values for CACC spacing lower than 5.0m are not recommended to avoid crashes!")
        self._platoon_role = PlatoonRole.NONE  # the current platoon role
        self._platoon = Platoon(self.vid, [self], self.desired_speed)
        self._in_maneuver = False

        if formation_algorithm is not None:
            # initialize formation algorithm
            # TODO make enum
            if formation_algorithm == "speedposition":
                self._formation_algorithm = SpeedPosition(self, alpha, speed_deviation_threshold, position_deviation_threshold)
            else:
                logging.critical("Unkown formation algorithm %s!" % formation_algorithm)
                exit(1)

            # initialize timer
            self._last_advertisement_step = None

        else:
            self._formation_algorithm = None

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

    @property
    def in_maneuver(self) -> bool:
        return self._in_maneuver

    @in_maneuver.setter
    def in_maneuver(self, var: bool):
        if self._in_maneuver and var:
            # we can only start a new maneuver if we are not already in one
            logging.warn("%d is are already in a meneuver" % self.vid)
            return
        self._in_maneuver = var

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
                logging.debug("%d's front gap %f" % (self.vid, gap_to_predecessor))
                logging.debug("%d's desired gap %f" % (self.vid, self.desired_gap))
                logging.debug("%d's desired speed %f" % (self.vid, self.desired_speed))
                logging.debug("%d's predecessor (%d) speed %f" % (self.vid, self._simulator._get_predecessor(self).vid, speed_predecessor))

                u = self._acc_acceleration(speed_predecessor, gap_to_predecessor, self.acc_headway_time * self.speed)

                logging.debug("%d's ACC safe speed %f" % (self.vid, self.speed + u))

                u = min(self.max_acceleration, u)  # we cannot accelerate stronger than we actually can
                logging.debug("%d's ACC max acceleration speed %f" % (self.vid, self.speed + u))

                u = max(-self.max_deceleration, u)  # we cannot decelerate stronger than we actually can
                logging.debug("%d's ACC max deceleration speed %f" % (self.vid, self.speed + u))

                new_speed = self.speed + u
                new_speed = min(self.max_speed, new_speed)  # only drive as fast as possible
                logging.debug("%d's ACC max possible speed %f" % (self.vid, new_speed))

                new_speed = min(self.desired_speed, new_speed)  # only drive as fast as desired
                logging.debug("%d's ACC desired speed %f" % (self.vid, new_speed))

                if new_speed < self.desired_speed and u <= 0:
                    logging.info("%d is blocked by slow vehicle!" % self.vid)
                    self._blocked_front = True
                else:
                    self._blocked_front = False

                logging.debug("%d's ACC new speed %f" % (self.vid, new_speed))

                return new_speed

        elif self._cf_mode is CF_Mode.CACC:
            assert(self.is_in_platoon())
            assert(self.platoon_role is PlatoonRole.FOLLOWER)  # only followers can use CACC

            # TODO make sure that the leader uses the platoon's parameters for ACC

            # sanity checks for front vehicle in platoon
            assert(speed_predecessor >= 0 and predecessor_rear_position >= 0)
            # check whether there is a vehicle between us and our front vehicle

            assert(self.platoon.get_front(self) is self._simulator._get_predecessor(self))

            gap_to_predecessor = predecessor_rear_position - self.position
            logging.debug("%d's front gap %f" % (self.vid, gap_to_predecessor))
            logging.debug("%d's desired gap %f" % (self.vid, self.desired_gap))
            logging.debug("%d's predecessor speed %f" % (self.vid, speed_predecessor))

            ### HACK FOR AVOIDING COMMUNICATION ###
            # acceleration_predecessor = self.platoon.get_front(self).acceleration
            # acceleration_leader = self.platoon.leader.acceleration
            speed_leader = self.platoon.leader.speed
            #######################################

            ### HACK FOR CACC ###
            u = self._acc_acceleration(speed_leader, gap_to_predecessor, self.desired_gap)
            #####################

            logging.debug("%d's CACC safe speed %f" % (self.vid, self.speed + u))

            u = min(self.max_acceleration, u)  # we cannot accelerate stronger than we actually can
            logging.debug("%d's CACC max acceleration speed %f" % (self.vid, self.speed + u))

            u = max(-self.max_deceleration, u)  # we cannot decelerate stronger than we actually can
            logging.debug("%d's CACC max deceleration speed %f" % (self.vid, self.speed + u))

            new_speed = self.speed + u
            new_speed = min(self.max_speed, new_speed)  # only drive as fast as possible
            logging.debug("%d's CACC possible speed %f" % (self.vid, new_speed))

            logging.debug("%d's CACC new speed %f" % (self.vid, new_speed))

            return new_speed

        # default: drive freely
        return super().new_speed(speed_predecessor, predecessor_rear_position, super().desired_gap)

    def finish(self):
        # clean up platoon
        if self.is_in_platoon():
            self._leave()

        super().finish()

    def _action(self):
        """Trigger concrete actions of a PlatooningVehicle"""

        super()._action()

        if self._formation_algorithm is not None:
            # transmit regular platoon advertisements
            self._advertise()

            # search for a platoon (depending on the algorithm)
            self._formation_algorithm.do_formation()

    def _get_available_platoons(self):
        # HACK FOR AVOIDING MAINTAINING A NEIGHBORTABLE (for now)
        platoons = []
        for vehicle in self._simulator._vehicles.values():

            # filter out self
            if vehicle == self:
                continue

            # filter vehicles that are technically not able to do platooning
            if not isinstance(vehicle, PlatooningVehicle):
                continue

            # filter non-platoons
            if vehicle.platoon_role != PlatoonRole.LEADER and vehicle.platoon_role != PlatoonRole.NONE:
                continue

#            # filter based on communication range
#            communication_range = self._simulator.road_length
#            if abs(vehicle.position - self.position) > communication_range:
#                logging.debug("%d's platoon %d is out of communication range" % (self.vid, vehicle.vid))
#                continue

            platoons.append(vehicle.platoon)

        return platoons

    def _join(self, platoon_id: int, leader_id: int):
        # just join, without any communication

        assert(not self.is_in_platoon())

        logging.info("%d is trying to join platoon %d (leader %d)" % (self.vid, platoon_id, leader_id))

        # TODO make sure to set all platoon speed related values
        # TODO make sure to use them as your new defaults

        self.in_maneuver = True

        leader = self._simulator._vehicles[leader_id]
        assert(isinstance(leader, PlatooningVehicle))

        # TODO joint at front
        # TODO join at arbitrary positions
        # FIXME HACK TO ONLY ALLOW JOINING AT THE BACK
        if self.position >= leader.platoon.rear_position:
            logging.warn("%d is in front of (at least) the last vehicle %d of the target platoon %d (leader %d)" % (self.vid, leader.platoon.last.vid, platoon_id, leader_id))
            self.in_maneuver = False
            return

        if leader.in_maneuver:
            logging.warn("%d's new leader %d was already in a maneuver" % (self.vid, leader_id))
            self.in_maneuver = False
            return

        # update the leader
        leader.in_maneuver = True
        leader._platoon_role = PlatoonRole.LEADER
        logging.debug("%d became a leader of platoon %d" % (leader_id, leader.platoon.platoon_id))

        last = leader.platoon.last

        # TODO we do not want to teleport the vehicle
        current_position = self.position
        self._position = last.rear_position - self.cacc_spacing
        logging.warn("%d teleported to %d (from %d)" % (self.vid, self.position, current_position))
        current_lane = self.lane
        self._lane = leader.lane
        logging.warn("%d switched to lane %d (from %d)" % (self.vid, self.lane, current_lane))
        current_speed = self.speed
        self._speed = last.speed
        logging.warn("%d changed speed to %f (from %f)" % (self.vid, self.speed, current_speed))

        self._platoon_role = PlatoonRole.FOLLOWER

        # update all members
        leader.platoon._formation.append(self)

        for vehicle in leader.platoon.formation:
            # we copy all parameters from the platoon (for now)
            # thus, the follower no drives as fast as the already existing platoon (i.e., only the leader in the worst case)
            vehicle._platoon = leader.platoon

        # switch to CACC
        self._cf_mode = CF_Mode.CACC
        self._blocked_front = False

        logging.info("%d joined platoon %d (leader: %d)" % (self.vid, platoon_id, leader_id))

        self.in_maneuver = False
        leader.in_maneuver = False

    def _leave(self):
        # just leave, without any communication

        assert(self.is_in_platoon())
        if self.platoon.length == 1:
            return

        logging.info("%d is trying to leave platoon %d (leader %d)" % (self.vid, self.platoon.platoon_id, self.platoon.leader.vid))

        self.in_maneuver = True

        if self is self.platoon.last:
            # leave at back
            # TODO check whether it is safe to leave
            # TODO tell the leader (who needs to tell all other vehicles)
            # TODO leave
            logging.warn("Leave from back of a platoon is not yet implemented!")
            exit(1)
        elif self is self.platoon.leader:
            # leave at front

            # tell the second vehicle in the platoon to become the new leader
            new_leader = self.platoon.formation[1]
            new_leader._platoon_role = PlatoonRole.LEADER
            new_leader._cf_mode = CF_Mode.ACC

            self.platoon._formation.remove(self)

            # update formation all members
            for vehicle in self.platoon.formation:
                vehicle._platoon = self.platoon

            # leave
            self._platoon_role = PlatoonRole.NONE  # the current platoon role
            self._platoon = Platoon(self.vid, [self], self.desired_speed)

            self._cf_mode = CF_Mode.ACC  # not necessary, but we still do it explicitly

            logging.info("%d left platoon %d (new leader %d)" % (self.vid, new_leader.platoon.platoon_id, new_leader.vid))
        else:
            # leave in the middle
            # TODO check wether is is safe to leave
            # TODO tell the leader (who needs to tell all other vehicles and needs to tell them to make space)
            # TODO leave
            logging.warn("Leave from the middle of a platoon is not yet implemented!")
            exit(1)

        self.in_maneuver = False

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
        logging.info("%d received an advertisement from %d" % (self.vid, advertisement.origin))
