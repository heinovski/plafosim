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
import sys

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

LOG = logging.getLogger(__name__)


class CF_Mode(Enum):

    CC = 0  # safe speed
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
            communication_range: float,
            acc_headway_time: float,
            cacc_spacing: float,
            formation_algorithm: str,
            execution_interval: int,
            alpha: float,
            speed_deviation_threshold: float,
            position_deviation_threshold: int):
        super().__init__(simulator, vid, vehicle_type, depart_position, arrival_position, desired_speed, depart_lane,
                         depart_speed, depart_time, communication_range)

        self._cf_mode = CF_Mode.ACC
        self._acc_headway_time = acc_headway_time
        if acc_headway_time < 1.0:
            LOG.warn("Values for ACC headway time lower 1.0s are not recommended to avoid crashes!")
        self._acc_lambda = 0.1  # see Eq. 6.18 of R. Rajamani, Vehicle Dynamics and Control, 2nd. Springer, 2012.
        self._cacc_spacing = cacc_spacing
        if self.cacc_spacing < 5.0:
            LOG.warn("Values for CACC spacing lower than 5.0m are not recommended to avoid crashes!")
        self._platoon_role = PlatoonRole.NONE  # the current platoon role
        self._platoon = Platoon(self.vid, [self], self.desired_speed)
        self._in_maneuver = False

        if formation_algorithm is not None:
            # initialize formation algorithm
            # TODO make enum
            if formation_algorithm == "speedposition":
                self._formation_algorithm = SpeedPosition(self, alpha, speed_deviation_threshold, position_deviation_threshold)
            else:
                sys.exit(f"Unknown formation algorithm {formation_algorithm}!")
            self._execution_interval = execution_interval

            # initialize timers
            self._last_formation_step = self.depart_time  # initialize with vehicle start
            self._last_advertisement_step = None

        else:
            self._formation_algorithm = None

        self._last_platoon_join_time = -1
        self._time_in_platoon = 0
        self._last_platoon_join_position = -1
        self._distance_in_platoon = 0

    @property
    def cf_mode(self) -> CF_Mode:
        return self._cf_mode

    @property
    def desired_headway_time(self) -> float:
        if self.cf_mode == CF_Mode.ACC or self.cf_mode == CF_Mode.CACC:
            return self._acc_headway_time
        return super().desired_headway_time

    @property
    def acc_lambda(self) -> float:
        return self._acc_lambda

    @property
    def cacc_spacing(self) -> float:
        return self._cacc_spacing

    @property
    def desired_speed(self) -> float:
        # return desired speed of platoon if in platoon
        return self.platoon.desired_speed if self.is_in_platoon() else self._desired_speed

    @property
    def desired_gap(self) -> float:
        if self.cf_mode == CF_Mode.CACC:
            return self.cacc_spacing
        return super().desired_gap

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
            LOG.warn(f"{self.vid} is are already in a meneuver")
            return
        self._in_maneuver = var

    @property
    def time_in_platoon(self) -> int:
        return self._time_in_platoon

    @property
    def distance_in_platoon(self) -> float:
        return self._distance_in_platoon

    def _acc_acceleration(self, desired_speed: float, gap_to_predecessor: float, desired_gap: float) -> float:
        """Helper method to calculate the ACC acceleration based on the given parameters"""

        # Eq. 6.18 of R. Rajamani, Vehicle Dynamics and Control, 2nd. Springer, 2012.
        return -1.0 / self.desired_headway_time * (self.speed - desired_speed + self.acc_lambda * (-gap_to_predecessor + desired_gap))

    def new_speed(self, speed_predecessor: float, predecessor_rear_position: float) -> float:
        if self._cf_mode is CF_Mode.ACC:
            # TODO we should use different maximum accelerations/decelerations and headway times/gaps for different modes
            if speed_predecessor >= 0 and predecessor_rear_position >= 0:
                gap_to_predecessor = predecessor_rear_position - self.position
                LOG.debug(f"{self.vid}'s front gap {gap_to_predecessor}")
                if gap_to_predecessor < 0:
                    LOG.warn(f"{self.vid}'s front gap is negative")
                LOG.debug(f"{self.vid}'s desired gap {self.desired_gap}")
                LOG.debug(f"{self.vid}'s desired speed {self.desired_speed}")
                LOG.debug(f"{self.vid}'s predecessor ({self._simulator._get_predecessor(self).vid}) speed {speed_predecessor}")

                u = self._acc_acceleration(speed_predecessor, gap_to_predecessor, self.desired_gap)

                LOG.debug(f"{self.vid}'s ACC safe speed {self.speed + u}")

                u = min(self.max_acceleration, u)  # we cannot accelerate stronger than we actually can
                LOG.debug(f"{self.vid}'s ACC max acceleration speed {self.speed + u}")

                u = max(-self.max_deceleration, u)  # we cannot decelerate stronger than we actually can
                LOG.debug(f"{self.vid}'s ACC max deceleration speed {self.speed + u}")

                new_speed = self.speed + u
                new_speed = min(self.max_speed, new_speed)  # only drive as fast as possible
                LOG.debug(f"{self.vid}'s ACC max possible speed {new_speed}")

                if self.is_in_platoon():
                    assert(self.platoon_role is PlatoonRole.LEADER)

                    LOG.debug(f"{self.vid}'s ACC new individual speed {new_speed}")

                    # make sure that the leader uses the platoon's parameters for ACC

                    # only accelerate as fast as the maximum acceleration of the individual platoon members
                    new_speed = min(new_speed, self.platoon.speed + self.platoon.max_acceleration)

                    # only decelerate as fast as the maximum deceleration of the individual platoon members
                    new_speed = max(new_speed, self.platoon.speed - self.platoon.max_deceleration)

                    # only drive as fast as the maximum speed of the individual platoon members
                    new_speed = min(new_speed, self.platoon.max_speed)

                    LOG.debug(f"{self.vid}'s CACC possible speed {new_speed}")

                    # only drive as fast as the platoon's desired speed
                    new_speed = min(new_speed, self.platoon.desired_speed)
                    LOG.debug(f"{self.vid}'s CACC desired speed {new_speed}")

                    LOG.debug(f"{self.vid}'s CACC new speed {new_speed}")

                    # HACK for avoiding communication for platoon management
                    # update all my followers
                    for follower in self.platoon.formation:
                        if follower is self:
                            continue

                        LOG.debug(f"{self.vid} is updating speed of its follower {follower.vid}")
                        follower._speed = new_speed
                else:
                    new_speed = min(self.desired_speed, new_speed)  # only drive as fast as desired
                    LOG.debug(f"{self.vid}'s ACC desired speed {new_speed}")

                    LOG.debug(f"{self.vid}'s ACC new speed {new_speed}")

                if new_speed < self.desired_speed and u <= 0:
                    LOG.info(f"{self.vid} is blocked by slow vehicle!")
                    self._blocked_front = True
                else:
                    self._blocked_front = False

                return new_speed

        elif self._cf_mode is CF_Mode.CACC:
            assert(self.is_in_platoon())
            assert(self.platoon_role is not PlatoonRole.LEADER)  # only followers can use CACC

            # sanity checks for front vehicle in platoon
            assert(speed_predecessor >= 0 and predecessor_rear_position >= 0)
            # check whether there is a vehicle between us and our front vehicle

            assert(self.platoon.get_front(self) is self._simulator._get_predecessor(self))

            gap_to_predecessor = predecessor_rear_position - self.position
            LOG.debug(f"{self.vid}'s front gap {gap_to_predecessor}")
            if gap_to_predecessor < 0:
                LOG.warn(f"{self.vid}'s front gap is negative")
            LOG.debug(f"{self.vid}'s desired gap {self.desired_gap}")
            LOG.debug(f"{self.vid}'s predecessor speed {speed_predecessor}")

            ### HACK FOR AVOIDING COMMUNICATION ###
            # acceleration_predecessor = self.platoon.get_front(self).acceleration
            # acceleration_leader = self.platoon.leader.acceleration
            speed_leader = self.platoon.leader.speed
            #######################################

            ### HACK FOR CACC ###
            u = self._acc_acceleration(speed_leader, gap_to_predecessor, self.desired_gap)
            #####################

            LOG.debug(f"{self.vid}'s CACC safe speed {self.speed + u}")

            u = min(self.max_acceleration, u)  # we cannot accelerate stronger than we actually can
            LOG.debug(f"{self.vid}'s CACC max acceleration speed {self.speed + u}")

            u = max(-self.max_deceleration, u)  # we cannot decelerate stronger than we actually can
            LOG.debug(f"{self.vid}'s CACC max deceleration speed {self.speed + u}")

            new_speed = self.speed + u
            new_speed = min(self.max_speed, new_speed)  # only drive as fast as possible
            LOG.debug(f"{self.vid}'s CACC possible speed {new_speed}")

            LOG.debug(f"{self.vid}'s CACC new speed {new_speed}")

            return new_speed

        # default: use CC or driving freely
        return super().new_speed(speed_predecessor, predecessor_rear_position)

    def _calculate_emission(self, a: float, v: float, f: list, scale: float) -> float:
        # Gino Sovran, "Tractive-Energy-Based Formulae for the Impact of Aerodynamics on Fuel Economy Over the EPA Driving Schedules," SAE International, Technical Paper, 830304, February 1983.
        # double fuelChange = 0.46 * getAirDragCoefficientChange();
        # double currentCO2 = value * (1 - fuelChange);
        #
        # calculate savings by platooning by using
        # Charles-Henri Bruneau, Khodor Khadra and Iraj Mortazavi, "Flow analysis of square-back simplified vehicles in platoon," International Journal of Heat and Fluid Flow, vol. 66, pp. 43–59, August 2017.
        # Table 5: d = L, vehicle length = 4m, distance = 5m

        air_drag_change = 0.0
        if self._platoon_role == PlatoonRole.LEADER:
            # savings by followers
            assert(self.platoon.size > 1)
            air_drag_change = 0.12  # bruneau
        elif self._platoon_role == PlatoonRole.FOLLOWER:
            # savings by leader/front vehicles
            if self is self.platoon.last:
                # last vehicle
                air_drag_change = 0.23  # bruneau
            else:
                # in between
                air_drag_change = 0.27  # bruneau
        emission_change = air_drag_change * 0.46  # sovran

        return super()._calculate_emission(a, v, f, scale) * (1.0 - emission_change)

    def finish(self):
        super().finish()

        # clean up platoon
        if self.is_in_platoon():
            self._leave()

        platoon_time_ratio = round(self.time_in_platoon / self.travel_time, 2)
        platoon_distance_ratio = round(self.distance_in_platoon / self.travel_distance, 2)

        LOG.info(f"{self.vid} drove {self.time_in_platoon}s ({self.distance_in_platoon}m) in a platoon, {platoon_time_ratio * 100} ({platoon_distance_ratio * 100})")

        # TODO log savings from platoon?
        if self._simulator._record_platoon_trips:
            with open(self._simulator._result_base_filename + '_platoon_trips.csv', 'a') as f:
                f.write(f"{self.vid},{self.time_in_platoon},{self.distance_in_platoon},{platoon_time_ratio},{platoon_distance_ratio}\n")

    def _action(self, step: int):
        """Trigger concrete actions of a PlatooningVehicle"""

        super()._action(step)

        if self._formation_algorithm is not None:
            # transmit regular platoon advertisements
            self._advertise()

            if step >= self._last_formation_step + self._execution_interval:
                # search for a platoon (depending on the algorithm)
                self._formation_algorithm.do_formation()
                self._last_execution_step = step

    def info(self):
        """Return info of a PlatooningVehicle"""

        return f"{super().info()}, platoon {self.platoon.platoon_id if self.is_in_platoon() else None}"

    def _statistics(self):
        """Write continuous statistics"""

        super()._statistics()

        if self._platoon_role == PlatoonRole.LEADER and self._simulator._record_platoon_traces:
            # write statistics about this platoon
            with open(self._simulator._result_base_filename + '_platoon_traces.csv', 'a') as f:
                f.write(f"{self._simulator.step},{self.platoon.platoon_id},{self.platoon.leader.vid},{self.platoon.position},{self.platoon.rear_position},{self.platoon.lane},{self.platoon.speed},{self.platoon.size},{self.platoon.length}\n")

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

            # filter based on communication range
            if abs(vehicle.position - self.position) > self._communication_range:
                LOG.debug(f"{self.vid}'s neighbor {vehicle.vid} is out of communication range ({self._communication_range})")
                continue

            platoons.append(vehicle.platoon)

        return platoons

    def _join(self, platoon_id: int, leader_id: int):
        # just join, without any communication

        assert(not self.is_in_platoon())

        LOG.info(f"{self.vid} is trying to join platoon {platoon_id} (leader {leader_id})")

        # TODO make sure to set all platoon speed related values
        # TODO make sure to use them as your new defaults

        self.in_maneuver = True

        leader = self._simulator._vehicles[leader_id]
        assert(isinstance(leader, PlatooningVehicle))

        # correct platoon
        assert(leader.platoon.platoon_id == platoon_id)
        # correct leader of that platoon
        assert(leader.vid == leader.platoon.leader.vid)

        # TODO joint at front
        # TODO join at arbitrary positions
        # FIXME HACK TO ONLY ALLOW JOINING AT THE BACK
        if self.position >= leader.platoon.rear_position:
            LOG.warn(f"{self.vid} is in front of (at least) the last vehicle {leader.platoon.last.vid} of the target platoon {platoon_id} ({leader_id})")
            self.in_maneuver = False
            return

        if leader.in_maneuver:
            LOG.warn(f"{self.vid}'s new leader {leader_id} was already in a maneuver")
            self.in_maneuver = False
            return

        # update the leader
        leader.in_maneuver = True
        leader._platoon_role = PlatoonRole.LEADER
        LOG.debug(f"{leader_id} became a leader of platoon {leader.platoon.platoon_id}")

        last = leader.platoon.last

        platoon_successor = self._simulator._get_successor(last)

        # teleport the vehicle
        current_position = self.position
        self._position = last.rear_position - self.cacc_spacing
        LOG.warn(f"{self.vid} teleported to {self.position} (from {current_position})")
        current_lane = self.lane
        self._lane = leader.lane
        LOG.warn(f"{self.vid} switched to lane {self.lane} (from {current_lane})")
        current_speed = self.speed
        self._speed = last.speed
        LOG.warn(f"{self.vid} changed speed to {self.speed} (from {current_speed})")

        # we also need to check interfering vehicles!
        if platoon_successor is not self and platoon_successor is not None:
            diff = self.rear_position - platoon_successor.min_gap - platoon_successor.position
            if diff < 0:
                # adjust this vehicle
                platoon_successor._position = platoon_successor._position + diff
                LOG.warn(f"adjusted position of {platoon_successor.vid} to {platoon_successor.position}")
                if isinstance(platoon_successor, PlatooningVehicle):
                    assert(not platoon_successor.is_in_platoon() or platoon_successor.platoon_role == PlatoonRole.LEADER)
                    # adjust also all platoon members
                    for vehicle in platoon_successor.platoon.formation[1:]:
                        # adjust this vehicle
                        vehicle._position = vehicle._position + diff
                        LOG.warn(f"adjusted position of {vehicle.vid} to {vehicle.position}")

        self._platoon_role = PlatoonRole.FOLLOWER

        # update all members
        leader.platoon._formation.append(self)
        # update the desired speed of the platoon to the average of all platoom members
        leader.platoon.update_desired_speed()

        # switch to CACC
        self._cf_mode = CF_Mode.CACC
        self._blocked_front = False

        for vehicle in leader.platoon.formation:
            # we copy all parameters from the platoon (for now)
            # thus, the follower now drives as fast as the already existing platoon (i.e., only the leader in the worst case)
            vehicle._platoon = leader.platoon
            self._simulator._adjust_speed(vehicle)

        # set color of vehicle
        if self._simulator._gui:
            import traci
            traci.vehicle.setColor(str(vehicle.vid), leader._color)

        LOG.info(f"{self.vid} joined platoon {platoon_id} (leader: {leader_id})")

        self.in_maneuver = False
        leader.in_maneuver = False

        self._last_platoon_join_time = self._simulator.step
        leader._last_platoon_join_time = self._simulator.step
        self._last_platoon_join_position = self.position
        leader._last_platoon_join_position = leader.position

    def _leave(self):
        # just leave, without any communication

        if self.platoon.size == 1:
            return
        assert(self.is_in_platoon())

        LOG.info(f"{self.vid} is trying to leave platoon {self.platoon.platoon_id} (leader {self.platoon.leader.vid})")

        self.in_maneuver = True

        if self is self.platoon.leader:
            # leave at front

            if self.platoon.size == 2:
                # tell the only follower to drive individually
                follower = self.platoon.formation[1]
                follower._platoon_role = PlatoonRole.NONE
                follower._cf_mode = CF_Mode.ACC
                follower._platoon = Platoon(follower.vid, [follower], follower.desired_speed)
                self._simulator._adjust_speed(follower)
                # reset color of vehicle
                if self._simulator._gui:
                    import traci
                    traci.vehicle.setColor(str(follower.vid), follower._color)
            else:
                # tell the second vehicle in the platoon to become the new leader
                new_leader = self.platoon.formation[1]
                new_leader._platoon_role = PlatoonRole.LEADER
                new_leader._cf_mode = CF_Mode.ACC

                LOG.debug(f"{new_leader.vid} became leader of platoon {new_leader.platoon.platoon_id}")

                self.platoon._formation.remove(self)
                assert(self.platoon.size > 1)
                self.platoon.update_desired_speed()

                # update formation for all members
                for vehicle in self.platoon.formation:
                    vehicle._platoon = self.platoon
                    self._simulator._adjust_speed(vehicle)

            # leave
            LOG.info(f"{self.vid} left platoon {self.platoon.platoon_id}")
            self._platoon_role = PlatoonRole.NONE  # the current platoon role
            self._platoon = Platoon(self.vid, [self], self.desired_speed)

            self._cf_mode = CF_Mode.ACC  # not necessary, but we still do it explicitly
        elif self is self.platoon.last:
            # leave at back
            assert(self is not self.platoon.leader)
            # TODO check whether it is safe to leave
            # TODO tell the leader (who needs to tell all other vehicles)
            # TODO leave
            sys.exit("Leave from back of a platoon is not yet implemented!")
        else:
            # leave in the middle
            # TODO check wether is is safe to leave
            # TODO tell the leader (who needs to tell all other vehicles and needs to tell them to make space)
            # TODO leave
            sys.exit("Leave from the middle of a platoon is not yet implemented!")

        # reset color of vehicle
        if self._simulator._gui:
            import traci
            traci.vehicle.setColor(str(self.vid), self._color)

        self.in_maneuver = False

        assert(self._last_platoon_join_time >= 0)
        self._time_in_platoon = self.time_in_platoon + (self._simulator.step - self._last_platoon_join_time)
        assert(self._last_platoon_join_position >= 0)
        self._distance_in_platoon = self.distance_in_platoon + (self.position - self._last_platoon_join_position)

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
        LOG.info(f"{self.vid} received an advertisement from {advertisement.origin}")
