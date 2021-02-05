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

from .cf_model import CF_Model
from .message import Message, PlatoonAdvertisement
from .platoon import Platoon
from .platoon_role import PlatoonRole
from .speed_position import SpeedPosition
from .vehicle import Vehicle
from .vehicle_type import VehicleType

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .simulator import Simulator  # noqa 401

LOG = logging.getLogger(__name__)


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
            communication_range: int,
            acc_headway_time: float,
            cacc_spacing: float,
            formation_algorithm: str,
            execution_interval: int,
            alpha: float,
            speed_deviation_threshold: float,
            position_deviation_threshold: int):
        super().__init__(
            simulator,
            vid,
            vehicle_type,
            depart_position,
            arrival_position,
            desired_speed,
            depart_lane,
            depart_speed,
            depart_time,
            communication_range
        )

        self._cf_model = CF_Model.ACC
        self._acc_headway_time = acc_headway_time
        self._acc_lambda = 0.1  # see Eq. 6.18 of R. Rajamani, Vehicle Dynamics and Control, 2nd. Springer, 2012.
        self._cacc_spacing = cacc_spacing
        self._platoon_role = PlatoonRole.NONE  # the current platoon role
        self._platoon = Platoon(self.vid, [self], self.desired_speed)
        self._in_maneuver = False

        if formation_algorithm is not None:
            # initialize formation algorithm
            # TODO make enum
            if formation_algorithm == "speedposition":
                self._formation_algorithm = SpeedPosition(self, alpha, speed_deviation_threshold, position_deviation_threshold)
            else:
                sys.exit(f"ERROR: Unknown formation algorithm {formation_algorithm}!")
            self._execution_interval = execution_interval

            # initialize timers
            self._last_formation_step = self.depart_time  # initialize with vehicle start
            self._last_advertisement_step = None

        else:
            self._formation_algorithm = None

        # platoon statistics
        self._first_platoon_join_time = -1
        self._last_platoon_join_time = -1
        self._time_in_platoon = 0
        self._first_platoon_join_position = -1
        self._last_platoon_join_position = -1
        self._distance_in_platoon = 0
        self._number_platoons = 0

        # maneuver statistics
        self._joins_attempted = 0
        self._joins_succesful = 0
        self._joins_aborted = 0
        self._joins_aborted_front = 0
        self._joins_aborted_arbitrary = 0
        self._joins_aborted_road_begin = 0
        self._joins_aborted_leader_maneuver = 0
        self._joins_aborted_no_space = 0

        self._joins_front = 0
        self._joins_arbitrary = 0
        self._joins_back = 0
        self._joins_teleport_position = 0
        self._joins_teleport_lane = 0
        self._joins_teleport_speed = 0
        self._joins_correct_position = 0
        self._leaves_attempted = 0
        self._leaves_successful = 0
        self._leaves_aborted = 0
        self._leaves_front = 0
        self._leaves_arbitrary = 0
        self._leaves_back = 0

        # formation statistics
        self._candidates_found = 0
        self._candidates_filtered = 0
        self._candidates_filtered_maneuver = 0
        self._candidates_filtered_speed = 0
        self._candidates_filtered_position = 0
        self._candidates_filtered_front = 0

    @property
    def acc_headway_time(self) -> float:
        return self._acc_headway_time

    @property
    def desired_headway_time(self) -> float:
        if self.cf_model == CF_Model.ACC or self.cf_model == CF_Model.CACC:
            return self.acc_headway_time
        return super().desired_headway_time

    @property
    def desired_speed(self) -> float:
        # return desired speed of platoon if in platoon
        return self.platoon.desired_speed if self.is_in_platoon() else self._desired_speed

    @property
    def desired_gap(self) -> float:
        if self.cf_model == CF_Model.CACC:
            return self._cacc_spacing
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
            LOG.warning(f"{self.vid} is already in a meneuver")
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
        return -1.0 / self.desired_headway_time * (self.speed - desired_speed + self._acc_lambda * (-gap_to_predecessor + desired_gap))

    def new_speed(self, speed_predecessor: float, predecessor_rear_position: float) -> float:
        if self._cf_model is CF_Model.ACC:
            # TODO we should use different maximum accelerations/decelerations and headway times/gaps for different models
            if speed_predecessor >= 0 and predecessor_rear_position >= 0:
                gap_to_predecessor = predecessor_rear_position - self.position
                LOG.debug(f"{self.vid}'s front gap {gap_to_predecessor}")
                if gap_to_predecessor < 0:
                    LOG.warning(f"{self.vid}'s front gap is negative ({gap_to_predecessor})")
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

                # make sure we do not drive backwards
                if (new_speed < 0):
                    new_speed = 0

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
                    LOG.debug(f"{self.vid} is blocked by slow vehicle!")
                    self._blocked_front = True
                else:
                    self._blocked_front = False

                return new_speed

        elif self._cf_model is CF_Model.CACC:
            assert(self.is_in_platoon())
            assert(self.platoon_role is not PlatoonRole.LEADER)  # only followers can use CACC

            # sanity checks for front vehicle in platoon
            assert(speed_predecessor >= 0 and predecessor_rear_position >= 0)
            # check whether there is a vehicle between us and our front vehicle

            assert(self.platoon.get_front(self) is self._simulator._get_predecessor(self))

            gap_to_predecessor = predecessor_rear_position - self.position
            LOG.debug(f"{self.vid}'s front gap {gap_to_predecessor}")
            if gap_to_predecessor < 0:
                LOG.warning(f"{self.vid}'s front gap is negative ({gap_to_predecessor})")
            LOG.debug(f"{self.vid}'s desired gap {self.desired_gap}")
            LOG.debug(f"{self.vid}'s predecessor speed {speed_predecessor}")

            ### HACK FOR AVOIDING COMMUNICATION ###
            # acceleration_predecessor = self.platoon.get_front(self).acceleration
            # acceleration_leader = self.platoon.leader.acceleration
            speed_leader = self.platoon.leader.speed
            #######################################

            ### HACK FOR CACC ###
            # avoid to calculation
            if gap_to_predecessor == self._cacc_spacing and self.speed == speed_leader and self.speed == speed_predecessor:
                LOG.debug(f"{self.vid} does not need to calcucate a CACC new speed")
                return self.speed
            LOG.debug(f"{self.vid} needs to calculate a new CACC speed")
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

            # make sure we do not drive backwards
            if (new_speed < 0):
                new_speed = 0

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
        if (self.position < self.arrival_position):
            LOG.warning(f"{self.vid}'s finish method was called even though vehicle did not arrive yet!")
            return

        super().finish()

        # clean up platoon
        if self.is_in_platoon():
            self._leave()

        platoon_time_ratio = round(self.time_in_platoon / self.travel_time, 2)
        platoon_distance_ratio = round(self.distance_in_platoon / self.travel_distance, 2)

        LOG.info(f"{self.vid} drove {self.time_in_platoon}s ({self.distance_in_platoon}m) in a platoon, {platoon_time_ratio * 100}% ({platoon_distance_ratio * 100}%) of the trip")

        # statistic recording

        if not self._simulator._record_prefilled and self._depart_time == -1:
            # we do not record statistics for pre-filled vehicles
            return

        # TODO should we avoid logging if the mimimum trip length has not been fulfilled?

        time_until_first_platoon = self._first_platoon_join_time - self._depart_time  # NOTE: this produces wrong values when prefilled
        distance_until_first_platoon = self._first_platoon_join_position - self._depart_position  # NOTE: this produces wrong values when prefilled

        # TODO log savings from platoon?
        if self._simulator._record_platoon_trips:
            with open(f'{self._simulator._result_base_filename}_platoon_trips.csv', 'a') as f:
                f.write(
                    f"{self.vid},"
                    f"{self.time_in_platoon},"
                    f"{self.distance_in_platoon},"
                    f"{platoon_time_ratio},"
                    f"{platoon_distance_ratio},"
                    f"{self._number_platoons},"
                    f"{time_until_first_platoon},"
                    f"{distance_until_first_platoon}"
                    "\n"
                )

        if self._simulator._record_platoon_maneuvers:
            with open(f'{self._simulator._result_base_filename}_platoon_maneuvers.csv', 'a') as f:
                f.write(
                    f"{self.vid},"
                    f"{self._joins_attempted},"
                    f"{self._joins_succesful},"
                    f"{self._joins_aborted},"
                    f"{self._joins_aborted_front},"
                    f"{self._joins_aborted_arbitrary},"
                    f"{self._joins_aborted_road_begin},"
                    f"{self._joins_aborted_leader_maneuver},"
                    f"{self._joins_aborted_no_space},"
                    f"{self._joins_front},"
                    f"{self._joins_arbitrary},"
                    f"{self._joins_back},"
                    f"{self._joins_teleport_position},"
                    f"{self._joins_teleport_lane},"
                    f"{self._joins_teleport_speed},"
                    f"{self._joins_correct_position},"
                    f"{self._leaves_attempted},"
                    f"{self._leaves_successful},"
                    f"{self._leaves_aborted},"
                    f"{self._leaves_front},"
                    f"{self._leaves_arbitrary},"
                    f"{self._leaves_back}"
                    "\n"
                )

        if self._simulator._record_platoon_formation:
            with open(f'{self._simulator._result_base_filename}_platoon_formation.csv', 'a') as f:
                f.write(
                    f"{self.vid},"
                    f"{self._candidates_found},"
                    f"{self._candidates_filtered},"
                    f"{self._candidates_filtered_maneuver},"
                    f"{self._candidates_filtered_speed},"
                    f"{self._candidates_filtered_position},"
                    f"{self._candidates_filtered_front}"
                    "\n"
                )

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

        if not self._simulator._record_prefilled and self._depart_time == -1:
            # we do not record statistics for pre-filled vehicles
            return

        if self._simulator._record_platoon_traces:
            # write statistics about the current platoon
            with open(f'{self._simulator._result_base_filename}_platoon_traces.csv', 'a') as f:
                f.write(
                    f"{self._simulator.step},"
                    f"{self.vid},"
                    f"{self.platoon.platoon_id},"
                    f"{self.platoon.leader.vid},"
                    f"{self.platoon.position},"
                    f"{self.platoon.rear_position},"
                    f"{self.platoon.lane},"
                    f"{self.platoon.speed},"
                    f"{self.platoon.size},"
                    f"{self.platoon.length},"
                    f"{self.platoon.desired_speed},"
                    f"{self.platoon_role.name},"
                    f"{self.platoon.get_member_index(self)}"
                    "\n"
                )

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
        self._joins_attempted += 1

        self.in_maneuver = True

        leader = self._simulator._vehicles[leader_id]
        assert(isinstance(leader, PlatooningVehicle))

        # correct platoon
        assert(leader.platoon.platoon_id == platoon_id)
        # correct leader of that platoon
        assert(leader.vid == leader.platoon.leader.vid)

        # HACK for determining the join position
        if self.position > leader.platoon.position:
            # TODO join at front
            LOG.warning(f"{self.vid} is in front of the target platoon {platoon_id} ({leader_id})")
            self._joins_front += 1
            LOG.warning("Join at the front of a platoon is not yet implemented!")
            self.in_maneuver = False
            self._joins_aborted += 1
            self._joins_aborted_front += 1
            return
        elif self.position > leader.platoon.last.position:
            # TODO join at (arbitrary position) in the middle
            LOG.warning(f"{self.vid} is in front of (at least) the last vehicle {leader.platoon.last.vid} of the target platoon {platoon_id} ({leader_id})")
            self._joins_arbitrary += 1
            LOG.warning("Join at arbitrary positions of a platoon is not yet implemented!")
            self.in_maneuver = False
            self._joins_aborted += 1
            self._joins_aborted_arbitrary += 1
            return

        # join at back
        self._joins_back += 1

        last = leader.platoon.last
        new_position = last.rear_position - self._cacc_spacing

        if new_position - self.length < 0:
            # we cannot join since we would be outside of the road
            LOG.warning(f"{self.vid} is too close to the begining of the road!")
            self._joins_aborted += 1
            self._joins_aborted_road_begin += 1
            return

        if leader.in_maneuver:
            LOG.warning(f"{self.vid}'s new leader {leader_id} was already in a maneuver")
            self.in_maneuver = False
            self._joins_aborted += 1
            self._joins_aborted_leader_maneuver += 1
            return

        platoon_successor = self._simulator._get_successor(last)
        if platoon_successor is None:
            LOG.debug(f"{self.vid}'s new position is {new_position} (behind {last.vid})")
        else:
            LOG.debug(f"{self.vid}'s new position is {new_position} (between {last.vid} and {platoon_successor.vid})")

            # MAKE SPACE FOR THE JOINER
            # the idea is to move the vehicle(s) behind the platoon to make room for the joiner
            # we start with the first one (direct follower) and proceed with the next vehicle(s) as long as we need more space
            # thus, we move only as little vehicles and as little as necessary (until they reach the mingap to the front vehicle)
            # this way, it is more realistic and keeps the correct order of vehicles without producing an incorrect state during the process
            # we might also move the method to the vehicle class or make it even part of the simulator

            # how big needs the gap behind the platoon in front of the platoon successor to be
            # cacc spacing + joiner + successor's min gap
            required_gap = self._cacc_spacing + self.length + platoon_successor.min_gap
            LOG.debug(f"We need a gap of {required_gap}m behind the platoon (vehicle {last.vid}) to teleport vehicle {self.vid}")
            current_gap = last.rear_position - platoon_successor.position
            still_required_space = required_gap - current_gap
            LOG.debug(f"We currently have a gap of {current_gap} and thus still require {still_required_space}m")
            if last.rear_position - (required_gap + platoon_successor.length) < 0:
                # it is not possible to join because we cannot shift the current platoon successor out of the road
                LOG.warning(f"Could not make enough space to teleport vehicle {self.vid}!")
                self.in_maneuver = False
                self._joins_aborted += 1
                self._joins_aborted_no_space += 1
                return

            # list of vehicles that where moved already
            already_moved_vehicles = []
            # vehicle we are moving next
            current_vehicle = platoon_successor
            # we need to do act to make space
            while still_required_space > 0:
                assert(current_vehicle is not None)
                # move vehicle until min gap to its successor is reached
                LOG.debug(f"We are checking vehicle {current_vehicle.vid} for a possible move")
                # successor of the vehicle we are moving now
                current_successor = self._simulator._get_successor(current_vehicle)
                if current_successor is not None:
                    LOG.debug(f"The successor of vehicle {current_vehicle.vid} is {current_successor.vid}")
                    # gap between vehicle we move now and its successor
                    current_back_gap = current_vehicle.rear_position - current_successor.position
                    LOG.debug(f"{current_vehicle.vid}'s back gap is {current_back_gap}m")
                    # avaiable space we could utilize during the move
                    current_possible_space = current_back_gap - current_successor.min_gap
                    LOG.debug(f"We could (theoretically) gain {current_possible_space}m by moving vehicle {current_vehicle.vid}")
                else:
                    LOG.debug(f"Vehicle {current_vehicle.vid} has no successor")
                    # we have no successor
                    # thus, we can move as far as we like to
                    # but only until the end of the road
                    current_possible_space = current_vehicle.position - current_vehicle.length
                    LOG.debug(f"We could (theoretically) gain {current_possible_space}m by moving vehicle {current_vehicle.vid}")
                # space we are actually gaining during this move
                current_gained_space = min(still_required_space, current_possible_space)
                LOG.debug(f"We will gain {current_gained_space}m by moving vehicle {current_vehicle.vid}")
                # can we move of the current vehicle?
                if current_gained_space > 0:
                    # move the current vehicle
                    current_vehicle._position -= current_gained_space
                    LOG.debug(f"We moved vehicle {current_vehicle.vid} to {current_vehicle.position}")
                    # move the previous vehicle(s) as well to keep the correct spacings between them
                    for v in already_moved_vehicles:
                        v._position -= current_gained_space
                        LOG.debug(f"We moved vehicle {v.vid} to {v.position} as well")
                    # how much space do we still need?
                    still_required_space -= current_gained_space
                else:
                    LOG.warning(f"Could not make enough space to teleport vehicle {self.vid}!")
                    self.in_maneuver = False
                    self._joins_aborted += 1
                    self._joins_aborted_no_space += 1
                    return

                # we need to rember this vehicle
                already_moved_vehicles.append(current_vehicle)
                current_vehicle = current_successor
                LOG.debug(f"We still require {still_required_space}m")

        # teleport the vehicle
        current_position = self.position
        assert(new_position >= 0)
        if current_position != new_position:
            self._position = new_position
            LOG.info(f"{self.vid} teleported to {self.position} (from {current_position})")
            self._joins_teleport_position += 1
        current_lane = self.lane
        new_lane = leader.lane
        if current_lane != new_lane:
            self._lane = new_lane
            LOG.info(f"{self.vid} switched to lane {self.lane} (from {current_lane})")
            self._joins_teleport_lane += 1
        current_speed = self.speed
        new_speed = last.speed
        if current_speed != new_speed:
            self._speed = new_speed
            LOG.info(f"{self.vid} changed speed to {self.speed} (from {current_speed})")
            self._joins_teleport_speed += 1

        # update the leader
        leader.in_maneuver = True
        if not leader.is_in_platoon():
            # only if leader was alone before
            LOG.info(f"{leader_id} became a leader of platoon {leader.platoon.platoon_id}")
            leader._last_platoon_join_time = self._simulator.step
            leader._last_platoon_join_position = leader.position
            if leader._first_platoon_join_time == -1:
                leader._first_platoon_join_time = self._simulator.step
                assert(leader._first_platoon_join_position == -1)
                leader._first_platoon_join_position = self.position
            leader._number_platoons += 1
        leader._platoon_role = PlatoonRole.LEADER

        self._platoon_role = PlatoonRole.FOLLOWER

        # update all members
        leader.platoon._formation.append(self)
        # update the desired speed of the platoon to the average of all platoom members
        leader.platoon.update_desired_speed()

        # switch to CACC
        self._cf_model = CF_Model.CACC
        self._blocked_front = False

        for vehicle in leader.platoon.formation:
            # we copy all parameters from the platoon (for now)
            # thus, the follower now drives as fast as the already existing platoon (i.e., only the leader in the worst case)
            vehicle._platoon = leader.platoon
            self._simulator._adjust_speed(vehicle)  # FIXME duplicated execution of CACC

        # set color of vehicle
        if self._simulator._gui:
            import traci
            traci.vehicle.setColor(str(vehicle.vid), leader._color)

        LOG.info(f"{self.vid} joined platoon {platoon_id} (leader: {leader_id})")

        self.in_maneuver = False
        leader.in_maneuver = False

        # the last time we joined is now
        self._last_platoon_join_time = self._simulator.step
        self._last_platoon_join_position = self.position
        if self._first_platoon_join_time == -1:
            self._first_platoon_join_time = self._simulator.step
            assert(self._first_platoon_join_position == -1)
            self._first_platoon_join_position = self.position
        self._joins_succesful += 1
        self._number_platoons += 1

    def _leave(self):
        # just leave, without any communication

        if self.platoon.size == 1:
            LOG.warning(f"Cannot leave when driving indiviudally ({self.vid})")
            return
        assert(self.is_in_platoon())

        LOG.info(f"{self.vid} is trying to leave platoon {self.platoon.platoon_id} (leader {self.platoon.leader.vid})")
        self._leaves_attempted += 1

        self.in_maneuver = True

        if self is self.platoon.leader:
            # leave at front
            self._leaves_front += 1

            if self.platoon.size == 2:
                # tell the only follower to drive individually
                follower = self.platoon.last
                LOG.debug(f"Only {follower.vid} is left in the platoon {self.platoon.platoon_id}. Thus, we are going to destroy the entire platoon.")
                follower._platoon_role = PlatoonRole.NONE
                follower._cf_model = CF_Model.ACC
                follower._platoon = Platoon(follower.vid, [follower], follower.desired_speed)
                self._simulator._adjust_speed(follower)  # FIXME duplicated execution of CACC

                # reset color of vehicle
                if self._simulator._gui:
                    import traci
                    traci.vehicle.setColor(str(follower.vid), follower._color)

                # statistics
                follower._leaves_attempted += 1
                assert(follower._last_platoon_join_time >= 0)
                follower._time_in_platoon += follower._simulator.step - follower._last_platoon_join_time
                assert(follower._last_platoon_join_position >= 0)
                follower._distance_in_platoon += follower.position - follower._last_platoon_join_position
                follower._leaves_successful += 1
                follower._leaves_back += 1
            else:
                # tell the second vehicle in the platoon to become the new leader
                new_leader = self.platoon.formation[1]
                new_leader._platoon_role = PlatoonRole.LEADER
                new_leader._cf_model = CF_Model.ACC
                LOG.debug(f"{new_leader.vid} became leader of platoon {new_leader.platoon.platoon_id}")
        elif self is self.platoon.last:
            # leave at back
            LOG.warning("Leave from back of a platoon is not yet properly implemented!")
            self._leaves_back += 1

            # TODO check whether it is safe to leave

            if self.platoon.size == 2:
                # tell the current leader to drive individually
                leader = self.platoon.leader
                LOG.debug(f"Only the current leader {leader.vid} is left in the platoon {self.platoon.platoon_id}. Thus, we are going to destroy the entire platoon.")
                leader._platoon_role = PlatoonRole.NONE
                leader._cf_model = CF_Model.ACC
                leader._platoon = Platoon(leader.vid, [leader], leader.desired_speed)
                self._simulator._adjust_speed(leader)  # FIXME duplicated execution of CACC

                # reset color of vehicle
                if self._simulator._gui:
                    import traci
                    traci.vehicle.setColor(str(leader.vid), leader._color)

                # statistics
                leader._leaves_attempted += 1
                assert(leader._last_platoon_join_time >= 0)
                leader._time_in_platoon += leader._simulator.step - leader._last_platoon_join_time
                assert(leader._last_platoon_join_position >= 0)
                leader._distance_in_platoon += leader.position - leader._last_platoon_join_position
                leader._leaves_successful += 1
                leader._leaves_front += 1
        else:
            # leave in the middle
            LOG.warning("Leave from the middle of a platoon is not yet properly implemented!")
            self._leaves_arbitrary += 1

            # TODO check wether is is safe to leave
            # TODO the leader needs to all other vehicles to make space

        # leave the platoon
        self.platoon._formation.remove(self)
        self.platoon.update_desired_speed()

        # update formation for all members
        for vehicle in self.platoon.formation:
            vehicle._platoon = self.platoon  # TODO superfluous?
            self._simulator._adjust_speed(vehicle)  # FIXME duplicated execution of CACC

        # leave
        LOG.info(f"{self.vid} left platoon {self.platoon.platoon_id} (leader {self.platoon.leader.vid})")
        self._platoon_role = PlatoonRole.NONE  # the current platoon role
        self._platoon = Platoon(self.vid, [self], self.desired_speed)
        self._cf_model = CF_Model.ACC  # not necessary, but we still do it explicitly

        # reset color of vehicle
        if self._simulator._gui:
            import traci
            traci.vehicle.setColor(str(self.vid), self._color)

        self.in_maneuver = False

        assert(self._last_platoon_join_time >= 0)
        self._time_in_platoon += self._simulator.step - self._last_platoon_join_time
        assert(self._last_platoon_join_position >= 0)
        self._distance_in_platoon += self.position - self._last_platoon_join_position
        self._leaves_successful += 1

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
