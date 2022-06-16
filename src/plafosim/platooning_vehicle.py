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
import math
import sys
from typing import TYPE_CHECKING

from .algorithms.speed_position import SpeedPosition
from .gui import change_gui_vehicle_color
from .message import Message, PlatoonAdvertisement
from .mobility import CF_Model
from .platoon import Platoon
from .platoon_role import PlatoonRole
from .statistics import (
    record_platoon_formation,
    record_platoon_trace,
    record_platoon_trip,
    record_vehicle_platoon_maneuvers,
    record_vehicle_platoon_trace,
)
from .util import round_to_next_base
from .vehicle import Vehicle
from .vehicle_type import VehicleType

if TYPE_CHECKING:
    from .simulator import Simulator  # noqa 401

LOG = logging.getLogger(__name__)


class PlatooningVehicle(Vehicle):
    """A vehicle that has platooning functionality."""

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
            depart_delay: int = 0,
            communication_range: int = 1000,  # TODO use defaults
            acc_headway_time: float = 1.0,  # TODO use defaults
            cacc_spacing: float = 5.0,  # TODO use defaults
            formation_algorithm: str = None,
            execution_interval: int = 1,  # TODO use defaults
            pre_filled: bool = False,
            **kw_args,
    ):
        # TODO use global default values
        """
        Initializes a platooning vehicle instance.

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
        acc_headway_time: float
            The headway time for the ACC
        cacc_spacing: float
            The constant spacing for the CACC
        formation_algorithm: str
            The platoon formation algorithm to use
        execution_interval: int
            The interval for executing the formation algorithm
        pre_filled : bool
            Whether this vehicle was pre-filled
        """

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
            depart_delay,
            communication_range,
            pre_filled,
        )

        self._cf_model = CF_Model.ACC
        self._acc_headway_time = acc_headway_time
        self._acc_lambda = 0.1  # see Eq. 6.18 of R. Rajamani, Vehicle Dynamics and Control, 2nd. Springer, 2012.
        self._cacc_spacing = cacc_spacing
        self._platoon_role = PlatoonRole.NONE  # the current platoon role
        self._platoon = Platoon(vid, [self], desired_speed)
        self._in_maneuver = False
        # for a JOINER
        self._join_approach_step = None
        self._join_data_leader = None
        self._join_data_last = None
        self._join_data_new_position = None
        # for a LEADER
        self._joiner = None

        if formation_algorithm:
            # initialize formation algorithm
            # TODO make enum
            if formation_algorithm == SpeedPosition.__name__:
                self._formation_algorithm = SpeedPosition(self, **kw_args)
            else:
                sys.exit(f"ERROR: Unknown formation algorithm {formation_algorithm}!")
            self._execution_interval = execution_interval

            # initialize timers
            if pre_filled:
                self._last_formation_step = simulator._rng.randint(0, self._execution_interval - 1)
            else:
                self._last_formation_step = self._depart_time  # initialize with vehicle start
            self._last_advertisement_step = None

        else:
            self._formation_algorithm = None

        ## statistics

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
        self._joins_aborted_trip_begin = 0
        self._joins_aborted_road_end = 0
        self._joins_aborted_trip_end = 0
        self._joins_aborted_leader_maneuver = 0
        self._joins_aborted_max_speed = 0
        self._joins_aborted_teleport_threshold = 0
        self._joins_aborted_approaching = 0
        self._joins_aborted_no_space = 0
        self._joins_aborted_leave_other = 0

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
        self._formation_iterations = 0
        self._candidates_found = 0
        self._candidates_filtered = 0
        self._candidates_filtered_follower = 0
        self._candidates_filtered_maneuver = 0

    @property
    def acc_headway_time(self) -> float:
        """Returns the ACC headway time of the vehicle."""

        return self._acc_headway_time

    @property
    def desired_headway_time(self) -> float:
        """
        Returns the desired headway time of the vehicle.

        This is based on the currently active car following model.
        """

        if self._cf_model == CF_Model.ACC:
            return self._acc_headway_time
        elif self._cf_model == CF_Model.CACC:
            return -1
        return super().desired_headway_time

    @property
    def desired_speed(self) -> float:
        """
        Returns the desired driving speed of the vehicle.

        If the vehicle is in a platoon, it returns the desired driving speed of the entire platoon.
        """

        return self._platoon.desired_speed if self.is_in_platoon() else self._desired_speed

    @property
    def desired_gap(self) -> float:
        """
        Returns the desired gap of the vehicle.

        This is based on the currently active car following model.
        """
        if self._cf_model == CF_Model.CACC:
            return self._cacc_spacing
        return super().desired_gap

    @property
    def platoon_role(self) -> PlatoonRole:
        """Returns the current platoon role of the vehicle."""

        return self._platoon_role

    @property
    def platoon(self) -> Platoon:
        """Returns the platoon of the vehicle."""

        return self._platoon

    def is_in_platoon(self) -> bool:
        """
        Returns whether the vehicle currently is in a platoon.

        This is based on the current PlatoonRole.
        A joining or leaving vehicle is either not yet or still part of a platoon, thus the returned value should be true.
        """

        return self._platoon_role is not PlatoonRole.NONE

    def get_front_gap(self) -> float:
        """
        Returns the gap to the vehicle in the front.

        This imitates a measurement of the front radar sensor.
        """

        return self._simulator._get_predecessor_rear_position(self) - self._position

    def get_front_speed(self) -> float:
        """
        Returns the speed to the vehicle in the front.

        This imitates a measurement of the front radar sensor.
        """

        return self._simulator._get_predecessor_speed(self)

    @property
    def in_maneuver(self) -> bool:
        """Returns whether the vehicle is currently in a maneuver."""

        return self._in_maneuver

    @in_maneuver.setter
    def in_maneuver(self, var: bool):
        """
        Sets the maneuver status of the vehicle.

        Parameters
        ----------
        var : bool
            The maneuver status
        """

        assert var != self._in_maneuver, f"Maneuver request {var} does not match maneuver status {self._in_maneuver} of {self._vid}!"
        self._in_maneuver = var

    @property
    def time_in_platoon(self) -> int:
        """Returns the travelled time within platoons."""

        return self._time_in_platoon

    @property
    def distance_in_platoon(self) -> float:
        """Returns the travelled distance within platoons."""

        return self._distance_in_platoon

    @property
    def color(self) -> tuple:
        """Return the current color of the vehicle."""

        return self._platoon.leader._color if self.is_in_platoon() else self._color

    def _calculate_emission(self, a: float, v: float, f: list, scale: float) -> float:
        """
        Calculates the emitted pollutant amount using the given speed and acceleration.

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

        # calculate impact of platooning on air drag based on
        # Charles-Henri Bruneau, Khodor Khadra and Iraj Mortazavi,
        # "Flow analysis of square-back simplified vehicles in platoon,"
        # International Journal of Heat and Fluid Flow, vol. 66, pp. 43–59,
        # August 2017. Table 5: d = L, vehicle length = 4m, distance = 5m
        air_drag_change = 0.0
        if self._platoon_role == PlatoonRole.LEADER:
            # savings by followers
            assert self._platoon.size > 1
            air_drag_change = 0.12
        elif self._platoon_role == PlatoonRole.FOLLOWER:
            # savings by leader/front vehicles
            if self is self._platoon.last:
                # last vehicle
                air_drag_change = 0.23
            else:
                # in between
                air_drag_change = 0.27

        # calculate impact of air drag on emissions based on
        # Gino Sovran, "Tractive-Energy-Based Formulae for the Impact of
        # Aerodynamics on Fuel Economy Over the EPA Driving Schedules,"
        # SAE International, Technical Paper, 830304, February 1983.
        if self._simulator._reduced_air_drag:
            emission_change = air_drag_change * 0.46
        else:
            emission_change = 0.0

        return super()._calculate_emission(a, v, f, scale) * (1.0 - emission_change)

    def finish(self):
        """
        Cleans up the instance of the PlatooningVehicle.

        This includes leaving the platoon and mostly statistics recording.
        """

        if (self._position < self._arrival_position):
            LOG.warning(f"{self._vid}'s finish method was called even though vehicle did not arrive yet!")
            return

        super().finish()

        # clean up platoon
        if self.is_in_platoon():
            self._leave()

        assert self.travel_time > 0
        assert self._time_in_platoon >= 0
        platoon_time_ratio = self._time_in_platoon / self.travel_time
        assert self.travel_distance > 0
        assert self._distance_in_platoon >= 0
        platoon_distance_ratio = self._distance_in_platoon / self.travel_distance

        LOG.info(f"{self._vid} drove {self._time_in_platoon}s ({self._distance_in_platoon}m) in a platoon, {platoon_time_ratio * 100}% ({platoon_distance_ratio * 100}%) of the trip")

        # statistic recording

        # TODO use pre_filled flag
        if not self._simulator._record_prefilled and self._depart_time == -1:
            # we do not record statistics for pre-filled vehicles
            LOG.debug(f"Not recording statistics for pre-filled vehicle {self._vid}")
            return

        candidates_found_avg = (
            self._candidates_found / self._formation_iterations
            if self._formation_iterations > 0
            else 0
        )
        candidates_filtered_avg = (
            self._candidates_filtered / self._formation_iterations
            if self._formation_iterations > 0
            else 0
        )

        assert platoon_time_ratio >= 0
        assert platoon_distance_ratio >= 0

        if self._first_platoon_join_time == -1:
            # was not set yet (no platoon)
            assert self._first_platoon_join_position == -1
            time_until_first_platoon = None
            distance_until_first_platoon = None
        else:
            # NOTE: this produces wrong values when pre-filled
            time_until_first_platoon = self._first_platoon_join_time - self._depart_time
            assert time_until_first_platoon >= 0
            assert self._first_platoon_join_position != -1
            # NOTE: this produces wrong values when pre-filled
            distance_until_first_platoon = self._first_platoon_join_position - self._depart_position
            assert distance_until_first_platoon >= 0

        if self._simulator._record_platoon_trips:
            record_platoon_trip(
                basename=self._simulator._result_base_filename,
                vehicle=self,
                platoon_time_ratio=platoon_time_ratio,
                platoon_distance_ratio=platoon_distance_ratio,
                time_until_first_platoon=time_until_first_platoon,
                distance_until_first_platoon=distance_until_first_platoon,
            )

        if self._simulator._record_platoon_maneuvers:
            record_vehicle_platoon_maneuvers(basename=self._simulator._result_base_filename, vehicle=self)

        if self._simulator._record_platoon_formation:
            record_platoon_formation(
                basename=self._simulator._result_base_filename,
                vehicle=self,
                candidates_found_avg=candidates_found_avg,
                candidates_filtered_avg=candidates_filtered_avg,
            )

    def _action(self, step: int):
        """
        Triggers specific actions of a PlatooningVehicle.

        Parameters
        ----------
        step : int
            The current simulation step
        """

        super()._action(step)

        # continue jojn maneuver (approaching the target platoon)
        if self._join_approach_step:
            assert self._in_maneuver
            if step >= self._join_approach_step:
                self._join_teleport(self._join_data_leader, self._join_data_last, self._join_data_new_position)
                # cleaning up
                self._join_approach_step = None
                self._join_data_leader._joiner = None
                self._join_data_leader = None
                self._join_data_last = None
                self._join_data_new_position = None

        # do platoon formation
        if self._formation_algorithm:
            # transmit regular platoon advertisements
            self._advertise()

            # execute formation algorithm at every execution interval
            if step >= self._last_formation_step + self._execution_interval:
                # search for a platoon (depending on the algorithm)
                self._formation_algorithm.do_formation()
                self._last_formation_step = step

    def info(self) -> str:
        """Returns information about the PlatooningVehicle."""

        return f"{super().info()}, platoon {self._platoon}"

    def __str__(self) -> str:
        """Returns the str representation of the platooning vehicle."""

        self_dict = self.__dict__.copy()
        self_dict.update({'_vehicle_type': str(self._vehicle_type)})  # use str representation of vehicle type
        self_dict.update({'_platoon': str(self._platoon)})  # use str representation of platoon
        return str(self_dict)

    def _statistics(self):
        """Writes continuous statistics."""

        super()._statistics()

        if not self._simulator._record_prefilled and self._depart_time == -1:
            # we do not record statistics for pre-filled vehicles
            return

        if self._simulator._record_platoon_traces:
            # write statistics about the current platoon
            record_vehicle_platoon_trace(
                basename=self._simulator._result_base_filename,
                step=self._simulator.step,
                vehicle=self
            )

            if self._platoon_role is PlatoonRole.LEADER:
                # write statistics about the platoon
                record_platoon_trace(
                    basename=self._simulator._result_base_filename,
                    step=self._simulator.step,
                    vehicle=self
                )

    # TODO rework to only include "neighbors" and move platoon extraction to formation algorithm
    def _get_available_platoons(self):
        """
        Returns the available platoon candidates of the vehicle.

        This imitates neighbor maintenance by using a neighbor table.
        """

        # HACK FOR AVOIDING MAINTAINING A NEIGHBOR TABLE (for now)
        platoons = []
        for vehicle in self._simulator._vehicles.values():

            # filter own vehicle
            if vehicle is self:
                continue

            # filter non-available vehicles which are technically not able to do platooning
            if not isinstance(vehicle, PlatooningVehicle):
                continue

            # filter non-platoons
            if vehicle.platoon_role != PlatoonRole.LEADER and vehicle.platoon_role != PlatoonRole.NONE:
                continue

            # filter non-available vehicles based on communication range
            if abs(vehicle.position - self._position) > self._communication_range:
                LOG.trace(f"{self._vid}'s neighbor {vehicle.vid} is out of communication range ({self._communication_range}m)")
                continue

            platoons.append(vehicle.platoon)

        return platoons

    def _join(self, platoon_id: int, leader_id: int):
        """
        Lets a vehicle join a platoon.

        Communication and fine-grained maneuver control is out-of-scope and thus omitted.

        Parameters
        ----------
        platoon_id : int
            The id of the target platoon
        leader_id : int
            The id of the leader of the target platoon
        """

        assert not self.is_in_platoon()

        LOG.trace(f"{self._vid} is trying to join platoon {platoon_id} (leader {leader_id})")
        self._joins_attempted += 1

        assert not self.in_maneuver
        self.in_maneuver = True
        self._platoon_role = PlatoonRole.JOINER

        leader = self._simulator._vehicles[leader_id]
        assert isinstance(leader, PlatooningVehicle)

        # correct platoon
        assert leader.platoon.platoon_id == platoon_id
        # correct leader of that platoon
        assert leader.vid == leader.platoon.leader.vid

        if leader.in_maneuver:
            LOG.warning(f"{self._vid}'s new leader {leader_id} was already in a maneuver! Aborting the join maneuver!")
            self.in_maneuver = False
            self._platoon_role = PlatoonRole.NONE

            self._joins_aborted += 1
            self._joins_aborted_leader_maneuver += 1
            return

        # HACK for determining the join position
        if self._position > leader.platoon.position:
            # TODO join at front
            LOG.debug(f"{self._vid} is in front of the target platoon {platoon_id} ({leader_id})")
            LOG.warning("Join at the front of a platoon is not yet implemented! Aborting the join maneuver!")
            self.in_maneuver = False
            self._platoon_role = PlatoonRole.NONE

            self._joins_front += 1
            self._joins_aborted += 1
            self._joins_aborted_front += 1
            return
        elif self._position > leader.platoon.last.position:
            # TODO join at (arbitrary position) in the middle
            LOG.debug(f"{self._vid} is in front of (at least) the last vehicle {leader.platoon.last.vid} of the target platoon {platoon_id} ({leader_id})")
            LOG.warning("Join at arbitrary positions of a platoon is not yet implemented! Aborting the join maneuver!")
            self.in_maneuver = False
            self._platoon_role = PlatoonRole.NONE

            self._joins_arbitrary += 1
            self._joins_aborted += 1
            self._joins_aborted_arbitrary += 1
            return

        # join at back
        self._joins_back += 1

        last = leader.platoon.last
        new_position = last.rear_position - self._cacc_spacing
        LOG.trace(f"{self._vid}'s new position is ({new_position},{leader.platoon.lane}) (current {self._position},{self._lane})")

        if new_position < self.length:
            # we cannot join since we would be outside of the road
            LOG.warning(f"{self._vid}'s new position would be too close to the beginning of the road! Aborting the join maneuver!")
            self.in_maneuver = False
            self._platoon_role = PlatoonRole.NONE

            self._joins_aborted += 1
            self._joins_aborted_road_begin += 1
            return

        if new_position < self._depart_position:
            LOG.warning(f"{self._vid}'s new position would be before its depart position! Aborting the join maneuver!")
            self.in_maneuver = False
            self._platoon_role = PlatoonRole.NONE

            self._joins_aborted += 1
            self._joins_aborted_trip_begin += 1
            return
        # TODO should we also avoid teleporting backwards at all?
        # for now this is allowed to simulate decelerating and waiting for the platoon for passing further

        if new_position >= self._simulator.road_length:
            LOG.warning(f"{self._vid}'s new position would be outside of the road! Aborting the join maneuver")
            self.in_maneuver = False
            self._platoon_role = PlatoonRole.NONE

            self._joins_aborted += 1
            self._joins_aborted_road_end += 1
            return

        if new_position >= self._arrival_position:
            LOG.warning(f"{self._vid}'s new position would be outside of its trip! Aborting the join maneuver")
            self.in_maneuver = False
            self._platoon_role = PlatoonRole.NONE

            self._joins_aborted += 1
            self._joins_aborted_trip_end += 1
            return

        if self.max_speed <= leader.platoon.speed:
            # we will never be able to approach the platoon
            LOG.warning(f"{self._vid}'s maximum speed is too low such that it can never reach the platoon! Aborting the join maneuver!")
            self.in_maneuver = False
            self._platoon_role = PlatoonRole.NONE

            self._joins_aborted += 1
            self._joins_aborted_max_speed += 1
            return

        if abs(new_position - self._position) > self._simulator._maximum_teleport_distance:
            # the vehicle is too far away to be teleported
            LOG.warning(f"{self._vid} is too far away from the target platoon to realistically do a teleport! Aborting the join maneuver!")
            self.in_maneuver = False
            self._platoon_role = PlatoonRole.NONE

            self._joins_aborted += 1
            self._joins_aborted_teleport_threshold += 1
            return

        assert new_position >= self.length
        assert new_position <= self._simulator.road_length

        # consider the actual approaching duration
        total_approach_time = self.calculate_approaching_time(new_position, leader.platoon.speed)

        assert total_approach_time >= 0
        if total_approach_time > self._simulator._maximum_appraoch_time:
            # approaching the platoon would take too long
            LOG.warning(f"It would take too long ({total_approach_time}s) for {self._vid} to approach the platoon {leader.platoon.platoon_id} ({leader.vid})! Aborting the join maneuver!")
            self.in_maneuver = False
            self._platoon_role = PlatoonRole.NONE

            self._joins_aborted += 1
            self._joins_aborted_approaching += 1
            return

        # the join has been "allowed" by the leader
        # the actual join procedure starts here
        assert not leader.in_maneuver
        leader.in_maneuver = True
        assert leader.cf_model == CF_Model.ACC

        # delay teleport by approach duration
        if self._simulator._delay_teleports and total_approach_time > 0:
            # round approach time to next simulation step
            total_approach_time = round_to_next_base(total_approach_time, self._simulator.step_length)
            # the platoon will be driving while we are waiting
            new_position += total_approach_time * leader.platoon.speed
            if new_position >= self._simulator.road_length:
                LOG.warning(f"{self._vid}'s new position would be outside of the road! Aborting the join maneuver")
                self.in_maneuver = False
                self._platoon_role = PlatoonRole.NONE
                leader.in_maneuver = False

                self._joins_aborted += 1
                self._joins_aborted_road_end += 1
                return

            # check if the new position would be outside of the trip
            # NOTE: we need to add the distance the vehicle drives within one step as well in order to avoid that the vehicles finishes within the next step after the maneuver
            if new_position + self._simulator.step_length * self.speed >= self._arrival_position:
                LOG.warning(f"{self._vid}'s new position would be outside of its trip! Aborting the join maneuver")
                self.in_maneuver = False
                self._platoon_role = PlatoonRole.NONE
                leader.in_maneuver = False

                self._joins_aborted += 1
                self._joins_aborted_trip_end += 1
                return

            self._join_approach_step = self._simulator.step + total_approach_time
            self._join_data_leader = leader
            self._join_data_last = last
            self._join_data_new_position = new_position
            leader._joiner = self
            LOG.trace(f"Scheduled the teleport of vehicle {self._vid} to step {self._join_approach_step} (new position: {new_position})")
        else:
            # perform the teleport now
            LOG.trace(f"The teleport for vehicle {self._vid} will be performed instantaneous.")
            self._join_teleport(leader, last, new_position)

    def _join_teleport(self, leader: 'PlatooningVehicle', last: 'PlatooningVehicle', new_position: float):
        """
        Parameters
        ----------
        leader : PlatooningVehicle
            The leader of the target platoon
        last : PlatooningVehicle
            The last vehicle of the target platoon
        new_position : float
            The new position for joining the target platoon
        """

        LOG.trace(f"Continuing the join maneuver for vehicle {self._vid} -> {leader.platoon.platoon_id} ({leader.vid})")

        if leader.position >= leader.arrival_position:
            # the leader is gone (or will be gone soon)
            # TODO we might still want to teleport the vehicle for realism
            LOG.warning(f"{self._vid}'s platoon leader meanwhile reached the end of its trip! Aborting the join maneuver")
            self.in_maneuver = False
            self._platoon_role = PlatoonRole.NONE
            leader.in_maneuver = False

            self._joins_aborted += 1
            self._joins_aborted_trip_end += 1
            return

        assert self._in_maneuver
        assert self._platoon_role is PlatoonRole.JOINER
        assert leader.in_maneuver
        assert (leader.platoon_role is PlatoonRole.LEADER or leader.platoon_role is PlatoonRole.NONE)

        # re-calculate new position
        new_position = last.rear_position - self._cacc_spacing
        LOG.trace(f"{self._vid}'s new position is ({new_position},{leader.platoon.lane}) (current {self._position},{self._lane})")

        assert new_position >= self.length
        assert new_position >= self._depart_position
        assert new_position <= self._simulator.road_length
        assert new_position <= self._arrival_position

        platoon_successor = self._simulator._get_successor(last)
        if not platoon_successor or platoon_successor is self:
            LOG.trace(f"{self._vid}'s new position is {new_position} (behind {last.vid})")
        else:
            LOG.trace(f"{self._vid}'s new position is {new_position} (between {last.vid} and {platoon_successor.vid})")

            # MAKE SPACE FOR THE JOINER
            # the idea is to move the vehicle(s) behind the platoon to make room for the joiner
            # we start with the first one (direct follower) and proceed with the next vehicle(s) as long as we need more space
            # thus, we move only as little vehicles and as little as necessary (until they reach the mingap to the front vehicle)
            # this way, it is more realistic and keeps the correct order of vehicles without producing an incorrect state during the process
            # we might also move the method to the vehicle class or make it even part of the simulator

            # how big needs the gap behind the platoon in front of the platoon successor to be
            # cacc spacing + joiner + successor's min gap
            required_gap = self._cacc_spacing + self.length + platoon_successor.min_gap
            LOG.trace(f"We need a gap of {required_gap}m behind the platoon (vehicle {last.vid}) to teleport vehicle {self._vid}")
            current_gap = last.rear_position - platoon_successor.position
            still_required_space = required_gap - current_gap
            LOG.trace(f"We currently have a gap of {current_gap} and thus still require {still_required_space}m")
            if last.rear_position - (required_gap + platoon_successor.length) < 0:
                # it is not possible to join because we cannot shift the current platoon successor out of the road
                LOG.warning(f"Could not make enough space to teleport vehicle {self._vid}!")
                self.in_maneuver = False
                self._platoon_role = PlatoonRole.NONE
                leader.in_maneuver = False
                self._joins_aborted += 1
                self._joins_aborted_no_space += 1
                return

            # list of vehicles that where moved already
            already_moved_vehicles = []
            # vehicle we are moving next
            current_vehicle = platoon_successor
            # we need to act to make space
            while still_required_space > 0 and current_vehicle:
                # move vehicle until min gap to its successor is reached
                LOG.trace(f"We are checking vehicle {current_vehicle.vid} for a possible move")
                # successor of the vehicle we are moving now
                current_successor = self._simulator._get_successor(current_vehicle)
                if current_vehicle is self:
                    # we cannot move the current vehicle as we are going to teleport it soon
                    # we need to select the next successor
                    LOG.trace(f"We cannot move {current_vehicle.vid} as this is the joiner!")
                    current_vehicle = current_successor
                    continue
                if current_successor is self:
                    # we cannot consider the joiner as successor, thus we need to select the next successor
                    LOG.trace("We are skipping the joiner as successor")
                    current_successor = self._simulator._get_successor(self)
                if current_successor:
                    LOG.trace(f"The successor of vehicle {current_vehicle.vid} is {current_successor.vid}")
                    # gap between vehicle we move now and its successor
                    current_back_gap = current_vehicle.rear_position - current_successor.position
                    LOG.trace(f"{current_vehicle.vid}'s back gap is {current_back_gap}m")
                    # available space we could gain during the move of the current vehicle closer to its successor (deceleration)
                    current_possible_space = current_back_gap - current_successor.min_gap
                    LOG.trace(f"We could (theoretically) gain {current_possible_space}m by moving vehicle {current_vehicle.vid}")
                else:
                    LOG.trace(f"Vehicle {current_vehicle.vid} has no successor")
                    # we have no successor
                    # thus, we can move as far as we like to
                    # but only until the end of the road
                    current_possible_space = current_vehicle.position - current_vehicle.length
                    LOG.trace(f"We could (theoretically) gain {current_possible_space}m by moving vehicle {current_vehicle.vid}")
                # space we are actually gaining during this move
                current_gained_space = min(still_required_space, current_possible_space)
                LOG.trace(f"We will gain {current_gained_space}m by moving vehicle {current_vehicle.vid}")
                # the actual gain by moving the current vehicle
                if current_gained_space > 0:
                    # we do gain
                    # move the current vehicle
                    current_vehicle._position -= current_gained_space
                    LOG.trace(f"We moved vehicle {current_vehicle.vid} to {current_vehicle.position}-{current_vehicle.rear_position} and gained {current_gained_space}m")
                    # move the previous vehicle(s) as well to keep the correct spacings between them
                    for v in already_moved_vehicles:
                        v._position -= current_gained_space
                        LOG.trace(f"We moved vehicle {v.vid} by the same distance to {v.position}-{v.rear_position}")
                    # how much space do we still need?
                    still_required_space -= current_gained_space
                else:
                    # we do not move the current vehicle and check again with its follower
                    LOG.trace(f"We can not move vehicle {current_vehicle.vid}")

                # we need to remember this vehicle
                already_moved_vehicles.append(current_vehicle)
                current_vehicle = current_successor
                LOG.trace(f"We still require {still_required_space}m")

            if still_required_space > 0:
                # we still require some space for the teleport but do not have any more vehicles to move
                # we need to abort the maneuver
                # unfortunately, we move some vehicles already but this is collateral damage?
                # NOTE: will this produce a collision as we did not move the joiner?
                LOG.warning(f"Could not make enough space to teleport vehicle {self._vid}! Aborting the join maneuver!")
                self.in_maneuver = False
                self._platoon_role = PlatoonRole.NONE
                leader.in_maneuver = False
                self._joins_aborted += 1
                self._joins_aborted_no_space += 1
                return

        # teleport the vehicle
        self._teleport(new_position, leader.lane, last.speed)

        # update the leader
        if not leader.is_in_platoon():
            # only if leader was alone before
            LOG.debug(f"{leader.vid} became a leader of platoon {leader.platoon.platoon_id}")
            leader._last_platoon_join_time = self._simulator.step
            leader._last_platoon_join_position = leader.position
            if leader._first_platoon_join_time == -1:
                # was not set before
                leader._first_platoon_join_time = self._simulator.step
                assert leader._first_platoon_join_position == -1
                # was not set before
                leader._first_platoon_join_position = leader.position
            leader._number_platoons += 1
            leader._platoon_role = PlatoonRole.LEADER
        else:
            assert leader.platoon_role is PlatoonRole.LEADER

        # update self
        self._platoon_role = PlatoonRole.FOLLOWER
        # switch to CACC
        self._cf_model = CF_Model.CACC
        self._blocked_front = False
        # set color of vehicle
        if self._simulator._gui and self._simulator.step >= self._simulator._gui_start:
            assert self._color == self.platoon.leader._color == self.color
            change_gui_vehicle_color(self._vid, leader._color)

        # collect statistics
        # the last time we joined is now
        self._last_platoon_join_time = self._simulator.step
        self._last_platoon_join_position = self._position
        if self._first_platoon_join_time == -1:
            # was not set before
            self._first_platoon_join_time = self._simulator.step
            assert self._first_platoon_join_position == -1
            # was not set before
            self._first_platoon_join_position = self._position
        self._joins_succesful += 1
        self._number_platoons += 1

        # update all members
        leader.platoon._formation.append(self)
        # update the desired speed of the platoon to the average of all platoon members
        if self._simulator._update_desired_speed:
            leader.platoon.update_desired_speed()
        leader.platoon.update_limits()
        # update formation for all members
        for vehicle in leader.platoon.formation:
            # we copy all parameters from the platoon (for now)
            # thus, the follower now drives as fast as the already existing platoon
            # (i.e., only the leader in the worst case)
            vehicle._platoon = leader.platoon
            vehicle._cf_target_speed = vehicle._platoon.desired_speed

        LOG.debug(f"{self._vid} joined platoon {leader.platoon.platoon_id} (leader: {leader.vid})")

        self.in_maneuver = False
        leader.in_maneuver = False

    def calculate_approaching_time(self, target_position: float, target_speed: float) -> float:
        """
        Calculate approximate time to approach the target position at target speed

        Parameters
        ----------

        target_position : float
            The target position to approach
        target_speed : float
            The target speed after approaching
        """

        initial_distance = target_position - self._position
        total_approach_time = -1
        if initial_distance > 0:
            # we need to approach the platoon
            if self._speed <= target_speed:
                # we need to accelerate to approach the platoon
                self._cf_target_speed = self.max_speed
                # the time we need to accelerate to our maximum speed (given a linear acceleration)
                time_acceleration = (self.max_speed - self._speed) / self.max_acceleration
                # the time we need to decelerate to target speed (given a linear deceleration)
                time_deceleration = (self.max_speed - target_speed) / self.max_deceleration
                # the distance driven while accelerating to the maximum speed (given a linear acceleration
                distance_acceleration = (self._speed + self.max_speed) / 2 * time_acceleration
                # the distance driven while decelerating to the target speed (given a linear acceleration
                distance_deceleration = (self.max_speed + target_speed) / 2 * time_deceleration
                # the distance to drive with maximum speed (i.e., after acceleration and before deceleration)
                distance_max_speed = initial_distance - (distance_acceleration + distance_deceleration)
                # the time we need drive with the maximum speed
                time_max_speed = distance_max_speed / self.max_speed
                # the total time we need for approaching our target position
                total_approach_time = time_acceleration + time_max_speed + time_deceleration
            else:
                # we need to decelerate to approach the platoon
                # the time we need to decelerate to target speed (given a linear deceleration)
                time_deceleration = (self._speed - target_speed) / self.max_deceleration
                # the distance driven while decelerating to the target speed (given a linear acceleration
                distance_deceleration = (self._speed + target_speed) / 2 * time_deceleration
                # the distance to drive with our current speed (i.e., before deceleration)
                distance_current_speed = initial_distance - distance_deceleration
                # the time we need drive with the current speed
                time_current_speed = distance_current_speed / self._speed
                # the total time we need for approaching our target position
                total_approach_time = time_current_speed + time_deceleration
        else:
            # we do not need to consider this case as our error is only between 0m and last.length + cacc_spacing (e.g., 9m)
            assert abs(initial_distance) <= self.length + self._cacc_spacing  # FIXME use length of last vehicle in platoon
            total_approach_time = 0  # FIXME: we ignore that for now, since the time should be very small anyhow

        return total_approach_time

    def _teleport(self, new_position: float, new_lane: int, new_speed: float):
        """
        Teleport a vehicle to a given position.

        Parameters
        ----------
        new_position : float
            The new position
        new_lane : int
            The new lane
        new_speed : int
            The new speed
        """

        current_position = self._position
        if current_position != new_position:
            self._position = new_position
            LOG.trace(f"{self._vid} teleported to {self._position} (from {current_position}, {self._position - current_position}m)")
            self._joins_teleport_position += 1
        current_lane = self._lane
        if current_lane != new_lane:
            self._lane = new_lane
            LOG.trace(f"{self._vid} switched to lane {self._lane} (from {current_lane})")
            self._joins_teleport_lane += 1
        current_speed = self._speed
        self._cf_target_speed = new_speed
        if current_speed != new_speed:
            self._speed = new_speed
            LOG.trace(f"{self._vid} changed speed to {self._speed}m/s (from {current_speed}m/s)")
            self._joins_teleport_speed += 1

    def _leave(self):
        """
        Lets a vehicle leave a platoon.

        Communication and fine-grained maneuver control is out-of-scope and thus omitted.
        """

        if self._platoon.size == 1:
            LOG.warning(f"Can not leave when driving individually ({self._vid})!")
            return
        assert self.is_in_platoon()

        leader = self._platoon.leader

        LOG.trace(f"{self._vid} is trying to leave platoon {self._platoon.platoon_id} (leader {leader.vid})")
        self._leaves_attempted += 1

        if leader.in_maneuver:
            # the platoon leader has already a (join) maneuver ongoing
            assert leader._joiner
            LOG.warning(f"{leader.vid} currently has a (join) maneuver by {leader._joiner.vid} ongoing. For now, we are going to abort this (join) maneuver as handling this situation is not yet implemented properly!")
            # TODO implement complex leave to not abort the ongoing (join) maneuver
            # abort ongoing (join) maneuver
            leader._joiner._join_approach_step = None
            leader._joiner._join_data_leader = None
            leader._joiner._join_data_last = None
            leader._joiner._join_data_new_position = None
            leader._joiner._joins_aborted += 1
            leader._joiner._joins_aborted_leave_other += 1
            leader._joiner.in_maneuver = False
            leader._joiner._platoon_role = PlatoonRole.NONE
            leader._joiner = None
            leader.in_maneuver = False
        # by checking the leader first, we should already cover cases where self == leader
        assert not self.in_maneuver

        self.in_maneuver = True
        self._platoon_role = PlatoonRole.LEAVER

        if self is leader:
            # leave at front
            self._leaves_front += 1

            if self._platoon.size == 2:
                # tell the only follower to drive individually
                follower = self._platoon.last
                assert not follower.in_maneuver
                LOG.trace(f"Only {follower.vid} is left in the platoon {self._platoon.platoon_id}. Thus, we are going to destroy the entire platoon.")
                follower._platoon_role = PlatoonRole.NONE
                follower._cf_model = CF_Model.ACC
                follower._cf_target_speed = follower._desired_speed
                follower._platoon = Platoon(follower.vid, [follower], follower._desired_speed)

                # reset color of vehicle
                if self._simulator._gui and self._simulator.step >= self._simulator._gui_start:
                    assert follower._color == follower.platoon.leader._color == follower.color
                    change_gui_vehicle_color(follower.vid, follower._color)

                # statistics
                follower._leaves_attempted += 1
                assert follower._last_platoon_join_time >= 0
                follower._time_in_platoon += follower._simulator.step - follower._last_platoon_join_time
                assert follower._last_platoon_join_position >= 0
                follower._distance_in_platoon += follower.position - follower._last_platoon_join_position
                follower._leaves_successful += 1
                follower._leaves_back += 1
            else:
                # tell the second vehicle in the platoon to become the new leader
                new_leader = self._platoon.formation[1]
                new_leader._platoon_role = PlatoonRole.LEADER
                new_leader._cf_model = CF_Model.ACC
                LOG.debug(f"{new_leader.vid} became leader of platoon {new_leader.platoon.platoon_id}")
        elif self is self._platoon.last:
            # leave at back
            leader.in_maneuver = True
            self._leaves_back += 1

            if self._platoon.size == 2:
                # tell the current leader to drive individually
                LOG.trace(f"Only the current leader {leader.vid} is left in the platoon {self._platoon.platoon_id}. Thus, we are going to destroy the entire platoon.")
                leader._platoon_role = PlatoonRole.NONE
                leader._cf_model = CF_Model.ACC  # TODO superfluous?
                leader._cf_target_speed = leader._desired_speed  # TODO superfluous?
                leader._platoon = Platoon(leader.vid, [leader], leader._desired_speed)

                # reset color of vehicle
                if self._simulator._gui and self._simulator.step >= self._simulator._gui_start:
                    assert leader._color == leader.platoon.leader._color == leader.color
                    change_gui_vehicle_color(leader.vid, leader._color)

                # statistics
                leader._leaves_attempted += 1
                assert leader._last_platoon_join_time >= 0
                leader._time_in_platoon += leader._simulator.step - leader._last_platoon_join_time
                assert leader._last_platoon_join_position >= 0
                leader._distance_in_platoon += leader.position - leader._last_platoon_join_position
                leader._leaves_successful += 1
                leader._leaves_front += 1
        else:
            # leave in the middle
            leader.in_maneuver = True
            self._leaves_arbitrary += 1

            # we do not need to switch lanes if we arrived
            if self._position < self._arrival_position:
                # leave the formation by changing the lane
                # TODO this looks strange in the GUI, since we do this before actually leaving the formation
                self._platoon_role = PlatoonRole.LEAVER
                self._cf_model = CF_Model.ACC
                if not self._simulator._change_lane(self, self._lane + 1, "maneuver"):
                    # could not leave the platoon
                    # TODO this could be just a return in future to let the leaver try again
                    sys.exit(f"ERROR: Could not move vehicle {self._vid} to the adjacent lane to leave the platoon {self._platoon.platoon_id}!")

            # move all remaining platoon members further to the front
            front = self._platoon.get_front(self)
            for vehicle in self._platoon.formation[self._platoon.get_member_index(self) + 1:]:
                follower_gap = front.rear_position - vehicle.position
                gap_error = follower_gap - vehicle._cacc_spacing
                LOG.trace(f"The open gap between {front.vid} and {vehicle.vid} is {follower_gap}m (error of {gap_error}m)")
                if gap_error <= 0:
                    LOG.trace("We do not need to do anything")
                    front = vehicle
                    continue
                if vehicle.position >= vehicle.arrival_position:
                    LOG.trace(f"We are not moving {vehicle.vid} because it arrived as well")
                    continue
                LOG.trace(f"Moving follower {vehicle.vid} from {vehicle.position} by {gap_error}m ")
                vehicle._position += gap_error
                LOG.trace(f"{vehicle.vid} is not at {vehicle.position}")
                follower_gap = front.rear_position - vehicle.position
                # avoid issues due to floating point precision
                assert math.isclose(follower_gap, vehicle._cacc_spacing)
                front = vehicle
                assert self._simulator._get_predecessor(vehicle) in vehicle.platoon.formation

        # leave the platoon
        self._platoon.formation.remove(self)
        if self._simulator._update_desired_speed:
            self._platoon.update_desired_speed()
        self._platoon.update_limits()

        # update formation for all (remaining) members
        # TODO this could be nicer, e.g., by taking the (new) leader's updated formation
        if self._platoon.size > 1:
            # more than one remaining member
            for vehicle in self._platoon.formation:
                vehicle._platoon = self._platoon

        # leave
        LOG.debug(f"{self._vid} left platoon {self._platoon.platoon_id} (leader {self._platoon.leader.vid})")
        self._platoon_role = PlatoonRole.NONE  # the current platoon role
        self._platoon = Platoon(self._vid, [self], self._desired_speed)  # use explicit individual desired speed
        self._cf_model = CF_Model.ACC  # not necessary, but we still do it explicitly

        # reset color of vehicle
        if self._simulator._gui and self._simulator.step >= self._simulator._gui_start:
            assert self._color == self.platoon.leader._color == self.color
            change_gui_vehicle_color(self._vid, self._color)

        self.in_maneuver = False
        if self is not leader:
            leader.in_maneuver = False

        assert self._last_platoon_join_time >= 0
        self._time_in_platoon += self._simulator.step - self._last_platoon_join_time
        assert self._last_platoon_join_position >= 0
        self._distance_in_platoon += self._position - self._last_platoon_join_position
        self._leaves_successful += 1

    def _advertise(self):
        """
        Maintains regular sending of platoon advertisements.

        Advertisement are in general not used at the moment.

        THIS IS DEPRECATED AT THE MOMENT!!!
        """

        return  # TODO this is not necessary as a perfect communication guarantees information

        advertisement_interval = 600  # in s # TODO make parameter
        if (
            not self._last_advertisement_step
            or self._last_advertisement_step + advertisement_interval <= self._simulator.step
        ):
            self._send_advertisements()
            self._last_advertisement_step = self._simulator.step

    def _send_advertisements(self):
        """
        Transmit a broadcast to advertise as platoon.

        Advertisement are in general not used at the moment.

        THIS IS DEPRECATED AT THE MOMENT!!!
        """

        for vehicle in self._simulator._vehicles.values():
            self._transmit(-1, PlatoonAdvertisement(
                self._vid,
                vehicle.vid,
                self._vid,
                self._vid,
                self._speed,
                self._lane,
                self._vid,
                self._position,
                self.rear_position,
            ))

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

        func = self.__class__.__dict__.get(
            '_receive_' + message.__class__.__name__,
            super().__dict__.get('_handle_message'))
        return func(self, message)

    def _receive_PlatoonAdvertisement(self, advertisement: PlatoonAdvertisement):
        """
        Handles a message of the specific type PlatoonAdvertisement.

        Messages are in general not used at the moment.

        Parameters
        ----------
        advertisement : PlatoonAdvertisement
            The advertisement to be received

        THIS IS DEPRECATED AT THE MOMENT!!!
        """

        # TODO add contents to the neighbor table
        LOG.debug(f"{self._vid} received an advertisement from {advertisement.origin}")
