#
# Copyright (c) 2023-2023 Arno Bock <arno.bock@hotmail.de>
# Copyright (c) 2023-2024 Julian Heinovski <heinovski@ccs-labs.org>
#
# SPDX-License-Identifier: GPL-3.0-or-later
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
from plafosim.simulator import Simulator, vtype


class TestLeaveSimulator:

    def create_base_scenario(self):
        """Create initial vehicle that wants to check whether left lane is blocked."""

        # create simulator
        self.s = Simulator()
        assert self.s

        # add vehicle that wants to leave (not specified) platoon
        self.s._add_vehicle(
            vid=0,
            vtype=vtype,
            depart_position=100,
            arrival_position=1000,
            desired_speed=36,
            depart_lane=0,
            depart_speed=36,
            depart_time=0,
        )

    def test_lane_empty(self):
        """Test left lane blocked check for "empty" lane."""

        self.create_base_scenario()

        # add vehicle (front; on left lane)
        self.s._add_vehicle(
            vid=1,
            vtype=vtype,
            depart_position=self.s._vehicles[0].position + 100,
            arrival_position=1000,
            desired_speed=36,
            depart_lane=1,
            depart_speed=36,
            depart_time=0,
        )

        # add vehicle (back; on left lane)
        self.s._add_vehicle(
            vid=2,
            vtype=vtype,
            depart_position=0,
            arrival_position=1000,
            desired_speed=36,
            depart_lane=1,
            depart_speed=36,
            depart_time=0,
        )

        assert not self.s._vehicles[0]._left_lane_blocked()

    def test_lane_blocked(self):
        """Test left lane blocked check with interferer."""

        self.create_base_scenario()

        # add vehicle (front; on left lane)
        self.s._add_vehicle(
            vid=1,
            vtype=vtype,
            depart_position=self.s._vehicles[0].position,
            arrival_position=1000,
            desired_speed=36,
            depart_lane=1,
            depart_speed=36,
            depart_time=0,
        )

        assert self.s._vehicles[0]._left_lane_blocked()

    def test_lane_blocked_front(self):
        """Test left lane blocked check with interferer in the front."""

        self.create_base_scenario()

        # add vehicle (front; on left lane)
        self.s._add_vehicle(
            vid=1,
            vtype=vtype,
            depart_position=self.s._vehicles[0].position + 10,
            arrival_position=1000,
            desired_speed=36,
            depart_lane=1,
            depart_speed=36,
            depart_time=0,
        )

        # add vehicle (back; on left lane)
        self.s._add_vehicle(
            vid=2,
            vtype=vtype,
            depart_position=0,
            arrival_position=1000,
            desired_speed=36,
            depart_lane=1,
            depart_speed=36,
            depart_time=0,
        )

        assert self.s._vehicles[0]._left_lane_blocked()

    def test_lane_blocked_back(self):
        """Test left lane blocked check with interferer in the back."""

        self.create_base_scenario()

        # add vehicle (front; on left lane)
        self.s._add_vehicle(
            vid=1,
            vtype=vtype,
            depart_position=self.s._vehicles[0].position + 100,
            arrival_position=1000,
            desired_speed=36,
            depart_lane=1,
            depart_speed=36,
            depart_time=0,
        )

        # add vehicle (back; on left lane)
        self.s._add_vehicle(
            vid=2,
            vtype=vtype,
            depart_position=self.s._vehicles[0].position - 1,
            arrival_position=1000,
            desired_speed=36,
            depart_lane=1,
            depart_speed=36,
            depart_time=0,
        )

        assert self.s._vehicles[0]._left_lane_blocked()

    def test_lane_blocked_back_very_close(self):
        """
        Test left lane blocked check with interferer in the back.

        The blocking vehicle is 1m behind leaving vehicle on left lane.
        """

        self.create_base_scenario_leave()

        # add far vehicle (front; on left lane)
        self.s._add_vehicle(
            vid=6,
            vtype=vtype,
            depart_position=self.s._vehicles[0].position + 150,
            arrival_position=1000,
            desired_speed=36,
            depart_lane=1,
            depart_speed=36,
            depart_time=0,
        )

        # add closer vehicle (front; on left lane)
        self.s._add_vehicle(
            vid=5,
            vtype=vtype,
            depart_position=self.s._vehicles[0].position + 100,
            arrival_position=1000,
            desired_speed=36,
            depart_lane=1,
            depart_speed=36,
            depart_time=0,
        )
        # add close vehicle (back; on left lane)
        self.s._add_vehicle(
            vid=3,
            vtype=vtype,
            depart_position=self.s._vehicles[1].position - 1,
            arrival_position=1000,
            desired_speed=36,
            depart_lane=1,
            depart_speed=36,
            depart_time=0,
        )
        # add far vehicle (back; on left lane)
        self.s._add_vehicle(
            vid=4,
            vtype=vtype,
            depart_position=0,
            arrival_position=1000,
            desired_speed=36,
            depart_lane=1,
            depart_speed=36,
            depart_time=0,
        )
        self.s._vehicles[1]._leave()

        leaver = self.s._vehicles[1]

        assert self.s._vehicles[1]._left_lane_blocked()
        assert leaver.is_in_platoon()

    def test_lane_blocked_front_2vehicles(self):
        """Test left lane blocked check with 2 interferers in the front."""

        self.create_base_scenario()

        # add vehicle (front; on left lane)
        self.s._add_vehicle(
            vid=1,
            vtype=vtype,
            depart_position=self.s._vehicles[0].position + 10,
            arrival_position=1000,
            desired_speed=36,
            depart_lane=1,
            depart_speed=36,
            depart_time=0,
        )

        # add vehicle (front; on left lane)
        self.s._add_vehicle(
            vid=2,
            vtype=vtype,
            depart_position=self.s._vehicles[0].position + 100,
            arrival_position=1000,
            desired_speed=36,
            depart_lane=1,
            depart_speed=36,
            depart_time=0,
        )

        # add vehicle (back; on left lane)
        self.s._add_vehicle(
            vid=3,
            vtype=vtype,
            depart_position=0,
            arrival_position=1000,
            desired_speed=36,
            depart_lane=1,
            depart_speed=36,
            depart_time=0,
        )

        assert self.s._vehicles[0]._left_lane_blocked()

    def test_lane_blocked_back_2vehicles(self):
        """Test left lane blocked check with 2 interferers in the back."""

        self.create_base_scenario()

        # add vehicle (front; on left lane)
        self.s._add_vehicle(
            vid=1,
            vtype=vtype,
            depart_position=self.s._vehicles[0].position + 100,
            arrival_position=1000,
            desired_speed=36,
            depart_lane=1,
            depart_speed=36,
            depart_time=0,
        )

        # add vehicle (back; on left lane)
        self.s._add_vehicle(
            vid=2,
            vtype=vtype,
            depart_position=0,
            arrival_position=1000,
            desired_speed=36,
            depart_lane=1,
            depart_speed=36,
            depart_time=0,
        )

        # add vehicle (back; on left lane)
        self.s._add_vehicle(
            vid=2,
            vtype=vtype,
            depart_position=self.s._vehicles[0].position - 10,
            arrival_position=1000,
            desired_speed=36,
            depart_lane=1,
            depart_speed=36,
            depart_time=0,
        )

        assert self.s._vehicles[0]._left_lane_blocked()

    def create_base_scenario_leave(self):
        """Create initial platoon from which the vehicle in the middle wants to leave."""

        # create simulation environment
        self.s = Simulator(
            number_of_lanes=2,
            number_of_vehicles=3,
            penetration_rate=1,
            start_as_platoon=True,
            pre_fill=True,
            random_desired_speed=False,
            update_desired_speed=False,
            depart_desired=True,
            desired_speed=33,
        )
        assert self.s
        assert self.s._vehicles[0].platoon.size == 3
        assert self.s._vehicles[0].platoon.length == 3 * vtype.length + 2 * self.s._cacc_spacing

        for vehicle in self.s._vehicles.values():
            vehicle._position += 100

        assert self.s._vehicles[0].position == 100 + self.s._vehicles[0].platoon.length

    def test_leave_in_middle_empty(self):
        """Test leave in the nicke for "empty" lane."""

        self.create_base_scenario_leave()

        # add vehicle (front; on left lane)
        self.s._add_vehicle(
            vid=3,
            vtype=vtype,
            depart_position=self.s._vehicles[0].position + 100,
            arrival_position=1000,
            desired_speed=36,
            depart_lane=1,
            depart_speed=36,
            depart_time=0,
        )

        # add vehicle (back; on left lane)
        self.s._add_vehicle(
            vid=4,
            vtype=vtype,
            depart_position=0,
            arrival_position=1000,
            desired_speed=36,
            depart_lane=1,
            depart_speed=36,
            depart_time=0,
        )

        self.s._vehicles[1]._leave()

        assert self.s._vehicles[0].platoon.size == 2
        assert not self.s._vehicles[1].is_in_platoon()
        assert self.s._vehicles[1].lane == self.s._vehicles[3].lane == self.s._vehicles[4].lane
        assert self.s._vehicles[0].platoon.length == 2 * vtype.length + self.s._cacc_spacing

    def test_leave_in_middle_blocked(self):
        """Test leave in the middle with interferer."""

        self.create_base_scenario_leave()

        # add vehicle (front; on left lane)
        self.s._add_vehicle(
            vid=3,
            vtype=vtype,
            depart_position=self.s._vehicles[1].position,
            arrival_position=1000,
            desired_speed=36,
            depart_lane=1,
            depart_speed=36,
            depart_time=0,
        )

        self.s._vehicles[1]._leave()

        assert self.s._vehicles[0].platoon.size == 3
        assert self.s._vehicles[1].is_in_platoon()
        assert self.s._vehicles[1].lane == self.s._vehicles[0].lane == self.s._vehicles[2].lane

    def test_leave_in_middle_blocked_front(self):
        """Test leave in the middle with interferer in the front."""

        self.create_base_scenario_leave()

        # add vehicle (front; on left lane)
        self.s._add_vehicle(
            vid=3,
            vtype=vtype,
            depart_position=self.s._vehicles[0].position,
            arrival_position=1000,
            desired_speed=36,
            depart_lane=1,
            depart_speed=36,
            depart_time=0,
        )

        # add vehicle (back; on left lane)
        self.s._add_vehicle(
            vid=4,
            vtype=vtype,
            depart_position=0,
            arrival_position=1000,
            desired_speed=36,
            depart_lane=1,
            depart_speed=36,
            depart_time=0,
        )

        self.s._vehicles[1]._leave()

        assert self.s._vehicles[0].platoon.size == 3
        assert self.s._vehicles[1].is_in_platoon()
        assert self.s._vehicles[1].lane == self.s._vehicles[0].lane == self.s._vehicles[2].lane

    def test_leave_in_middle_blocked_back(self):
        """Test leave in the middle with interferer in the back."""

        self.create_base_scenario_leave()

        # add vehicle (front; on left lane)
        self.s._add_vehicle(
            vid=3,
            vtype=vtype,
            depart_position=self.s._vehicles[0].position + 100,
            arrival_position=1000,
            desired_speed=36,
            depart_lane=1,
            depart_speed=36,
            depart_time=0,
        )

        # add vehicle (back; on left lane)
        self.s._add_vehicle(
            vid=4,
            vtype=vtype,
            depart_position=self.s._vehicles[2].position,
            arrival_position=1000,
            desired_speed=36,
            depart_lane=1,
            depart_speed=36,
            depart_time=0,
        )

        self.s._vehicles[1]._leave()

        assert self.s._vehicles[0].platoon.size == 3
        assert self.s._vehicles[1].is_in_platoon()
        assert self.s._vehicles[1].lane == self.s._vehicles[0].lane == self.s._vehicles[2].lane

    def test_leave_in_middle_platoon_on_leftmost_lane(self):
        """Test leave in the middle with platoon on left most lane."""

        self.create_base_scenario_leave()

        # change platoon to leftmost lane
        for vehicle in self.s._vehicles.values():
            vehicle._lane = self.s.number_of_lanes - 1

        self.s._vehicles[1]._leave()

        assert self.s._vehicles[0].platoon.size == 3
        assert self.s._vehicles[1].is_in_platoon()
        assert self.s._vehicles[1].lane == self.s._vehicles[0].lane == self.s._vehicles[2].lane
