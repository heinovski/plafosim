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

from plafosim.simulator import Simulator, vtype


class TestSimulator:

    def setup(self):
        self.s = Simulator()
        assert self.s is not None

    def test__init__(self):
        self.setup()

        assert len(self.s._vehicles) == 0
        assert len(self.s._infrastructures) == 0

    def test_get_predecessor(self):
        self.setup()
        self.s._add_vehicle(
            0,
            vtype,
            100,
            1000,
            36,
            0,
            36,
            0,
        )
        self.s._add_vehicle(
            1,
            vtype,
            60,
            1000,
            36,
            0,
            36,
            1,
        )
        self.s._add_vehicle(
            2,
            vtype,
            20,
            1000,
            36,
            0,
            36,
            2,
        )

        # test simple situation
        assert self.s._get_predecessor(self.s._vehicles[0]) is None
        assert self.s._get_predecessor(self.s._vehicles[1]) is self.s._vehicles[0]
        assert self.s._get_predecessor(self.s._vehicles[2]) is self.s._vehicles[1]

        # test two vehicles same position
        # this should throw an error in a real simulation
        self.s._vehicles[1]._position = self.s._vehicles[0].position
        assert self.s._get_predecessor(self.s._vehicles[1]) is self.s._vehicles[0]

        # test two vehicles same position
        # this should throw an error in a real simulation
        self.s._vehicles[1]._position = self.s._vehicles[0].position - 1
        assert self.s._get_predecessor(self.s._vehicles[1]) is self.s._vehicles[0]

        # test two vehicles same position
        # this should throw an error in a real simulation
        self.s._vehicles[1]._position = self.s._vehicles[0].rear_position
        assert self.s._get_predecessor(self.s._vehicles[1]) is self.s._vehicles[0]

        # test small inter-vehicle gap
        self.s._vehicles[1]._position = self.s._vehicles[0].rear_position - 1
        assert self.s._get_predecessor(self.s._vehicles[1]) is self.s._vehicles[0]

        # test changed vehicle order
        self.s._vehicles[1]._position = 120
        assert self.s._get_predecessor(self.s._vehicles[1]) is None
        assert self.s._get_predecessor(self.s._vehicles[0]) is self.s._vehicles[1]
        assert self.s._get_predecessor(self.s._vehicles[2]) is self.s._vehicles[0]

        # test vehicle on adjacent lane
        self.s._vehicles[0]._lane = 1
        assert self.s._get_predecessor(self.s._vehicles[0]) is None
        assert self.s._get_predecessor(self.s._vehicles[2]) is self.s._vehicles[1]

        # test vehicle next to us on adjacent lane
        self.s._vehicles[2]._position = self.s._vehicles[0].position - 1
        assert self.s._get_predecessor(self.s._vehicles[0], self.s._vehicles[2].lane) is self.s._vehicles[1]
        assert self.s._get_predecessor(self.s._vehicles[2], self.s._vehicles[0].lane) is self.s._vehicles[0]

    def test_get_predecessor_rear_position(self):
        self.setup()
        self.s._add_vehicle(
            0,
            vtype,
            100,
            1000,
            36,
            0,
            36,
            0,
        )
        self.s._add_vehicle(
            1,
            vtype,
            60,
            1000,
            36,
            0,
            36,
            1,
        )
        self.s._add_vehicle(
            2,
            vtype,
            20,
            1000,
            36,
            0,
            36,
            2,
        )

        # test simple situation
        assert self.s._get_predecessor_rear_position(self.s._vehicles[0]) == -1
        assert self.s._get_predecessor_rear_position(self.s._vehicles[1]) == self.s._vehicles[0].rear_position
        assert self.s._get_predecessor_rear_position(self.s._vehicles[2]) == self.s._vehicles[1].rear_position

        # we are skipping more complex scenarios, since they are handled by test_predecessor

    def test_get_predecessor_speed(self):
        self.setup()
        self.s._add_vehicle(
            0,
            vtype,
            100,
            1000,
            36,
            0,
            36,
            0,
        )
        self.s._add_vehicle(
            1,
            vtype,
            60,
            1000,
            40,
            0,
            36,
            1,
        )
        self.s._add_vehicle(
            2,
            vtype,
            20,
            1000,
            36,
            0,
            27.5,
            2,
        )

        # test simple situation
        assert self.s._get_predecessor_speed(self.s._vehicles[0]) == -1
        assert self.s._get_predecessor_speed(self.s._vehicles[1]) == self.s._vehicles[0].speed
        assert self.s._get_predecessor_speed(self.s._vehicles[2]) == self.s._vehicles[1].speed

        # we are skipping more complex scenarios, since they are handled by test_predecessor
