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
from plafosim.simulator import Simulator


class TestSimulator:

    def setup(self):
        self.s = Simulator()
        assert(self.s is not None)

    def test__init__(self):
        self.setup()

        assert(len(self.s._vehicles) == 0)
        assert(len(self.s._infrastructures) == 0)

    def test_speed2distance(self):
        assert(Simulator.speed2distance(36.0) == 36.0)
        assert(Simulator.speed2distance(26.0, 10) == 260.0)

    def test_distance2speed(self):
        assert(Simulator.distance2speed(275.0) == 275.0)
        assert(Simulator.distance2speed(257.0, 10) == 25.7)

    def test_acceleration2speed(self):
        assert(Simulator.acceleration2speed(2.5) == 2.5)
        assert(Simulator.acceleration2speed(5.2, 10) == 52.0)  # TODO is this physically correct?

    def test_speed2acceleration(self):
        assert(Simulator.speed2acceleration(36.0, 24.0) == -12.0)
        assert(Simulator.speed2acceleration(32.0, 42.0, 10) == 1.0)  # TODO is this physically correct?

    def test_get_predecessor(self):
        self.setup()
        self.s._step = 1
        self.s._spawn_vehicle()
#        self.s._vehicles[0]._started = True
        self.s._step = 2
        self.s._spawn_vehicle()
#        self.s._vehicles[1]._started = True
        self.s._step = 3
        self.s._spawn_vehicle()
#        self.s._vehicles[2]._started = True
        self.s._vehicles[0]._position = 100
        self.s._vehicles[0]._lane = 0
        self.s._vehicles[1]._position = 60
        self.s._vehicles[1]._lane = 0
        self.s._vehicles[2]._position = 20
        self.s._vehicles[2]._lane = 0

        # test simple situation
        assert(self.s._get_predecessor(self.s._vehicles[0]) is None)
        assert(self.s._get_predecessor(self.s._vehicles[1]) is self.s._vehicles[0])
        assert(self.s._get_predecessor(self.s._vehicles[2]) is self.s._vehicles[1])

        # test two vehicles same position
        # this should throw an error in a real simulation
        self.s._vehicles[1]._position = self.s._vehicles[0].position
        assert(self.s._get_predecessor(self.s._vehicles[1]) is self.s._vehicles[0])

        # test two vehicles same position
        # this should throw an error in a real simulation
        self.s._vehicles[1]._position = self.s._vehicles[0].position - 1
        assert(self.s._get_predecessor(self.s._vehicles[1]) is self.s._vehicles[0])

        # test two vehicles same position
        # this should throw an error in a real simulation
        self.s._vehicles[1]._position = self.s._vehicles[0].rear_position
        assert(self.s._get_predecessor(self.s._vehicles[1]) is self.s._vehicles[0])

        # test small inter-vehicle gap
        self.s._vehicles[1]._position = self.s._vehicles[0].rear_position - 1
        assert(self.s._get_predecessor(self.s._vehicles[1]) is self.s._vehicles[0])

        # test changed vehicle order
        self.s._vehicles[1]._position = 120
        assert(self.s._get_predecessor(self.s._vehicles[1]) is None)
        assert(self.s._get_predecessor(self.s._vehicles[0]) is self.s._vehicles[1])
        assert(self.s._get_predecessor(self.s._vehicles[2]) is self.s._vehicles[0])

        # test vehicle on adjacent lane
        self.s._vehicles[0]._lane = 1
        assert(self.s._get_predecessor(self.s._vehicles[0]) is None)
        assert(self.s._get_predecessor(self.s._vehicles[2]) is self.s._vehicles[1])

        # test vehicle next to us on adjacent lane
        self.s._vehicles[2]._position = self.s._vehicles[0].position - 1
        assert(self.s._get_predecessor(self.s._vehicles[0], self.s._vehicles[2].lane) is self.s._vehicles[1])
        assert(self.s._get_predecessor(self.s._vehicles[2], self.s._vehicles[0].lane) is self.s._vehicles[0])

    def test_spawn_vehicle(self):
        self.setup()

        ## test depart method interval
        self.s._depart_method = 'interval'
        self.s._depart_time_interval = 1

        # test simple spawning
        assert(len(self.s._vehicles) == 0)
        self.s._spawn_vehicle()
        assert(len(self.s._vehicles) == 1)

        # test only one spawn per time step
        self.s._spawn_vehicle()
        assert(len(self.s._vehicles) == 1)

        # TODO test other depart methods
