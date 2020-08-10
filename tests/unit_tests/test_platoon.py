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
from plafosim.vehicle import Platoon


def test_creation():
    pid = 2
    lid = 1
    formation = [1, 2, 3]
    speed = 36
    lane = 0
    max_speed = 55
    max_acceleration = 2.5
    max_deceleration = 15

    platoon = Platoon(pid, lid, formation, speed, lane, max_speed, max_acceleration, max_deceleration)

    assert(platoon is not None)
    assert(platoon.pid == pid)
    assert(platoon.lid == lid)
    assert(platoon.formation == formation)
    assert(platoon.speed == speed)
    assert(platoon.lane == lane)
    assert(platoon.max_speed == max_speed)
    assert(platoon.max_acceleration == max_acceleration)
    assert(platoon.max_deceleration == max_deceleration)
