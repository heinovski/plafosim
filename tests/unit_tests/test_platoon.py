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

import random

from plafosim.platoon import Platoon
from plafosim.platooning_vehicle import PlatooningVehicle
from plafosim.simulator import vtype


class FakeSimulator:
    _rng = random


def test_creation():
    v1 = PlatooningVehicle(FakeSimulator(), 1, vtype, 100, 1000, 36, 0, 0, 0)
    v2 = PlatooningVehicle(FakeSimulator(), 2, vtype, 90, 1000, 36, 0, 0, 1)
    v3 = PlatooningVehicle(FakeSimulator(), 3, vtype, 80, 1000, 36, 0, 0, 2)
    v3._vehicle_type._max_speed = vtype.max_speed - 10
    v3._vehicle_type._max_acceleration = vtype.max_acceleration - 1
    v3._vehicle_type._max_deceleration = vtype.max_deceleration - 5

    platoon_id = 2
    formation = [v1, v2, v3]
    platoon = Platoon(platoon_id, formation, v1.desired_speed)

    assert platoon is not None
    assert platoon.platoon_id == platoon_id
    assert platoon.leader is v1
    assert platoon.last is v3
    assert platoon.formation is formation
    assert platoon.desired_speed == v1.desired_speed
    assert platoon.speed == 0
    assert platoon.lane == 0
    assert platoon.max_speed == v3.max_speed
    assert platoon.max_acceleration == v3.max_acceleration
    assert platoon.max_deceleration == v3.max_deceleration
    assert platoon.size == 3
    assert platoon.position == v1.position
    assert platoon.rear_position == v3.rear_position
    assert platoon.length == v1.position - v3.rear_position
    assert platoon.member_ids == [1, 2, 3]
    assert platoon.get_member_index(v2) == 1
    assert platoon.get_front(v3) == v2
