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

from plafosim import Platoon, PlatooningVehicle


def test_creation():
    v1 = PlatooningVehicle(None, 1, None, 0, 1000, 36, 0, 0, 0, -1, 1, 5, None, 1, 0.5, 0.1, 300)
    v2 = PlatooningVehicle(None, 2, None, 0, 1000, 36, 0, 0, 1, -1, 1, 5, None, 1, 0.5, 0.1, 300)
    v3 = PlatooningVehicle(None, 3, None, 0, 1000, 36, 0, 0, 2, -1, 1, 5, None, 1, 0.5, 0.1, 300)

    platoon_id = 2
    formation = [v1, v2, v3]
    platoon = Platoon(platoon_id, formation, v1.desired_speed)

    assert(platoon is not None)
    assert(platoon.platoon_id == platoon_id)
    assert(platoon.leader is v1)
    assert(platoon.last is v3)
    assert(platoon.formation is formation)
    assert(platoon.desired_speed == v1.desired_speed)
    assert(platoon.speed == 0)
    assert(platoon.lane == 0)
    # TODO max_acceleration
    # TODO max_deceleration
    assert(platoon.size == 3)
    assert(platoon.position == 0)
    # TODO rear_position
    # TODO length
    assert(platoon.member_ids == [1, 2, 3])
    assert(platoon.get_member_index(v2) == 1)
    assert(platoon.get_front(v3) is v2)
