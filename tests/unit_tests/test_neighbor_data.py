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

from plafosim.neighbortable import NeighborData


def test_creation():

    vid = 1
    originator_id = 42
    platoon_id = 2
    leader_id = 3
    platoon_speed = 36
    platoon_lane = 0
    platoon_formation = [3, 42, 1]
    platoon_position_front = 1000
    platoon_position_back = [1022]
    timestamp = 1337

    nd = NeighborData(vid, originator_id, platoon_id, leader_id, platoon_speed, platoon_lane, platoon_formation, platoon_position_front, platoon_position_back, timestamp)

    assert(nd is not None)
    assert(nd.is_valid())
    assert(nd.vid == vid)
    assert(nd.originator_id == originator_id)
    assert(nd.platoon_id == platoon_id)
    assert(nd.leader_id == leader_id)
    assert(nd.platoon_speed == platoon_speed)
    assert(nd.platoon_lane == platoon_lane)
    assert(nd.platoon_formation == platoon_formation)
    assert(nd.platoon_position_front == platoon_position_front)
    assert(nd.platoon_position_back == platoon_position_back)
    assert(nd.timestamp == timestamp)
