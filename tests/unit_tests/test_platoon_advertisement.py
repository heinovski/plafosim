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

from plafosim.message import Message, PlatoonAdvertisement


def test_creation():
    origin = 1
    destination = -1
    platoon_id = 1
    leader_id = 1
    platoon_speed = 36
    platoon_lane = 0
    platoon_formation = [1, 2]
    platoon_position_front = 30
    platoon_position_back = 30 + 5 + 4
    string = "%d -> %d (PlatoonAdvertisement): None" % (origin, destination)

    message = PlatoonAdvertisement(origin, destination, platoon_id, leader_id, platoon_speed, platoon_lane, platoon_formation, platoon_position_front, platoon_position_back)

    assert message is not None
    assert message.data is None
    assert message.origin == origin
    assert message.destination == destination
    assert message.platoon_id == platoon_id
    assert message.leader_id == leader_id
    assert message.platoon_speed == platoon_speed
    assert message.platoon_lane == platoon_lane
    assert message.platoon_formation == platoon_formation
    assert message.platoon_position_front == platoon_position_front
    assert message.platoon_position_back == platoon_position_back
    assert str(message) == string
    assert isinstance(message, Message)
