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
from plafosim.message import ManeuverMessage, ManeuverType, Message


def test_creation():
    origin = 1
    destination = -1
    maneuver_type = ManeuverType.JOIN  # TODO check simple integer as maneuver type
    platoon_id = 1
    leader_id = 1
    string = "%d -> %d (ManeuverMessage): None" % (origin, destination)

    message = ManeuverMessage(origin, destination, maneuver_type, platoon_id, leader_id)

    assert(message is not None)
    assert(message.origin == origin)
    assert(message.destination == destination)
    assert(message.maneuver_type == maneuver_type)
    assert(message.data is None)
    assert(message.platoon_id == platoon_id)
    assert(message.leader_id == leader_id)
    assert(str(message) == string)
    assert(isinstance(message, Message))
