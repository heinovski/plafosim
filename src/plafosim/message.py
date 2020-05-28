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
from enum import Enum


class MessageType(Enum):
    """A collection of available message types"""

    CAM = 1  # corresponds to a regular beacon (e.g., ETSI CAM)
    MANEUVER = 2  # corresponds to a maneuver message
    PLATOON_ADVERTISEMENT = 3  # corresponds to a regular advertisement of a platoon


class Message:
    """A collection of general data for an arbitrary message"""

    def __init__(self, origin: int, destination: int, message_type, *data):
        self._origin = origin  # id of the originiator of this message
        self._destination = destination  # id of the destination of this message
        self._message_type = message_type  # MessageType of this message
        self._data = data  # generic data of this message

    @property
    def origin(self) -> int:
        """Get the originator of the message"""
        return self._origin

    @property
    def destination(self) -> int:
        """Get the destination of the message"""
        return self._destination

    @property
    def data(self):
        """Get the data of the message"""
        return self._data

    def __str__(self) -> str:
        return "%d -> %d (%s): %s" % (self._origin, self._destination, self._message_type, self._data)


class ManeuverType(Enum):
    """A collection of available maneuver (message) types"""

    JOIN = 1  # corresponds to a join maneuver
    LEAVE = 2  # corresponds to a leave maneuver
    MERGE = 3  # corresponds to a merge maneuver
    SPLIT = 4  # corresponds to a split maneuver
    LANE_CHANGE = 5  # corresponds to a lane change maneuver


class ManeuverMessage(Message):
    """A collection of general data for an arbitrary maneuver"""

    def __init__(self, origin: int, destination: int, maneuver_type, platoon_id: int, leader_id: int):
        super().__init__(origin, destination, MessageType.MANEUVER)
        self._maneuver_type = maneuver_type  # ManeuverType of this message
        self._platoon_id = platoon_id  # id of the platoon the message corresponds to
        self._leader_id = leader_id  # id of the leader of the corresponding platoon

    @property
    def platoon_id(self) -> int:
        """Get the platoon id the message corresponds to"""
        return self._platoon_id

    @property
    def leader_id(self) -> int:
        """Get the leader id of the platoon the message corresponds to"""
        return self._leader_id


# TODO
class JoinPlatoonRequest(ManeuverMessage):
    # joiner_speed
    # joiner_lane
    # joiner_position
    # joiner_length
    pass


# TODO
class JoinPlatoonResponse(ManeuverMessage):
    # permitted
    pass


# TODO
class MoveToPosition(ManeuverMessage):
    # platoon_speed
    # platoon_lane
    # platoon_position_front
    # platoon_position_back
    # new_platoon_formation
    pass


# TODO
class MoveToPositionAck(ManeuverMessage):
    # platoon_speed
    # platoon_lane
    # new_platoon_formation
    pass


# TODO
class JoinFormation(ManeuverMessage):
    # platoon_speed
    # platoon_lane
    # new_platoon_formation
    pass


# TODO
class JoinFormationAck(ManeuverMessage):
    # platoon_speed
    # platoon_lane
    # new_platoon_formation
    pass


# TODO
class UpdatePlatoonFormation(ManeuverMessage):
    # platoon_speed
    # platoon_lane
    # new_platoon_formation
    pass


# TODO
class UpdatePlatoonFormationAck(ManeuverMessage):
    # platoon_speed
    # platoon_lane
    # new_platoon_formation
    pass


# TODO
class AbortManeuver(ManeuverMessage):
    pass


class PlatoonAdvertisement(Message):

    def __init__(
            self,
            origin: int,
            destination: int,
            platoon_id: int,
            leader_id: int,
            platoon_speed: int,
            platoon_lane: int,
            platoon_formation: list,
            platoon_position_front: int,
            platoon_position_back: int):
        super().__init__(origin, destination, MessageType.PLATOON_ADVERTISEMENT, platoon_id, leader_id)
        self._platoon_speed = platoon_speed  # current speed of the advertised platoon
        self._platoon_lane = platoon_lane  # current lane of the advertised platoon
        self._platoon_formation = platoon_formation  # current formation of the advertised platoon
        # current position of the front of the advertised platoon (front of leader)
        self._platoon_position_front = platoon_position_front
        # current position of the back of the advertised platoon (back of last vehicle)
        self._platoon_position_back = platoon_position_back

    @property
    def platoon_speed(self) -> int:
        return self._platoon_speed

    @property
    def platoon_lane(self) -> int:
        return self._platoon_lane

    @property
    def platoon_formation(self) -> list:
        return self._platoon_formation

    @property
    def platoon_position_front(self) -> int:
        return self._platoon_position_front

    @property
    def platoon_position_back(self) -> int:
        return self._platoon_position_back
