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
    CAM = 1
    MANEUVER = 2
    PLATOON_ADVERTISEMENT = 3


class Message:

    def __init__(self, origin, destination, message_type, *data):
        self._origin = origin  # id of the originiator of this message
        self._destination = destination  # id of the destination of this message
        self._message_type = message_type  # MessageType of this message
        self._data = data  # generic data of this message

    @property
    def origin(self):
        return self._origin

    @property
    def destination(self):
        return self._destination

    @property
    def data(self):
        return self._data

    def __str__(self):
        return "%d -> %d (%s): %s" % (self._origin,  self._destination, self._message_type, self._data)


class ManeuverType(Enum):
    JOIN = 1
    LEAVE = 2
    MERGE = 3
    SPLIT = 4
    LANE_CHANGE = 5


class ManeuverMessage(Message):

    def __init__(self, origin, destination, maneuver_type, platoon_id, leader_id):
        super().__init__(origin, destination, MessageType.MANEUVER)
        self._maneuver_type = maneuver_type  # ManeuverType of this message
        self._platoon_id = platoon_id  # id of the platoon this message is about
        self._leder_id = leader_id  # id of the corresponding leader of the platoon

    @property
    def platoon_id(self):
        return self._platoon_id

    @property
    def leader_id(self):
        return self._leader_id


class PlatoonAdvertisement(Message):

    def __init__(self, origin, destination, platoon_id, leader_id, platoon_speed, platoon_lane, platoon_formation, platoon_position_front, platoon_position_back):
        super().__init__(origin, destination, MessageType.PLATOON_ADVERTISEMENT, platoon_id, leader_id)
        self._platoon_speed = platoon_speed  # current speed of the advertised platoon
        self._platoon_lane = platoon_lane  # current lane of the advertised platoon
        self._platoon_formation = platoon_formation  # current formation of the advertised platoon
        self._platoon_position_front = platoon_position_front  # current position of the front of the advertised platoon (front of leader)
        self._platoon_position_back = platoon_position_back  # current position of the back of the advertised platoon (back of last vehicle)

    @property
    def platoon_speed(self):
        return self._platoon_speed

    @property
    def platoon_lane(self):
        return self._platoon_lane

    @property
    def platoon_formation(self):
        return self._platoon_formation

    @property
    def platoon_position_front(self):
        return self._platoon_position_front

    @property
    def platoon_position_back(self):
        return self._platoon_position_back
