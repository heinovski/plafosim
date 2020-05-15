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
