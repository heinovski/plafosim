#
# Copyright (c) 2020-2022 Julian Heinovski <heinovski@ccs-labs.org>
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

from plafosim.message import Message


def test_creation():
    origin = 1
    destination = 2
    data = "Hello World"
    string1 = "%d -> %d (Message): None" % (origin, destination)
    string2 = "%d -> %d (Message): %s" % (origin, destination, data)

    message1 = Message(origin, destination)
    message2 = Message(origin, destination, data)

    assert message1 is not None
    assert message1.data is None
    assert message1.origin == origin
    assert message1.destination == destination
    assert str(message1) == string1

    assert message2.data is data
    assert str(message2) == string2
