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
import logging


class Infrastructure:

    def __init__(self, iid: int, position: int):
        self._iid = iid  # the id of the infrastructure
        self._position = position  # the x position of the infrastructure

    @property
    def iid(self) -> int:
        """Return the id of a infrastructure"""
        return self._iid

    @property
    def position(self) -> int:
        """Return the position of a infrastructure"""
        return self._position

    def action(self):
        """Trigger actions of a infrastructure"""

        logging.info("%d was triggered" % self.iid)
        pass  # this infrastructure has no application running
