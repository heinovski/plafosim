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

import logging
from statistics import mean
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .platooning_vehicle import PlatooningVehicle  # noqa 401

LOG = logging.getLogger(__name__)


class Platoon:
    """A collection of parameters for a specific platoon."""

    def __init__(
            self,
            platoon_id: int,
            formation: list,
            desired_speed: float):
        """
        Parameters
        ----------
        platoon_id : int
            The id of the platoon
        formation : list
            The list of PlatooningVehicles within the platoon
        desired_speed : float
            The platoon's desired driving speed
        """

        self._platoon_id = platoon_id  # the id of the platoon
        #TODO convert to dict?
        self._formation = formation  # the current formation of the platoon
        self._desired_speed = desired_speed  # the current (desired) speed of the platoon
        self._max_speed = None
        self.update_max_speed()
        self._max_acceleration = None
        self.update_max_acceleration()
        self._max_deceleration = None
        self.update_max_deceleration()

    @property
    def platoon_id(self) -> int:
        """Returns the id of the platoon."""

        return self._platoon_id

    @property
    def leader(self) -> 'PlatooningVehicle':
        """Returns the leading PlatoonVehicle of the platoon."""

        return self._formation[0]

    @property
    def last(self) -> 'PlatooningVehicle':
        """Returns the last PlatooningVehicle of the platoon."""

        return self._formation[-1]

    @property
    def formation(self) -> list:
        """Returns the complete formation of the platoon."""

        return self._formation

    @property
    def member_ids(self) -> list:
        """Returns the ids of all platoon members."""

        return [x._vid for x in self._formation]

    @property
    def desired_speed(self) -> float:
        """Returns the desired driving speed of the platoon."""

        return self._desired_speed

    @property
    def speed(self) -> float:
        """Returns the current driving speed of the platoon."""

        # HACK for keeping the speed up to date
        return self.leader.speed

    @property
    def lane(self) -> int:
        """Returns the current lane of the platoon."""

        # HACK for keeping the lane up to date
        return self.leader.lane

    @property
    def max_speed(self) -> float:
        """
        Returns the maximum speed of the platoon.

        The maximum speed is based on the slowest vehicle within the platoon.
        """

        return self._max_speed

    def update_max_speed(self):
        """
        Updates the maximum speed of the platoon.

        The maximum speed is based on the slowest vehicle within the platoon.
        """

        self._max_speed = min(v.max_speed for v in self._formation)

    @property
    def max_acceleration(self) -> float:
        """
        Returns the maximum acceleration of the platoon.

        The maximum acceleration is based on the slowest vehicle within the platoon.
        """

        return self._max_acceleration

    def update_max_acceleration(self):
        """
        Updates the maximum acceleration of the platoon.

        The maximum acceleration is based on the slowest vehicle within the platoon.
        """

        self._max_acceleration = min(v.max_acceleration for v in self._formation)

    @property
    def max_deceleration(self) -> float:
        """
        Returns the maximum deceleration of the platoon.

        The maximum deceleration is based on the slowest vehicle within the platoon.
        """

        return self._max_deceleration

    def update_max_deceleration(self):
        """
        Updates the maximum deceleration of the platoon.

        The maximum deceleration is based on the slowest vehicle within the platoon.
        """

        self._max_deceleration = min(v.max_deceleration for v in self._formation)

    @property
    def size(self) -> int:
        """Returns the size of the platoon."""

        return len(self._formation)

    @property
    def position(self) -> int:
        """Returns the current position of the platoon."""

        # HACK for keeping the position up to date
        return self.leader.position

    @property
    def rear_position(self) -> int:
        """Returns the current rear position of the platoon."""

        # HACK for keeping the rear position up to date
        return self.last.rear_position

    @property
    def length(self) -> float:
        """Returns the length of the platoon."""

        return self.position - self.rear_position

    def get_member_index(self, vehicle: 'PlatooningVehicle') -> int:
        """
        Returns the index of a member within the platoon.

        Parameters
        ----------
        vehicle : PlatooningVehicle
            The considered vehicle within the platoon.
        """

        return self._formation.index(vehicle)

    def get_front(self, vehicle: 'PlatooningVehicle'):
        """
        Returns the PlatooningVehicle in the front.

        Parameters
        ----------
        vehicle : PlatooningVehicle
            The considered vehicle within the platoon.
        """

        if vehicle is not self.leader:
            return self._formation[self.get_member_index(vehicle) - 1]
        else:
            return None

    def get_back(self, vehicle: 'PlatooningVehicle'):
        """
        Returns the PlatooningVehicle in the back.

        Parameters
        ----------
        vehicle : PlatooningVehicle
            The considered vehicle within the platoon.
        """

        if vehicle is not self.last:
            return self._formation[self.get_member_index(vehicle) + 1]
        else:
            return None

    def update_desired_speed(self):
        """
        Updates the desired driving speed of the platoon.

        This is based on the desired driving speed of all members.
        """

        old_desired_speed = self._desired_speed
        self._desired_speed = min(mean([v._desired_speed for v in self._formation]), self.max_speed)
        LOG.debug(f"Updated platoon {self.platoon_id}'s desired speed to {self.desired_speed} (from {old_desired_speed})")

    def update_limits(self):
        self.update_max_speed()
        self.update_max_acceleration()
        self.update_max_deceleration()

    def __str__(self) -> str:
        """Returns the str representation of the platoon."""

        return f"{self._platoon_id}: {self.member_ids}"
