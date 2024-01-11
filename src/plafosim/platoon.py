#
# Copyright (c) 2020-2024 Julian Heinovski <heinovski@ccs-labs.org>
#
# SPDX-License-Identifier: GPL-3.0-or-later
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
    from plafosim.platooning_vehicle import PlatooningVehicle  # noqa 401

LOG = logging.getLogger(__name__)


class Platoon:
    """
    A collection of parameters for a specific platoon.
    """

    def __init__(
        self,
        platoon_id: int,
        formation: list,
        desired_speed: float,
    ):
        """
        Initialize a platoon instance.

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
        """
        Return the id of the platoon.
        """

        return self._platoon_id

    @property
    def leader(self) -> 'PlatooningVehicle':
        """
        Return the leading PlatoonVehicle of the platoon.
        """

        return self._formation[0]

    @property
    def last(self) -> 'PlatooningVehicle':
        """
        Return the last PlatooningVehicle of the platoon.
        """

        return self._formation[-1]

    @property
    def formation(self) -> list:
        """
        Return the complete formation of the platoon.
        """

        return self._formation

    @property
    def member_ids(self) -> list:
        """
        Return the ids of all platoon members.
        """

        return [x._vid for x in self._formation]

    @property
    def desired_speed(self) -> float:
        """
        Return the desired driving speed of the platoon.
        """

        return self._desired_speed

    @property
    def speed(self) -> float:
        """
        Return the current driving speed of the platoon.
        """

        # HACK for keeping the speed up to date
        return self.leader.speed

    @property
    def lane(self) -> int:
        """
        Return the current lane of the platoon.
        """

        # HACK for keeping the lane up to date
        return self.leader.lane

    @property
    def max_speed(self) -> float:
        """
        Return the maximum speed of the platoon.

        The maximum speed is based on the slowest vehicle within the platoon.
        """

        return self._max_speed

    def update_max_speed(self):
        """
        Update the maximum speed of the platoon.

        The maximum speed is based on the slowest vehicle within the platoon.
        """

        self._max_speed = min(v.max_speed for v in self._formation)

    @property
    def max_acceleration(self) -> float:
        """
        Return the maximum acceleration of the platoon.

        The maximum acceleration is based on the slowest vehicle within the platoon.
        """

        return self._max_acceleration

    def update_max_acceleration(self):
        """
        Update the maximum acceleration of the platoon.

        The maximum acceleration is based on the slowest vehicle within the platoon.
        """

        self._max_acceleration = min(v.max_acceleration for v in self._formation)

    @property
    def max_deceleration(self) -> float:
        """
        Return the maximum deceleration of the platoon.

        The maximum deceleration is based on the slowest vehicle within the platoon.
        """

        return self._max_deceleration

    def update_max_deceleration(self):
        """
        Update the maximum deceleration of the platoon.

        The maximum deceleration is based on the slowest vehicle within the platoon.
        """

        self._max_deceleration = min(v.max_deceleration for v in self._formation)

    @property
    def size(self) -> int:
        """
        Return the size of the platoon.
        """

        return len(self._formation)

    @property
    def position(self) -> int:
        """
        Return the current position of the platoon.
        """

        # HACK for keeping the position up to date
        return self.leader.position

    @property
    def rear_position(self) -> int:
        """
        Return the current rear position of the platoon.
        """

        # HACK for keeping the rear position up to date
        return self.last.rear_position

    @property
    def length(self) -> float:
        """
        Return the length of the platoon.
        """

        return self.position - self.rear_position

    def get_member_index(self, vehicle: 'PlatooningVehicle') -> int:
        """
        Return the index of a member within the platoon.

        Parameters
        ----------
        vehicle : PlatooningVehicle
            The considered vehicle within the platoon.

        Returns
        -------
        int : The index of the member within the platoon
        """

        return self._formation.index(vehicle)

    def get_front(self, vehicle: 'PlatooningVehicle'):
        """
        Return the PlatooningVehicle in the front.

        Parameters
        ----------
        vehicle : PlatooningVehicle
            The considered vehicle within the platoon.

        Returns
        -------
        PlatooningVehicle : The member in the front
        """

        if vehicle is not self.leader:
            return self._formation[self.get_member_index(vehicle) - 1]
        return None

    def get_back(self, vehicle: 'PlatooningVehicle'):
        """
        Return the PlatooningVehicle in the back.

        Parameters
        ----------
        vehicle : PlatooningVehicle
            The considered vehicle within the platoon.

        Returns
        -------
        PlatooningVehicle : The member in the back
        """

        if vehicle is not self.last:
            return self._formation[self.get_member_index(vehicle) + 1]
        return None

    def update_desired_speed(self):
        """
        Update the desired driving speed of the platoon.

        This is based on the desired driving speed of all members.
        """

        old_desired_speed = self._desired_speed
        self._desired_speed = min(mean([v._desired_speed for v in self._formation]), self.max_speed)
        LOG.debug(f"Updated platoon {self.platoon_id}'s desired speed to {self.desired_speed} (from {old_desired_speed})")

    def update_limits(self):
        """
        Update mobility limits for the platoon.
        """

        self.update_max_speed()
        self.update_max_acceleration()
        self.update_max_deceleration()

    def update_cf_target_speed(self):
        """
        Update the cf target speed for the platoon.
        """

        for member in self._formation:
            member._cf_target_speed = self._desired_speed

    def __str__(self) -> str:
        """
        Return the str representation of the platoon.
        """

        return f"{self._platoon_id}: {self.member_ids}"
