#
# Copyright (c) 2020-2025 Julian Heinovski <heinovski@ccs-labs.org>
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

from plafosim.emissions import EmissionClass


class VehicleType:
    """
    A collection of parameters for a concrete vehicle type.
    """

    def __init__(
            self, name: str,
            length: int,
            max_speed: float,
            max_acceleration: float,
            max_deceleration: float,
            min_gap: float,
            headway_time: float,
            emission_class: str,
    ):
        """
        Initialize a specific vehicle type.

        Parameters
        ----------
        name : str
            The name of the vehicle type
        length : int
            The length of the vehicle type
        max_speed : float
            The maximum speed of the vehicle type
        max_acceleration : float
            The maximum acceleration of the vehicle type
        max_deceleration : float
            The maximum deceleration of the vehicle type
        min_gap : float
            The minimum safety gap to the vehicle in front of the vehicle type
        headway_time : float
            The human headway time of the vehicle type
        emission_class : EmissionClass
            The emission class of the vehicle type
        """

        self._name = name  # the name of a vehicle type
        self._length = length  # the length of a vehicle type
        self._max_speed = max_speed  # the maximum speed of a vehicle type
        self._max_acceleration = max_acceleration  # the maximum acceleration of the vehicle type
        self._max_deceleration = max_deceleration  # the maximum deceleration of the vehicle type
        self._min_gap = min_gap  # the minimum gap to the vehicle in front
        self._headway_time = headway_time  # the desired human headway time
        self._emission_class = EmissionClass[emission_class]  # the emission class of the vehicle type

    @property
    def name(self) -> str:
        """
        Return the name of a vehicle type.
        """

        return self._name

    @property
    def length(self) -> int:
        """
        Return the length of a vehicle type.
        """

        return self._length

    @property
    def max_speed(self) -> float:
        """
        Return the maximum speed of a vehicle type.
        """

        return self._max_speed

    @property
    def max_acceleration(self) -> float:
        """
        Return the maximum acceleration of a vehicle type.
        """

        return self._max_acceleration

    @property
    def max_deceleration(self) -> float:
        """
        Return the maximum deceleration of a vehicle type.
        """

        return self._max_deceleration

    @property
    def min_gap(self) -> float:
        """
        Return the minimum gap of a vehicle type.
        """

        return self._min_gap

    @property
    def headway_time(self) -> float:
        """
        Return the desired human headway time of a vehicle type.
        """

        return self._headway_time

    @property
    def emission_class(self) -> EmissionClass:
        """
        Return the emission class of a vehicle type.
        """

        return self._emission_class

    @property
    def emission_factors(self) -> dict:
        """
        Return the emission factors of a vehicle type.
        """

        return self._emission_class.emission_factors

    def __str__(self) -> str:
        """
        Return the str representation of a vehicle type.
        """

        return str(self.__dict__)
