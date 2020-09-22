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


class VehicleType:
    """A collection of parameters for a concrete vehicle type"""

    def __init__(self, name: str, length: int, max_speed: float, max_acceleration: float, max_deceleration: float, min_gap: float):
        """Initializes a vehicle type"""
        self._name = name  # the name of a vehicle type
        self._length = length  # the length of a vehicle type
        self._max_speed = max_speed  # the maximum speed of a vehicle type
        self._max_acceleration = max_acceleration  # the maximum acceleration of the vehicle type
        self._max_deceleration = max_deceleration  # the maximum deceleration of the vehicle type
        self._min_gap = min_gap  # the minimum gap to the vehicle in front

    @property
    def name(self) -> str:
        """Returns the name of a vehicle type"""
        return self._name

    @property
    def length(self) -> int:
        """Returns the length of a vehicle type"""
        return self._length

    @property
    def max_speed(self) -> float:
        """Returns the maximum speed of a vehicle type"""
        return self._max_speed

    @property
    def max_acceleration(self) -> float:
        """Returns the maximum acceleration of a vehicle type"""
        return self._max_acceleration

    @property
    def max_deceleration(self) -> float:
        """Returns the maximum deceleration of a vehicle type"""
        return self._max_deceleration

    @property
    def min_gap(self) -> float:
        """Returns the minimum gap of a vehicle type"""
        return self._min_gap
