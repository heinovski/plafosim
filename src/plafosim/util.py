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


def speed2distance(speed: float, time_interval: float = 1.0) -> float:
    return speed * time_interval


def distance2speed(distance: float, time_interval: float = 1.0) -> float:
    return distance / time_interval


def acceleration2speed(acceleration: float, time_interval: float = 1.0) -> float:
    return acceleration * time_interval


def speed2acceleration(speed_from: float, speed_to: float, time_interval: float = 1.0) -> float:
    return (speed_to - speed_from) / time_interval
