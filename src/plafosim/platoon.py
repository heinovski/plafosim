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

from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .platooning_vehicle import PlatooningVehicle  # noqa 401


class Platoon:
    """A collection of parameters for a concrete platoon"""

    def __init__(
            self,
            platoon_id: int,
            formation: list,
            desired_speed: float):
        self._platoon_id = platoon_id  # the id of the platoon
        #TODO convert to dict?
        self._formation = formation  # the current formation of the platoon
        self._desired_speed = desired_speed  # the current (desired) speed of the platoon

    @property
    def platoon_id(self) -> int:
        return self._platoon_id

    @property
    def leader(self) -> 'PlatooningVehicle':
        return self.formation[0]

    @property
    def last(self) -> 'PlatooningVehicle':
        return self.formation[-1]

    @property
    def formation(self) -> list:
        return self._formation

    @property
    def member_ids(self) -> list:
        return [x.vid for x in self.formation]

    @property
    def desired_speed(self) -> float:
        return self._desired_speed

    @property
    def speed(self) -> float:
        # HACK for keeping the speed up to date
        return self.leader.speed

    @property
    def lane(self) -> int:
        # HACK for keeping the lane up to date
        return self.leader.lane

    @property
    def max_speed(self) -> float:
        return min(self.formation, key=lambda x: x.max_speed).max_speed

    @property
    def max_acceleration(self) -> float:
        return min(self.formation, key=lambda x: x.max_acceleration).max_acceleration

    @property
    def max_deceleration(self) -> float:
        return min(self.formation, key=lambda x: x.max_deceleration).max_deceleration

    @property
    def length(self) -> int:
        return len(self.formation)

    @property
    def position(self) -> int:
        return self.leader.position

    @property
    def rear_position(self) -> int:
        return self.last.rear_position

    def get_members(self) -> list:
        return [vehicle.vid for vehicle in self.formation]

    def get_member_index(self, vehicle: 'PlatooningVehicle') -> int:
        return self.formation.index(vehicle)

    def get_front(self, vehicle: 'PlatooningVehicle'):
        if vehicle is not self.leader:
            return self.formation[self.get_member_index(vehicle) - 1]
        else:
            return None

    def __str__(self) -> str:
        return f"platoon {self.platoon_id}: {self.member_ids}"
