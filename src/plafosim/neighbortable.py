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


class NeighborData:
    """
    Collection of information from a single neighbor to be used within a NeighborTable.

    Neighbor tables are in general not used at the moment.
    """

    def __init__(self,
                 vid: int,
                 originator_id: int,
                 platoon_id: int,
                 leader_id: int,
                 platoon_speed: float,
                 platoon_lane: int,
                 platoon_formation: list,
                 platoon_position_front: float,
                 platoon_position_back: float,
                 timestamp: int,
                 valid: bool = True):
        """Initialize a neighbor data instance"""

        self._vid = vid  # the id of the neighbor
        self._originator_id = originator_id  # the id the originator of this information
        self._platoon_id = platoon_id  # id of the platoon the message corresponds to
        self._leader_id = leader_id  # id of the leader of the corresponding platoon
        self._platoon_speed = platoon_speed  # current speed of the advertised platoon
        self._platoon_lane = platoon_lane  # current lane of the advertised platoon
        self._platoon_formation = platoon_formation  # current formation of the advertised platoon
        # current position of the front of the advertised platoon (front of leader)
        self._platoon_position_front = platoon_position_front
        # current position of the back of the advertised platoon (back of last vehicle)
        self._platoon_position_back = platoon_position_back
        self._timestamp = timestamp  # the time this information was received
        self._valid = valid  # a flag for the validity of the data

    @property
    def vid(self) -> int:
        return self._vid

    @property
    def originator_id(self) -> int:
        return self._originator_id

    @property
    def platoon_id(self) -> int:
        return self._platoon_id

    @property
    def leader_id(self) -> int:
        return self._leader_id

    @property
    def platoon_speed(self) -> float:
        return self._platoon_speed

    @property
    def platoon_lane(self) -> int:
        return self._platoon_lane

    @property
    def platoon_formation(self) -> list:
        return self._platoon_formation

    @property
    def platoon_position_front(self) -> float:
        return self._platoon_position_front

    @property
    def platoon_position_back(self) -> float:
        return self._platoon_position_back

    @property
    def timestamp(self) -> int:
        return self._timestamp

    def is_valid(self) -> bool:
        return self._valid


# TODO some abstract base class and an additional oracle neighbortable
class NeighborTable:
    """
    Collection of information from neighbors, received via communication.

    Neighbor tables are in general not used at the moment.
    """

    def __init__(self, validity_time: int):
        """Initialize a neighbor table instance"""

        self.neighbors = []  # the list of neighbors in the table
        self._validity_time = validity_time  # the time an entry in the table is valid

    def add_neighbor(self, neighbor: NeighborData):
        """Add a neighbor to the neighbor table"""
        self._neighbors.append({neighbor.vid: neighbor})

    def remove_neighbor(self, neighbor_id: int) -> bool:
        """Remove a neighbor from the neighbor table"""
        self._vehicles.remove()

    def update_neighbor(self, neighbor: NeighborData):
        """Update a neighbor in the neighbor table"""
        return self.add_neighbor(neighbor)  # TODO fix dict

    def get_neighbor(self, neighbor_id: int) -> NeighborData:
        """Return a neighbor entry with the given id from the table"""
        pass

    def is_in_table(self, neighbor_id: int) -> bool:
        """Return whether a neighbor with the given id is in the neighbor table"""
        pass

    def size(self) -> int:
        """Return the size of the neighbor table"""
        return len(self._neighbors)

    def invalidate_entries(self, current_step: int):
        """Invalidate entries in the neighbor table that exceeded the validity time"""
        pass

    def number_of_valid_entries(self) -> int:
        pass

    @property
    def valid_entries(self) -> list:
        pass
