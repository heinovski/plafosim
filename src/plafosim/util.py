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

import pandas as pd


def speed2distance(speed: float, time_interval: float = 1.0) -> float:
    return speed * time_interval


def distance2speed(distance: float, time_interval: float = 1.0) -> float:
    return distance / time_interval


def acceleration2speed(acceleration: float, time_interval: float = 1.0) -> float:
    return acceleration * time_interval


def speed2acceleration(speed_from: float, speed_to: float, time_interval: float = 1.0) -> float:
    return (speed_to - speed_from) / time_interval


def get_crashed_vehicles(vdf: pd.DataFrame) -> list:
    """
    index: vid
    columns: [position, length, lane, ..]
    """

    # sort vehicles by their position
    vdf = vdf.sort_values('position', ascending=False)

    # add rear_position
    vdf['rear_position'] = vdf.position - vdf.length

    # filter out lanes with 1 vehicle
    lane_mask = (vdf.groupby('lane').size() > 1)
    lanes_to_keep = lane_mask[lane_mask].index.values
    if len(lanes_to_keep) == 0:
        return []
    vdf = vdf[vdf.lane.isin(list(lanes_to_keep))]

    # TODO should we just use the precessor / successor

    # calculate vehicles with a crash in their back
    groupby = vdf.groupby('lane')
    crash_in_back = (
        # the car behind me touches my back
        groupby['position'].shift(-1) >= groupby['rear_position'].shift(0)
    )
    # calculate vehicles with a crash in their front
    crash_in_front = (
        # I touch the back of the car in front of me
        groupby['position'].shift(0) >= groupby['rear_position'].shift(1)
    )

    # calculate vehicles with a crash in the back or the front
    # TODO one check might be enough
    crash = (crash_in_back | crash_in_front)

    return list(crash[crash].index.values)
