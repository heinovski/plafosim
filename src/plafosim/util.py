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

import numpy as np
import pandas as pd


def assert_index_equal(a, b):
    """
    Ensures the indeces of two Sequences/DataFrames are equal.

    Parameters
    ----------
    a : pandas.Sequence / pandas.DataFrame
    b : pandas.Sequence / pandas.DataFrame
    """

    assert(list(a.index) == list(b.index))


def speed2distance(speed: float, time_interval: float = 1.0) -> float:
    """
    Converts a driving speed to a distance driven within a given time interval.

    Parameters
    ----------
    speed : float
        The speed to be converted
    time_interval : float, optional
        The time to consider
    """

    return speed * time_interval


def distance2speed(distance: float, time_interval: float = 1.0) -> float:
    """
    Converts a driven distance to a driving speed for a given time interval.

    Parameters
    ----------
    distance : float
        The distance to be converted
    time_interval : float, optional
        The time to consider
    """

    return distance / time_interval


def acceleration2speed(acceleration: float, time_interval: float = 1.0) -> float:
    """
    Converts an acceleration to a driving speed for a given time interval.

    Parameters
    ----------
    acceleration : float
        The acceleration to be converted
    time_interval : float, optional
        The time to consider
    """

    return acceleration * time_interval


def speed2acceleration(speed_from: float, speed_to: float, time_interval: float = 1.0) -> float:
    """
    Converts a speed range to an acceleration within a given time interval.

    Parameters
    ----------
    speed_from : float
        The initial speed
    speed_to : float
        The target speed
    time_interval : float, optional
        The time to consider
    """

    return (speed_to - speed_from) / time_interval


def clip_position(position: pd.Series, vdf: pd.DataFrame) -> pd.Series:
    """
    Returns the clipped positions (i.e., by arrival position) of vehicles within a pandas DataFrame.

    Parameters
    ----------
    position : pandas.Series
        The series containing the positions to be clipped
        index: vid
        columns: [position, length, lane, ..]
    vdf : pandas.DataFrame
        The dataframe containing the vehicles as rows
        index: vid
        columns: [position, length, lane, ..]
    """

    assert_index_equal(position, vdf)

    clipped_position = pd.Series(
        np.clip(
            position.values,
            0,
            vdf.arrival_position  # do not move further than arrival position
        ), index=position.index)
    assert_index_equal(clipped_position, position)

    return clipped_position


def update_position(vdf: pd.DataFrame, step_length: int) -> pd.DataFrame:
    """
    Updates the position of vehicles within a pandas DataFrame.

    This is based on Krauss' single lane traffic:
    adjust position (move)
    x(t + step_size) = x(t) + v(t)*step_size

    Parameters
    ----------
    vdf : pandas.DataFrame
        The dataframe containing the vehicles as rows
        index: vid
        columns: [position, length, lane, ..]
    step_length : int
        The length of the simulated step
    """
    position = vdf.position + (vdf.speed * step_length)
    vdf['position'] = clip_position(position, vdf)
    return vdf


def get_crashed_vehicles(vdf: pd.DataFrame) -> list:
    """
    Returns the list of crashed vehicles' ids

    Parameters
    ----------
    vdf : pandas.DataFrame
        The dataframe containing the vehicles as rows
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
