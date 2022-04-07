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

import random


def get_desired_speed(
    desired_speed: float,
    rng: random.Random,
    speed_variation: float,
    min_desired_speed: float,
    max_desired_speed: float,
    random_desired_speed: bool = False,
) -> float:
    """Returns a (random) depart speed."""

    if random_desired_speed:
        # normal distribution
        speed = desired_speed * rng.normalvariate(1.0, speed_variation)
        speed = max(speed, min_desired_speed)
        speed = min(speed, max_desired_speed)
    else:
        speed = desired_speed

    return speed


def get_depart_speed(
    desired_speed: float,
    rng: random.Random,
    depart_desired: bool = False,
    random_depart_speed: bool = False,
) -> float:
    """
    Returns a (random) depart speed.

    Parameters
    ----------
    desired_speed : float
        The desired speed to consider
    """

    if random_depart_speed:
        # make sure to also include the desired speed itself
        depart_speed = rng.randrange(0, desired_speed + 1, 1)
    else:
        depart_speed = 0

    if depart_desired:
        depart_speed = desired_speed

    return depart_speed


def get_arrival_position(
    depart_position: int,
    road_length: int,
    ramp_interval: int,
    min_trip_length: int,
    max_trip_length: int,
    rng: random.Random,
    random_arrival_position: bool = False,
    pre_fill: bool = False,
) -> int:
    """
    Returns a (random) arrival position for a given depart position.

    This considers the ramp interval, road length, and minimum trip length.

    Parameters
    ----------
    depart_position : int
        The depart position to consider
    prefill : bool, optional
        Whether the trip is for a pre-filled vehicle
    """

    # set minimum theoretical arrival position
    if pre_fill:
        # We cannot use the minimum trip time here,
        # since the pre-generation is supposed to produce a snapshot of a realistic simulation.
        # But we can assume that a vehicle has to drive at least 1m
        assert depart_position <= (road_length - 1)
        min_arrival = depart_position + 1
        max_arrival = min(depart_position + max_trip_length - ramp_interval, road_length)
    else:
        # make sure that the vehicles drive at least for the minimum length of a trip
        # and at least for one ramp
        min_arrival = min(depart_position + max(min_trip_length, 1), road_length)
        max_arrival = min(depart_position + max_trip_length, road_length)
    min_arrival_ramp = min_arrival + (ramp_interval - min_arrival) % ramp_interval
    max_arrival_ramp = max_arrival + (ramp_interval - max_arrival) % ramp_interval
    assert min_arrival_ramp >= 0
    assert min_arrival_ramp <= road_length
    assert min_arrival_ramp <= max_arrival_ramp
    assert max_arrival_ramp <= road_length
    if min_arrival % ramp_interval == 0:
        assert min_arrival == min_arrival_ramp

    if random_arrival_position and max_arrival_ramp < road_length:
        # make sure to also include the end of the road itself
        arrival_position = rng.randrange(min_arrival_ramp, max_arrival_ramp + 1, ramp_interval)
        assert arrival_position >= min_arrival_ramp
    else:
        # simply drive until the end of the trip or the road
        arrival_position = min(max_arrival_ramp, road_length)

    assert arrival_position <= road_length
    assert arrival_position > depart_position
    assert arrival_position - depart_position <= max_trip_length

    return arrival_position
