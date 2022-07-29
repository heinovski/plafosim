#
# Copyright (c) 2020-2022 Julian Heinovski <heinovski@ccs-labs.org>
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

import random

import pandas as pd

from plafosim.simulator import compute_vehicle_spawns, vtype
from plafosim.spawning import get_arrival_position, get_depart_speed, get_desired_speed


def test_single_ramp_empty_road():
    """
    A vehicle can be inserted into a completely empty road of enough length.
    """

    road_length = 1000
    ramp_interval = 1000
    ramp_positions = list(range(0, road_length + 1, ramp_interval))
    current_step = 0
    trip_length = road_length

    assert len(ramp_positions) == 2

    vid = 0
    desired_speed = 36
    depart_speed = desired_speed
    schedule_time = current_step

    vdf = pd.DataFrame(
        {
            'vid': [],
            'speed': [],
            'position': [],
            'lane': [],
        }
    )
    new_vehicles = [
        {
            'vid': vid,
            'desired_speed': desired_speed,
            'depart_speed': depart_speed,
            'schedule_time': schedule_time,
            'min_trip_length': trip_length,
            'max_trip_length': trip_length,
        }
    ]

    spawned_vehicles_df, not_spawned_vehicles = compute_vehicle_spawns(
        vehicles=new_vehicles,
        vdf=vdf,
        ramp_positions=ramp_positions,
        current_step=current_step,
        rng=random,
        random_depart_position=False,
        random_arrival_position=False,
    )
    # TODO add same test without random depart position

    # vehicle could be inserted
    assert len(spawned_vehicles_df) == 1
    assert not not_spawned_vehicles

    v = spawned_vehicles_df.iloc[0]
    assert v.vid == vid
    assert v.depart_position == ramp_positions[0] + vtype.length
    assert v.position == ramp_positions[0] + vtype.length
    assert v.depart_lane == 0
    assert v.lane == 0
    assert v.depart_speed == depart_speed
    assert v.speed == depart_speed
    assert v.depart_time == current_step
    assert v.depart_delay == current_step - schedule_time


def test_single_ramp_existing_vehicle():
    """
    A vehicle can be inserted into a road with enough length because another vehicle is far enough away from the spawn position.
    """

    road_length = 1000
    ramp_interval = 1000
    ramp_positions = list(range(0, road_length + 1, ramp_interval))
    current_step = 0
    trip_length = road_length

    assert len(ramp_positions) == 2

    vid = 1
    desired_speed = 36
    depart_speed = desired_speed
    schedule_time = current_step

    vdf = pd.DataFrame(
        {
            'vid': [0],
            'speed': [36],
            'position': [50],
            'lane': [0],
        }
    )
    new_vehicles = [
        {
            'vid': vid,
            'desired_speed': desired_speed,
            'depart_speed': depart_speed,
            'schedule_time': schedule_time,
            'min_trip_length': trip_length,
            'max_trip_length': trip_length,
        }
    ]

    spawned_vehicles_df, not_spawned_vehicles = compute_vehicle_spawns(
        vehicles=new_vehicles,
        vdf=vdf,
        ramp_positions=ramp_positions,
        current_step=current_step,
        rng=random,
        random_depart_position=False,
        random_arrival_position=False,
    )

    # vehicle could be inserted
    assert len(spawned_vehicles_df) == 1
    assert not not_spawned_vehicles

    v = spawned_vehicles_df.iloc[0]
    assert v.vid == vid
    assert v.depart_position == ramp_positions[0] + vtype.length
    assert v.position == ramp_positions[0] + vtype.length
    assert v.depart_lane == 0
    assert v.lane == 0
    assert v.depart_speed == depart_speed
    assert v.speed == depart_speed
    assert v.depart_time == current_step
    assert v.depart_delay == current_step - schedule_time


def test_single_ramp_blocked():
    """
    A vehicle can not be inserted into a road with enough length because another vehicle is blocking the spawn position.
    """

    road_length = 1000
    ramp_interval = 1000
    ramp_positions = list(range(0, road_length + 1, ramp_interval))
    current_step = 0
    trip_length = road_length

    assert len(ramp_positions) == 2

    vid = 1
    desired_speed = 36
    depart_speed = desired_speed
    schedule_time = current_step

    vdf = pd.DataFrame(
        {
            'vid': [0],
            'speed': [36],
            'position': [10],
            'lane': [0],
        }
    )
    new_vehicles = [
        {
            'vid': vid,
            'desired_speed': desired_speed,
            'depart_speed': depart_speed,
            'schedule_time': schedule_time,
            'min_trip_length': trip_length,
            'max_trip_length': trip_length,
        }
    ]

    spawned_vehicles_df, not_spawned_vehicles = compute_vehicle_spawns(
        vehicles=new_vehicles,
        vdf=vdf,
        ramp_positions=ramp_positions,
        current_step=current_step,
        rng=random,
        random_depart_position=False,
        random_arrival_position=False,
    )

    # vehicle could not be inserted
    assert spawned_vehicles_df.empty
    assert len(not_spawned_vehicles) == 1
    assert not_spawned_vehicles[0]['vid'] == vid


def test_single_ramp_blocked_exact():
    """
    A vehicle can not be inserted into a road with enough length because another vehicle is blocking the spawn position.
    """

    road_length = 1000
    ramp_interval = 1000
    ramp_positions = list(range(0, road_length + 1, ramp_interval))
    current_step = 0
    trip_length = ramp_interval

    assert len(ramp_positions) == 2

    vid = 1
    desired_speed = 36
    depart_speed = desired_speed
    schedule_time = current_step

    vdf = pd.DataFrame(
        {
            'vid': [0],
            'speed': [36],
            'position': [vtype.length],
            'lane': [0],
        }
    )
    new_vehicles = [
        {
            'vid': vid,
            'desired_speed': desired_speed,
            'depart_speed': depart_speed,
            'schedule_time': schedule_time,
            'min_trip_length': trip_length,
            'max_trip_length': trip_length,
        }
    ]

    spawned_vehicles_df, not_spawned_vehicles = compute_vehicle_spawns(
        vehicles=new_vehicles,
        vdf=vdf,
        ramp_positions=ramp_positions,
        current_step=current_step,
        rng=random,
        random_depart_position=False,
        random_arrival_position=False,
    )

    # vehicle could not be inserted
    assert spawned_vehicles_df.empty
    assert len(not_spawned_vehicles) == 1
    assert not_spawned_vehicles[0]['vid'] == vid


def test_first_ramp_blocked():
    """
    A vehicle can be inserted at the second ramp since the first is blocked by another vehicle.
    """

    road_length = 1000
    ramp_interval = 500
    ramp_positions = list(range(0, road_length + 1, ramp_interval))
    current_step = 0
    trip_length = ramp_interval

    assert len(ramp_positions) == 3

    vid = 1
    desired_speed = 36
    depart_speed = desired_speed
    schedule_time = current_step

    vdf = pd.DataFrame(
        {
            'vid': [0],
            'speed': [36],
            'position': [10],
            'lane': [0],
        }
    )
    new_vehicles = [
        {
            'vid': vid,
            'desired_speed': desired_speed,
            'depart_speed': depart_speed,
            'schedule_time': schedule_time,
            'min_trip_length': trip_length,
            'max_trip_length': trip_length,
        }
    ]

    spawned_vehicles_df, not_spawned_vehicles = compute_vehicle_spawns(
        vehicles=new_vehicles,
        vdf=vdf,
        ramp_positions=ramp_positions,
        current_step=current_step,
        rng=random,
        random_depart_position=True,
        random_arrival_position=False,
    )
    # TODO add same test without random depart position

    # vehicle could be inserted
    assert len(spawned_vehicles_df) == 1
    assert not not_spawned_vehicles

    v = spawned_vehicles_df.iloc[0]
    assert v.vid == vid
    assert v.depart_position == ramp_positions[1] + vtype.length
    assert v.position == ramp_positions[1] + vtype.length
    assert v.depart_lane == 0
    assert v.lane == 0
    assert v.depart_speed == depart_speed
    assert v.speed == depart_speed
    assert v.depart_time == current_step
    assert v.depart_delay == current_step - schedule_time


def test_multiple_single_ramp_empty_road():
    """
    Two vehicles should inserted into an empty road with a single ramp.
    Only one (i.e., the first) vehicle can be inserted.
    """

    road_length = 1000
    ramp_interval = 1000
    ramp_positions = list(range(0, road_length + 1, ramp_interval))
    current_step = 0
    trip_length = ramp_interval

    assert len(ramp_positions) == 2

    ids = [0, 1]
    desired_speeds = [36] * 2
    depart_speeds = desired_speeds
    schedule_times = [current_step] * 2

    vdf = pd.DataFrame(
        {
            'vid': [],
            'speed': [],
            'position': [],
            'lane': [],
        }
    )
    new_vehicles = [
        {
            'vid': ids[0],
            'desired_speed': desired_speeds[0],
            'depart_speed': depart_speeds[0],
            'schedule_time': schedule_times[0],
            'min_trip_length': trip_length,
            'max_trip_length': trip_length,
        },
        {
            'vid': ids[1],
            'desired_speed': desired_speeds[1],
            'depart_speed': depart_speeds[1],
            'schedule_time': schedule_times[1],
            'min_trip_length': trip_length,
            'max_trip_length': trip_length,
        },
    ]

    spawned_vehicles_df, not_spawned_vehicles = compute_vehicle_spawns(
        vehicles=new_vehicles,
        vdf=vdf,
        ramp_positions=ramp_positions,
        current_step=current_step,
        rng=random,
        random_depart_position=True,
        random_arrival_position=False
    )

    # 1 vehicle could be inserted, 1 vehicle could not be inserted
    assert len(spawned_vehicles_df) == 1
    assert len(not_spawned_vehicles) == 1
    assert not_spawned_vehicles[0]['vid'] == ids[1]

    v = spawned_vehicles_df.iloc[0]
    assert v.vid == ids[0]
    assert v.depart_position == ramp_positions[0] + vtype.length
    assert v.position == ramp_positions[0] + vtype.length
    assert v.depart_lane == 0
    assert v.lane == 0
    assert v.depart_speed == depart_speeds[0]
    assert v.speed == depart_speeds[0]
    assert v.depart_time == current_step
    assert v.depart_delay == current_step - schedule_times[0]


def test_multiple_first_ramp_blocked():
    """
    Two vehicles should inserted into a road with two ramps, where the first one is blocked by another vehicle.
    Only one (i.e., the first) vehicle can be inserted at the second ramp.
    """

    road_length = 1000
    ramp_interval = 500
    ramp_positions = list(range(0, road_length + 1, ramp_interval))
    current_step = 0
    trip_length = ramp_interval

    assert len(ramp_positions) == 3

    ids = [1, 2]
    desired_speeds = [36] * 2
    depart_speeds = desired_speeds
    schedule_times = [current_step] * 2

    vdf = pd.DataFrame(
        {
            'vid': [0],
            'speed': [36],
            'position': [10],
            'lane': [0],
        }
    )
    new_vehicles = [
        {
            'vid': ids[0],
            'desired_speed': desired_speeds[0],
            'depart_speed': depart_speeds[0],
            'schedule_time': schedule_times[0],
            'min_trip_length': trip_length,
            'max_trip_length': trip_length,
        },
        {
            'vid': ids[1],
            'desired_speed': desired_speeds[1],
            'depart_speed': depart_speeds[1],
            'schedule_time': schedule_times[1],
            'min_trip_length': trip_length,
            'max_trip_length': trip_length,
        },
    ]

    spawned_vehicles_df, not_spawned_vehicles = compute_vehicle_spawns(
        vehicles=new_vehicles,
        vdf=vdf,
        ramp_positions=ramp_positions,
        current_step=current_step,
        rng=random,
        random_depart_position=True,
        random_arrival_position=False,
    )
    # TODO add same test without random depart position

    # 1 vehicle could be inserted, 1 vehicle could not be inserted
    assert len(spawned_vehicles_df) == 1
    assert len(not_spawned_vehicles) == 1
    assert not_spawned_vehicles[0]['vid'] == ids[1]

    v = spawned_vehicles_df.iloc[0]
    assert v.vid == ids[0]
    assert v.depart_position == ramp_positions[1] + vtype.length
    assert v.position == ramp_positions[1] + vtype.length
    assert v.depart_lane == 0
    assert v.lane == 0
    assert v.depart_speed == depart_speeds[0]
    assert v.speed == depart_speeds[0]
    assert v.depart_time == current_step
    assert v.depart_delay == current_step - schedule_times[0]


def test_multiple_single_ramp_empty_road_priority():
    """
    Two vehicles should be inserted into an empty road with a single ramp.
    Only one vehicle can be inserted.
    The vehicle with the longest waiting time (i.e., vehicle two) is inserted.
    """

    road_length = 1000
    ramp_interval = 1000
    ramp_positions = list(range(0, road_length + 1, ramp_interval))
    current_step = 1
    trip_length = ramp_interval

    assert len(ramp_positions) == 2

    ids = [0, 1]
    desired_speeds = [36] * 2
    depart_speeds = desired_speeds
    schedule_times = [current_step, current_step - 1]

    vdf = pd.DataFrame(
        {
            'vid': [],
            'speed': [],
            'position': [],
            'lane': [],
        }
    )
    new_vehicles = [
        {
            'vid': ids[0],
            'desired_speed': desired_speeds[0],
            'depart_speed': depart_speeds[0],
            'schedule_time': schedule_times[0],
            'min_trip_length': trip_length,
            'max_trip_length': trip_length,
        },
        {
            'vid': ids[1],
            'desired_speed': desired_speeds[1],
            'depart_speed': depart_speeds[1],
            'schedule_time': schedule_times[1],
            'min_trip_length': trip_length,
            'max_trip_length': trip_length,
        },
    ]

    spawned_vehicles_df, not_spawned_vehicles = compute_vehicle_spawns(
        vehicles=sorted(new_vehicles, key=lambda d: d['schedule_time']),
        vdf=vdf,
        ramp_positions=ramp_positions,
        current_step=current_step,
        rng=random,
        random_depart_position=False,
        random_arrival_position=False,
    )

    # 1 vehicle could be inserted, 1 vehicle could not be inserted
    assert len(spawned_vehicles_df) == 1
    assert len(not_spawned_vehicles) == 1
    assert spawned_vehicles_df.iloc[0].vid == ids[1]
    assert not_spawned_vehicles[0]['vid'] == ids[0]

    v = spawned_vehicles_df.iloc[0]
    assert v.vid == ids[1]
    assert v.depart_position == ramp_positions[0] + vtype.length
    assert v.position == ramp_positions[0] + vtype.length
    assert v.depart_lane == 0
    assert v.lane == 0
    assert v.depart_speed == depart_speeds[1]
    assert v.speed == depart_speeds[1]
    assert v.depart_time == current_step
    assert v.depart_delay == current_step - schedule_times[1]


def test_multiple_vehicle_mutiple_ramps_empty_road():
    """
    Two vehicles should be inserted into an empty road with two ramps.
    Both can be inserted, one at each ramp.
    """

    road_length = 1000
    ramp_interval = 500
    ramp_positions = list(range(0, road_length + 1, ramp_interval))
    current_step = 0
    trip_length = ramp_interval

    assert len(ramp_positions) == 3

    ids = [0, 1]
    desired_speeds = [36] * 2
    depart_speeds = desired_speeds
    schedule_times = [current_step] * 2

    vdf = pd.DataFrame(
        {
            'vid': [],
            'speed': [],
            'position': [],
            'lane': [],
        }
    )
    new_vehicles = [
        {
            'vid': ids[0],
            'desired_speed': desired_speeds[0],
            'depart_speed': depart_speeds[0],
            'schedule_time': schedule_times[0],
            'min_trip_length': trip_length,
            'max_trip_length': trip_length,
        },
        {
            'vid': ids[1],
            'desired_speed': desired_speeds[1],
            'depart_speed': depart_speeds[1],
            'schedule_time': schedule_times[1],
            'min_trip_length': trip_length,
            'max_trip_length': trip_length,
        }
    ]

    spawned_vehicles_df, not_spawned_vehicles = compute_vehicle_spawns(
        vehicles=new_vehicles,
        vdf=vdf,
        ramp_positions=ramp_positions,
        current_step=current_step,
        rng=random,
        random_depart_position=True,
        random_arrival_position=False,
    )
    # TODO add same test without random depart position

    # 2 vehicle could be inserted, 1 at each ramp
    assert len(spawned_vehicles_df) == 2
    assert len(not_spawned_vehicles) == 0
    assert (spawned_vehicles_df.vid == ids).all()

    v1 = spawned_vehicles_df.iloc[0]
    assert v1.vid == ids[0]
    assert v1.depart_position - vtype.length in ramp_positions[:-1]
    assert v1.position == v1.depart_position
    assert v1.depart_lane == 0
    assert v1.lane == 0
    assert v1.depart_speed == depart_speeds[0]
    assert v1.speed == depart_speeds[0]
    assert v1.depart_time == current_step
    assert v1.depart_delay == current_step - schedule_times[0]

    v2 = spawned_vehicles_df.iloc[1]
    assert v2.vid == ids[1]
    assert v2.depart_position - vtype.length in ramp_positions[:-1]
    assert v2.depart_position != v1.depart_position
    assert v2.position == v2.depart_position
    assert v2.depart_lane == 0
    assert v2.lane == 0
    assert v2.depart_speed == depart_speeds[1]
    assert v2.speed == depart_speeds[1]
    assert v2.depart_time == current_step
    assert v2.depart_delay == current_step - schedule_times[1]


def test_rear_blocking_vehicle():
    """
    A vehicle should be inserted at a ramp in the middle of the road, the first ramp is blocked by a vehicle, the target ramp is also blocked but indirectly by a rear vehicle.
    Expect: the vehicle cannot be inserted.
    """

    road_length = 1000
    ramp_interval = 500
    ramp_positions = list(range(0, road_length + 1, ramp_interval))
    current_step = 0
    trip_length = ramp_interval

    assert len(ramp_positions) == 3

    vid = 2
    desired_speed = 36
    depart_speed = desired_speed
    schedule_time = current_step

    vdf = pd.DataFrame(
        {
            'vid': [0, 1],
            'speed': [36, 36],
            'position': [10, 490],
            'lane': [0, 0],
        }
    )
    new_vehicles = [
        {
            'vid': vid,
            'desired_speed': desired_speed,
            'depart_speed': depart_speed,
            'schedule_time': schedule_time,
            'min_trip_length': trip_length,
            'max_trip_length': trip_length,
        }
    ]

    spawned_vehicles_df, not_spawned_vehicles = compute_vehicle_spawns(
        vehicles=new_vehicles,
        vdf=vdf,
        ramp_positions=ramp_positions,
        current_step=current_step,
        rng=random,
        random_depart_position=True,
        random_arrival_position=False,
    )
    # TODO add same test without random depart position

    # 1 vehicle could not be inserted
    assert len(spawned_vehicles_df) == 0
    assert len(not_spawned_vehicles) == 1
    assert not_spawned_vehicles[0]['vid'] == vid


def test_single_trip_lengths():
    """
    A single vehicle should be inserted with a random trip.
    """

    random.seed(961)  # triggers some issues

    road_length = 1000
    ramp_interval = 100
    ramp_positions = list(range(0, road_length + 1, ramp_interval))
    current_step = 0
    min_trip_length = 300
    max_trip_length = 800

    vid = 0
    desired_speed = 36
    depart_speed = desired_speed
    schedule_time = current_step

    vdf = pd.DataFrame(
        {
            'vid': [],
            'speed': [],
            'position': [],
            'lane': [],
        }
    )
    new_vehicles = [
        {
            'vid': vid,
            'desired_speed': desired_speed,
            'depart_speed': depart_speed,
            'schedule_time': schedule_time,
            'min_trip_length': min_trip_length,
            'max_trip_length': max_trip_length,
        }
    ]

    spawned_vehicles_df, not_spawned_vehicles = compute_vehicle_spawns(
        vehicles=new_vehicles,
        vdf=vdf,
        ramp_positions=ramp_positions,
        current_step=current_step,
        rng=random,
        random_depart_position=True,
        random_arrival_position=True,
    )
    # TODO add same test without random depart position

    # vehicle could be inserted
    assert len(spawned_vehicles_df) == 1
    assert not not_spawned_vehicles

    v = spawned_vehicles_df.iloc[0]
    assert v.depart_position < road_length
    assert v.depart_position - vtype.length <= road_length - min_trip_length
    assert v.arrival_position >= v.depart_position + min_trip_length - vtype.length
    assert v.arrival_position <= v.depart_position + max_trip_length
    assert v.arrival_position <= road_length
    assert v.depart_position >= 0


def test_get_arrival_position():
    pos = get_arrival_position(
        depart_position=300,
        road_length=1000,
        ramp_interval=100,
        min_trip_length=100,
        max_trip_length=500,
        rng=random,
        random_arrival_position=False,
        pre_fill=False,
    )
    assert pos >= 400

    pos = get_arrival_position(
        depart_position=300,
        road_length=1000,
        ramp_interval=100,
        min_trip_length=100,
        max_trip_length=100,
        rng=random,
        random_arrival_position=False,
        pre_fill=False,
    )
    assert pos == 400

    pos = get_arrival_position(
        depart_position=300,
        road_length=1000,
        ramp_interval=100,
        min_trip_length=200,
        max_trip_length=200,
        rng=random,
        random_arrival_position=True,
        pre_fill=False,
    )
    assert pos == 500

    pos = get_arrival_position(
        depart_position=300,
        road_length=1000,
        ramp_interval=100,
        min_trip_length=200,
        max_trip_length=500,
        rng=random,
        random_arrival_position=True,
        pre_fill=False,
    )
    assert 500 <= pos <= 800

    pos = get_arrival_position(
        depart_position=300,
        road_length=1000,
        ramp_interval=100,
        min_trip_length=100,
        max_trip_length=1000,
        rng=random,
        random_arrival_position=True,
        pre_fill=False,
    )
    assert 400 <= pos <= 1000


def test_get_desired_speed():
    # no random
    s = get_desired_speed(
        desired_speed=36,
        rng=random,
        speed_variation=0.1,
        min_desired_speed=30,
        max_desired_speed=42,
        random_desired_speed=False,
    )
    assert s == 36

    # random, between min and max
    s = get_desired_speed(
        desired_speed=36,
        rng=random,
        speed_variation=0.1,
        min_desired_speed=30,
        max_desired_speed=42,
        random_desired_speed=True,
    )
    assert 30 <= s <= 42

    # min, max limits
    s = get_desired_speed(
        desired_speed=50,
        rng=random,
        speed_variation=0.1,
        min_desired_speed=50,
        max_desired_speed=50,
        random_desired_speed=True,
    )
    assert s == 50

    # random, no variation
    s = get_desired_speed(
        desired_speed=40,
        rng=random,
        speed_variation=0.0,
        min_desired_speed=30,
        max_desired_speed=42,
        random_desired_speed=True,
    )
    assert 30 <= s <= 42
    assert s == 40


def test_get_depart_speed():
    s = get_depart_speed(
        desired_speed=36,
        rng=random,
        depart_desired=False,
        random_depart_speed=False,
    )
    assert s == 0

    s = get_depart_speed(
        desired_speed=36,
        rng=random,
        depart_desired=True,
        random_depart_speed=False,
    )
    assert s == 36

    s = get_depart_speed(
        desired_speed=36,
        rng=random,
        depart_desired=False,
        random_depart_speed=True,
    )
    assert 0 <= s <= 36

    s = get_depart_speed(
        desired_speed=36,
        rng=random,
        depart_desired=True,
        random_depart_speed=True,
    )
    assert s == 36
