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

import pandas as pd
import pytest

from plafosim.mobility import is_gap_safe
from plafosim.simulator import Simulator, _desired_headway_time, vtype

HEADWAY_TIME = [1.0, 1.25, 1.5]
PLATOON_SIZE = [2, 3, 4]
CACC_SPACING = [5.0, 4.0, 5.5]


@pytest.mark.parametrize("penetration_rate", [0, 1], ids=["HUMAN", "ACC"])
@pytest.mark.parametrize("headway_time", HEADWAY_TIME)
def test_lc_models(penetration_rate: float, headway_time: float):
    """
    Two vehicles (a slow and a fast one) that drive on two lanes and the faster overtakes.
    """

    if penetration_rate == 0 and headway_time != _desired_headway_time:
        pytest.skip(
            f"The Human CF Model can only be configured with {_desired_headway_time}s desired headway time."
        )

    # create simulation environment
    s = Simulator(
        number_of_lanes=2,
        number_of_vehicles=2,
        penetration_rate=penetration_rate,
        record_vehicle_traces=True,
        result_base_filename=f"test_lc_models_penetration{penetration_rate}_headway{headway_time}",
        max_step=100,
        acc_headway_time=headway_time,
    )

    # add first (slow) vehicle
    s._add_vehicle(
        vid=0,
        vtype=vtype,
        depart_position=200,
        arrival_position=s.road_length,
        desired_speed=20,
        depart_lane=0,
        depart_speed=20,
        depart_time=0,
        communication_range=None,
    )

    # add second (fast) vehicle
    s._add_vehicle(
        vid=1,
        vtype=vtype,
        depart_position=10,
        arrival_position=s.road_length,
        desired_speed=30,
        depart_lane=0,
        depart_speed=25,
        depart_time=0,
        communication_range=None,
    )

    assert len(s._vehicles) == 2
    s._last_vehicle_id = 1

    # situation is clear at the beginning
    assert s._vehicles[0].position > s._vehicles[1].position
    assert not s._vehicles[0].blocked_front
    assert not s._vehicles[1].blocked_front
    assert is_gap_safe(
        front_position=s._vehicles[0].position,
        front_speed=s._vehicles[0].speed,
        front_max_deceleration=vtype.max_deceleration,
        front_length=vtype.length,
        back_position=s._vehicles[1].position,
        back_speed=s._vehicles[1].speed,
        back_max_acceleration=vtype.max_acceleration,
        back_min_gap=vtype.min_gap,
        step_length=s._step_length,
    )

    # run the simulation to record the trace file
    s.run()

    # read vehicle trace file
    traces = pd.read_csv(
        f"{s._result_base_filename}_vehicle_traces.csv"
    ).sort_values(["step", "id"])
    traces_indexed = traces.set_index(["step", "id"])
    assert not traces.empty

    # to plot outcome for these tests, use:
    # import seaborn as sns
    # import matplotlib.pyplot as plt
    # fig, ax = plt.subplots(3, figsize=(10, 5))
    # sns.lineplot(data=traces, x='step', y='position', hue='id', ax=ax[0])
    # sns.lineplot(data=traces, x='step', y='speed', hue='id', ax=ax[1])
    # sns.lineplot(data=traces, x='step', y='lane', hue='id', ax=ax[2])
    # fig.tight_layout()

    # both vehicles start in lane 0
    assert (traces.query("step == 0").lane == 0).all()
    # vehicle 0 stays in lane 0 for the whole simulation
    assert (traces.query("id == 0").lane == 0).all()
    # vehicle 1 moves to lane 1 at some point
    assert (traces.query("id == 1").lane == 1).any()
    # vehicle 1 lane-changes left and right exactly once each
    assert (
        traces.query("id == 1").lane.diff() == 1
    ).sum() == 1, "vehicle 1 lane-changes left and right exaclty once each"
    assert (
        traces.query("id == 1").lane.diff() == -1
    ).sum() == 1, "vehicle 1 lane-changes left and right exaclty once each"
    # vehicle 1 first changes left and then right
    change_left_step = traces.query("id == 1").set_index("step").lane.diff().idxmax()
    change_right_step = traces.query("id == 1").set_index("step").lane.diff().idxmin()
    assert change_left_step < change_right_step

    v_front = traces_indexed.loc[change_right_step, 1]
    v_back = traces_indexed.loc[change_right_step, 0]
    # after changing left, vehicle 1 is never again longer blocked
    assert not traces.query(f"id == 1 and step > {change_left_step}").blocked.any()
    # when changing right, vehicle 1 is faster than vehicle 0
    assert v_front.speed > v_back.speed
    # when changing right, vehicle 1 is in front of vehicle 0
    assert v_front.position > v_back.position
    # when changing right there is enough headway between vehicle 1 and 0
    # i.e., if v0 accelerated and v1 decelerated both as hard as possible,
    # they still would not crash
    assert is_gap_safe(
        front_position=s._vehicles[1].position,
        front_speed=s._vehicles[1].speed,
        front_max_deceleration=vtype.max_deceleration,
        front_length=vtype.length,
        back_position=s._vehicles[0].position,
        back_speed=s._vehicles[0].speed,
        back_max_acceleration=vtype.max_acceleration,
        back_min_gap=vtype.min_gap,
        step_length=s._step_length,
    )


@pytest.mark.parametrize("penetration_rate", [0, 1], ids=["HUMAN", "ACC"])
@pytest.mark.parametrize("headway_time", HEADWAY_TIME)
def test_lc_models_with_interferer(penetration_rate: float, headway_time: float):
    """
    Two vehicles (a slow and a fast one) that drive on two lanes and the faster overtakes.
    But there is a third vehicle also overtaking the slowest one, which is slower than the fastest.
    So the fastest has to wait before it can overtake.
    """

    if penetration_rate == 0 and headway_time != _desired_headway_time:
        pytest.skip(
            f"The Human CF model can only be configured with {_desired_headway_time}s desired headway time."
        )

    # create simulation environment
    s = Simulator(
        number_of_lanes=2,
        number_of_vehicles=2,
        penetration_rate=penetration_rate,
        record_vehicle_traces=True,
        result_base_filename=f"test_lc_models_with_interferer_penetration{penetration_rate}_headway{headway_time}",
        max_step=100,
        acc_headway_time=headway_time,
    )

    # add first (slow) vehicle
    s._add_vehicle(
        vid=0,
        vtype=vtype,
        depart_position=100,
        arrival_position=s.road_length,
        desired_speed=20,
        depart_lane=0,
        depart_speed=20,
        depart_time=0,
        communication_range=None,
    )

    # add second (fast) vehicle
    s._add_vehicle(
        vid=1,
        vtype=vtype,
        depart_position=10,
        arrival_position=s.road_length,
        desired_speed=30,
        depart_lane=0,
        depart_speed=30,
        depart_time=0,
        communication_range=None,
    )

    # add third (interfering) vehicle
    s._add_vehicle(
        vid=2,
        vtype=vtype,
        depart_position=60,
        arrival_position=s.road_length,
        desired_speed=25,
        depart_lane=1,
        depart_speed=25,
        depart_time=1,
        communication_range=None,
    )

    assert len(s._vehicles) == 3
    s._last_vehicle_id = 2

    # situation is clear at the beginning
    assert s._vehicles[0].position > s._vehicles[1].position
    assert not s._vehicles[0].blocked_front
    assert not s._vehicles[1].blocked_front
    assert is_gap_safe(
        front_position=s._vehicles[0].position,
        front_speed=s._vehicles[0].speed,
        front_max_deceleration=vtype.max_deceleration,
        front_length=vtype.length,
        back_position=s._vehicles[1].position,
        back_speed=s._vehicles[1].speed,
        back_max_acceleration=vtype.max_acceleration,
        back_min_gap=vtype.min_gap,
        step_length=s._step_length,
    )

    # run the simulation to record the trace file
    s.run()

    # read vehicle trace file
    traces = pd.read_csv(
        f"{s._result_base_filename}_vehicle_traces.csv"
    ).sort_values(["step", "id"])
    traces_indexed = traces.set_index(["step", "id"])
    assert not traces.empty

    # to plot outcome for these tests, use:
    # import seaborn as sns
    # import matplotlib.pyplot as plt
    # fig, ax = plt.subplots(3, figsize=(10, 5))
    # sns.lineplot(data=traces, x='step', y='position', hue='id', ax=ax[0])
    # sns.lineplot(data=traces, x='step', y='speed', hue='id', ax=ax[1])
    # sns.lineplot(data=traces, x='step', y='lane', hue='id', ax=ax[2])
    # fig.tight_layout()

    # both main vehicles start in lane 0
    assert (traces.query("step == 0 and id <= 1").lane == 0).all()
    # the interferer starts in lane 1
    assert traces_indexed.loc[0, 2].lane == 1
    # vehicle 0 stays in lane 0 for the whole simulation
    assert (traces.query("id == 0").lane == 0).all()
    # vehicle 1 moves to lane 1 at some point
    assert (traces.query("id == 1").lane == 1).any()
    # vehicle 1 lane-changes left and right exactly once each
    assert (
        traces.query("id == 1").lane.diff() == 1
    ).sum() == 1, "vehicle 1 lane-changes left and right exactly once each"
    assert (
        traces.query("id == 1").lane.diff() == -1
    ).sum() == 1, "vehicle 1 lane-changes left and right exactly once each"
    # vehicle 2 only changes lanes once
    assert (
        traces.query("id == 2").lane.diff().dropna() != 0
    ).sum() == 1, "vehicle 2 also only lane-changes once"
    # vehicle 1 first changes left and then right
    change_left_step = traces.query("id == 1").set_index("step").lane.diff().idxmax()
    change_right_step = traces.query("id == 1").set_index("step").lane.diff().idxmin()
    assert change_left_step < change_right_step

    # vehicle 1 only changes to the left lane once there is enough space
    # behind the interferer
    assert is_gap_safe(
        front_position=traces_indexed.loc[change_left_step, 2].position,
        front_speed=traces_indexed.loc[change_left_step, 2].speed,
        front_max_deceleration=vtype.max_deceleration,
        front_length=vtype.length,
        back_position=traces_indexed.loc[change_left_step, 1].position,
        back_speed=traces_indexed.loc[change_left_step, 1].speed,
        back_max_acceleration=vtype.max_acceleration,
        back_min_gap=vtype.min_gap,
        step_length=s.step_length,
    )
    # vehicle 1 only changes back to the right lane after vehicle 2 did that
    assert (
        change_right_step
        > traces.query("id == 2").set_index("step").lane.diff().idxmin()
    )
    # vehicle 1 eventually also overtakes vehicle 2
    assert (
        traces_indexed.xs(1, level="id").position
        > traces_indexed.xs(2, level="id").position
    ).any()


@pytest.mark.parametrize("size", PLATOON_SIZE)
@pytest.mark.parametrize("cacc_spacing", CACC_SPACING)
def test_lc_model_CACC(size: int, cacc_spacing: float):
    """
    A platoon driving on one lane.
    This is just the vehicles in the platoon.
    """

    # create simulation environment
    s = Simulator(
        number_of_lanes=2,
        number_of_vehicles=size,
        penetration_rate=1,
        record_vehicle_traces=True,
        result_base_filename=f"test_lc_models_CACC_size{size}_spacing{cacc_spacing}",
        max_step=100,
        start_as_platoon=True,
        pre_fill=True,
        random_desired_speed=False,
        depart_desired=True,
        desired_speed=30,
        cacc_spacing=cacc_spacing,
    )

    # add preceding (slow) vehicle
    s._add_vehicle(
        vid=size,
        vtype=vtype,
        depart_position=200,
        arrival_position=s.road_length,
        desired_speed=20,
        depart_lane=0,
        depart_speed=20,
        depart_time=0,
        communication_range=None,
    )

    assert len(s._vehicles) == size + 1
    s._last_vehicle_id = size

    # run the simulation to record the trace file
    s.run()

    # read vehicle trace file
    traces = pd.read_csv(
        f"{s._result_base_filename}_vehicle_traces.csv"
    ).sort_values(["step", "id"])
    assert not traces.empty

    platoon = (
        traces.query(f"id < {size}")
        .set_index("step")
        .assign(platoon_rank=lambda df: size - df.id)
    )
    leader = platoon.query("id == 0")
    followers = platoon.query(f"0 < id < {size}")
    platoon_end = platoon.query(f"id == {size - 1}")
    predecessor = traces.query(f"id == {size}").set_index("step")

    # all vehicles start in lane 0
    assert (traces.query("step == 0").lane == 0).all()
    # preceding vehicle stays in lane 0 for the whole simulation
    assert (traces.query(f"id == {size}").lane == 0).all()
    # all platoon followers have the same lane and speed as the leader
    assert followers.groupby("id").lane.apply(lambda x: x == leader.lane).all()
    assert followers.groupby("id").speed.apply(lambda x: x == leader.speed).all()
    # the platoon maintains its order through the whole simulation
    assert (
        (
            platoon.sort_values(["step", "platoon_rank"]).reset_index()
            == platoon.sort_values(["step", "position"]).reset_index()
        )
        .all()
        .all()
    )

    # now we can use vehicle `size` as the end of the platoon

    # the platoon moves to lane 1 at some point
    assert (platoon_end.lane == 1).any()
    # the platoon lane-changes left and right exactly once each
    assert (
        platoon_end.lane.diff() == 1
    ).sum() == 1, "the platoon lane-changes left and right exactly once each"
    assert (
        platoon_end.lane.diff() == -1
    ).sum() == 1, "the platoon lane-changes left and right exactly once each"
    # the platoon first changes left and then right
    change_left_step = platoon_end.lane.diff().idxmax()
    change_right_step = platoon_end.lane.diff().idxmin()
    assert change_left_step < change_right_step
    # when changing right, the platoon is faster than the (earlier) predecessor
    assert (
        platoon_end.loc[change_right_step].speed
        > predecessor.loc[change_right_step].speed
    )
    # when changing right, the platoon is in front of the (earlier) predecessor
    assert (
        platoon_end.loc[change_right_step].position
        > predecessor.loc[change_right_step].position
    )
    # when changing right there is enough headway the vehicles
    assert is_gap_safe(
        front_position=platoon_end.loc[change_right_step].position,
        front_speed=platoon_end.loc[change_right_step].speed,
        front_max_deceleration=vtype.max_deceleration,
        front_length=vtype.length,
        back_position=predecessor.loc[change_right_step].position,
        back_speed=predecessor.loc[change_right_step].speed,
        back_max_acceleration=vtype.max_acceleration,
        back_min_gap=vtype.min_gap,
        step_length=s.step_length,
    )


@pytest.mark.parametrize("size", PLATOON_SIZE)
@pytest.mark.parametrize("cacc_spacing", CACC_SPACING)
def test_lc_model_CACC_with_interferer(size: int, cacc_spacing: float):
    """
    A platoon driving on one lane.
    This is just the vehicles in the platoon.
    """

    # create simulation environment
    s = Simulator(
        number_of_lanes=2,
        number_of_vehicles=size,
        penetration_rate=1,
        record_vehicle_traces=True,
        result_base_filename=f"test_lc_models_CACC_with_interferer_size{size}_spacing{cacc_spacing}",
        max_step=100,
        start_as_platoon=True,
        pre_fill=True,
        random_desired_speed=False,
        depart_desired=True,
        desired_speed=30,
        cacc_spacing=cacc_spacing,
    )
    platoon_top_position = s._vehicles[0].position

    # add preceding (slow) vehicle
    s._add_vehicle(
        vid=size,
        vtype=vtype,
        depart_position=platoon_top_position + 80,
        arrival_position=s.road_length,
        desired_speed=20,
        depart_lane=0,
        depart_speed=20,
        depart_time=0,
        communication_range=None,
    )

    # add another (interfering) vehicle
    s._add_vehicle(
        vid=size + 1,
        vtype=vtype,
        depart_position=platoon_top_position + 40,
        arrival_position=s.road_length,
        desired_speed=25,
        depart_lane=1,
        depart_speed=25,
        depart_time=1,
        communication_range=None,
    )

    assert len(s._vehicles) == size + 2
    s._last_vehicle_id = size + 1

    # run the simulation to record the trace file
    s.run()

    # read vehicle trace file
    traces = pd.read_csv(
        f"{s._result_base_filename}_vehicle_traces.csv"
    ).sort_values(["step", "id"])
    assert not traces.empty

    platoon = (
        traces.query(f"id < {size}")
        .set_index("step")
        .assign(platoon_rank=lambda df: size - df.id)
    )
    leader = platoon.query("id == 0")
    followers = platoon.query(f"0 < id < {size}")
    platoon_end = platoon.query(f"id == {size - 1}")
    predecessor = traces.query(f"id == {size}").set_index("step")
    interferer = traces.query(f"id == {size+1}").set_index("step")

    # all platoon vehicles start in lane 0
    assert (platoon.loc[0].lane == 0).all()
    # preceding vehicle stays in lane 0 for the whole simulation
    assert (predecessor.lane == 0).all()
    # all platoon followers have the same lane and speed as the leader
    assert followers.groupby("id").lane.apply(lambda x: x == leader.lane).all()
    assert followers.groupby("id").speed.apply(lambda x: x == leader.speed).all()
    # the platoon maintains its order through the whole simulation
    assert (
        (
            platoon.sort_values(["step", "platoon_rank"]).reset_index()
            == platoon.sort_values(["step", "position"]).reset_index()
        )
        .all()
        .all()
    )

    # now we can use vehicle `size` as the end of the platoon

    # the platoon moves to lane 1 at some point
    assert (platoon_end.lane == 1).any()
    # the platoon lane-changes left and right exactly once each
    assert (
        platoon_end.lane.diff() == 1
    ).sum() == 1, "the platoon lane-changes left and right exactly once each"
    assert (
        platoon_end.lane.diff() == -1
    ).sum() == 1, "the platoon lane-changes left and right exactly once each"
    # the platoon first changes left and then right
    change_left_step = platoon_end.lane.diff().idxmax()
    change_right_step = platoon_end.lane.diff().idxmin()
    assert change_left_step < change_right_step
    # when changing right, the platoon is faster than the (earlier) predecessor
    assert (
        platoon_end.loc[change_right_step].speed
        > predecessor.loc[change_right_step].speed
    )
    # when changing right, the platoon is in front of the (earlier) predecessor
    assert (
        platoon_end.loc[change_right_step].position
        > predecessor.loc[change_right_step].position
    )
    # when changing right there is enough headway the vehicles
    assert is_gap_safe(
        front_position=platoon_end.loc[change_right_step].position,
        front_speed=platoon_end.loc[change_right_step].speed,
        front_max_deceleration=vtype.max_deceleration,
        front_length=vtype.length,
        back_position=predecessor.loc[change_right_step].position,
        back_speed=predecessor.loc[change_right_step].speed,
        back_max_acceleration=vtype.max_acceleration,
        back_min_gap=vtype.min_gap,
        step_length=s.step_length,
    )

    # the platoon only changes to the left lane once there is enough space
    # behind the interferer, like in the check above
    assert is_gap_safe(
        front_position=interferer.loc[change_left_step].position,
        front_speed=interferer.loc[change_left_step].speed,
        front_max_deceleration=vtype.max_deceleration,
        front_length=vtype.length,
        back_position=leader.loc[change_left_step].position,
        back_speed=leader.loc[change_left_step].speed,
        back_max_acceleration=vtype.max_acceleration,
        back_min_gap=vtype.min_gap,
        step_length=s.step_length,
    )
    # the platoon only changes back to the right lane after vehicle 2 did that
    assert change_right_step > interferer.lane.diff().idxmin()
    # vehicle 1 eventually also overtakes vehicle 2
    assert (platoon_end.position > interferer.position).any()


@pytest.mark.parametrize("size", PLATOON_SIZE)
@pytest.mark.parametrize("cacc_spacing", CACC_SPACING)
def test_lc_model_CACC_with_interferer_leader(size: int, cacc_spacing: float):
    """
    The platoon cannot change lane since there is an interferer at the leader.
    """

    # create simulation environment
    s = Simulator(
        number_of_lanes=2,
        number_of_vehicles=size,
        penetration_rate=1,
        record_vehicle_traces=True,
        result_base_filename=f"test_lc_models_CACC_with_interferer_leader_size{size}_spacing{cacc_spacing}",
        record_prefilled=True,
        max_step=100,
        start_as_platoon=True,
        pre_fill=True,
        random_desired_speed=False,
        depart_desired=True,
        desired_speed=30,
        cacc_spacing=cacc_spacing,
    )
    platoon_top_position = s._vehicles[0].position

    # add preceding (slow) vehicle
    s._add_vehicle(
        vid=size,
        vtype=vtype,
        depart_position=platoon_top_position + 80,
        arrival_position=s.road_length,
        desired_speed=20,
        depart_lane=0,
        depart_speed=20,
        depart_time=0,
        communication_range=None,
    )

    # add another (interfering) vehicle
    s._add_vehicle(
        vid=size + 1,
        vtype=vtype,
        depart_position=platoon_top_position - vtype.length,
        arrival_position=s.road_length,
        desired_speed=25,
        depart_lane=1,
        depart_speed=25,
        depart_time=1,
        communication_range=None,
    )

    assert len(s._vehicles) == size + 2
    s._last_vehicle_id = size + 1

    # run the simulation to record the trace file
    s.run()

    # read vehicle trace file
    traces = pd.read_csv(
        f"{s._result_base_filename}_vehicle_traces.csv"
    ).sort_values(["step", "id"])
    assert not traces.empty

    platoon = (
        traces.query(f"id < {size}")
        .set_index("step")
        .assign(platoon_rank=lambda df: size - df.id)
    )
    leader = platoon.query("id == 0")
    followers = platoon.query(f"0 < id < {size}")
    platoon_end = platoon.query(f"id == {size - 1}")
    predecessor = traces.query(f"id == {size}").set_index("step")
    interferer = traces.query(f"id == {size+1}").set_index("step")

    # all platoon vehicles start in lane 0
    assert (platoon.loc[0].lane == 0).all()
    # preceding vehicle stays in lane 0 for the whole simulation
    assert (predecessor.lane == 0).all()
    # all platoon followers have the same lane and speed as the leader
    assert followers.groupby("id").lane.apply(lambda x: x == leader.lane).all()
    assert followers.groupby("id").speed.apply(lambda x: x == leader.speed).all()
    # the platoon maintains its order through the whole simulation
    assert (
        (
            platoon.sort_values(["step", "platoon_rank"]).reset_index()
            == platoon.sort_values(["step", "position"]).reset_index()
        )
        .all()
        .all()
    )

    # now we can use vehicle `size` as the end of the platoon

    # the platoon moves to lane 1 at some point
    assert (platoon_end.lane == 1).any()
    # the platoon lane-changes left and right exactly once each
    assert (
        platoon_end.lane.diff() == 1
    ).sum() == 1, "the platoon lane-changes left and right exactly once each"
    assert (
        platoon_end.lane.diff() == -1
    ).sum() == 1, "the platoon lane-changes left and right exactly once each"
    # the platoon first changes left and then right
    change_left_step = platoon_end.lane.diff().idxmax()
    change_right_step = platoon_end.lane.diff().idxmin()
    assert change_left_step < change_right_step
    # when changing right, the platoon is faster than the (earlier) predecessor
    assert (
        platoon_end.loc[change_right_step].speed
        > predecessor.loc[change_right_step].speed
    )
    # when changing right, the platoon is in front of the (earlier) predecessor
    assert (
        platoon_end.loc[change_right_step].position
        > predecessor.loc[change_right_step].position
    )
    # when changing right there is enough headway the vehicles
    assert is_gap_safe(
        front_position=platoon_end.loc[change_right_step].position,
        front_speed=platoon_end.loc[change_right_step].speed,
        front_max_deceleration=vtype.max_deceleration,
        front_length=vtype.length,
        back_position=predecessor.loc[change_right_step].position,
        back_speed=predecessor.loc[change_right_step].speed,
        back_max_acceleration=vtype.max_acceleration,
        back_min_gap=vtype.min_gap,
        step_length=s.step_length,
    )

    # the platoon only changes to the left lane once there is enough space
    # behind the interferer, like in the check above
    assert is_gap_safe(
        front_position=interferer.loc[change_left_step].position,
        front_speed=interferer.loc[change_left_step].speed,
        front_max_deceleration=vtype.max_deceleration,
        front_length=vtype.length,
        back_position=leader.loc[change_left_step].position,
        back_speed=leader.loc[change_left_step].speed,
        back_max_acceleration=vtype.max_acceleration,
        back_min_gap=vtype.min_gap,
        step_length=s.step_length,
    )
    # the platoon only changes back to the right lane after vehicle 2 did that
    assert change_right_step > interferer.lane.diff().idxmin()
    # vehicle 1 eventually also overtakes vehicle 2
    assert (platoon_end.position > interferer.position).any()


@pytest.mark.parametrize("size", [10, 15, 20])
@pytest.mark.parametrize("cacc_spacing", CACC_SPACING)
def test_lc_model_CACC_with_interferer_members(size: int, cacc_spacing: float):
    """
    The platoon cannot change lane since there is an interferer at an arbitrary follower (e.g., the last)
    """

    # create simulation environment
    s = Simulator(
        number_of_lanes=2,
        number_of_vehicles=size,
        penetration_rate=1,
        record_vehicle_traces=True,
        result_base_filename=f"test_lc_models_CACC_with_interferer_members_size{size}_spacing{cacc_spacing}",
        record_prefilled=True,
        max_step=120,
        start_as_platoon=True,
        pre_fill=True,
        random_desired_speed=False,
        depart_desired=True,
        desired_speed=30,
        cacc_spacing=cacc_spacing,
    )
    platoon_top_position = s._vehicles[0].position

    # add preceding (slow) vehicle
    s._add_vehicle(
        vid=size,
        vtype=vtype,
        depart_position=platoon_top_position + 40,
        arrival_position=s.road_length,
        desired_speed=20,
        depart_lane=0,
        depart_speed=20,
        depart_time=0,
        communication_range=None,
    )

    # add another (interfering) vehicle
    s._add_vehicle(
        vid=size + 1,
        vtype=vtype,
        depart_position=vtype.length,
        arrival_position=s.road_length,
        desired_speed=25,
        depart_lane=1,
        depart_speed=25,
        depart_time=1,
        communication_range=None,
    )

    assert len(s._vehicles) == size + 2
    s._last_vehicle_id = size + 1

    # run the simulation to record the trace file
    s.run()

    # read vehicle trace file
    traces = pd.read_csv(
        f"{s._result_base_filename}_vehicle_traces.csv"
    ).sort_values(["step", "id"])
    assert not traces.empty

    platoon = (
        traces.query(f"id < {size}")
        .set_index("step")
        .assign(platoon_rank=lambda df: size - df.id)
    )
    leader = platoon.query("id == 0")
    followers = platoon.query(f"0 < id < {size}")
    platoon_end = platoon.query(f"id == {size - 1}")
    predecessor = traces.query(f"id == {size}").set_index("step")
    interferer = traces.query(f"id == {size+1}").set_index("step")

    # all platoon vehicles start in lane 0
    assert (platoon.loc[0].lane == 0).all()
    # preceding vehicle stays in lane 0 for the whole simulation
    assert (predecessor.lane == 0).all()
    # all platoon followers have the same lane and speed as the leader
    assert followers.groupby("id").lane.apply(lambda x: x == leader.lane).all()
    assert followers.groupby("id").speed.apply(lambda x: x == leader.speed).all()
    # the platoon maintains its order through the whole simulation
    assert (
        (
            platoon.sort_values(["step", "platoon_rank"]).reset_index()
            == platoon.sort_values(["step", "position"]).reset_index()
        )
        .all()
        .all()
    )

    # now we can use vehicle `size` as the end of the platoon

    # the platoon moves to lane 1 at some point
    assert (platoon_end.lane == 1).any()
    # the platoon lane-changes left and right exactly once each
    assert (
        platoon_end.lane.diff() == 1
    ).sum() == 1, "the platoon lane-changes left and right exactly once each"
    assert (
        platoon_end.lane.diff() == -1
    ).sum() == 1, "the platoon lane-changes left and right exactly once each"
    # the platoon first changes left and then right
    change_left_step = platoon_end.lane.diff().idxmax()
    change_right_step = platoon_end.lane.diff().idxmin()
    assert change_left_step < change_right_step
    # when changing right, the platoon is faster than the (earlier) predecessor
    assert (
        platoon_end.loc[change_right_step].speed
        > predecessor.loc[change_right_step].speed
    )
    # when changing right, the platoon is in front of the (earlier) predecessor
    assert (
        platoon_end.loc[change_right_step].position
        > predecessor.loc[change_right_step].position
    )
    # when changing right there is enough headway the vehicles
    assert is_gap_safe(
        front_position=platoon_end.loc[change_right_step].position,
        front_speed=platoon_end.loc[change_right_step].speed,
        front_max_deceleration=vtype.max_deceleration,
        front_length=vtype.length,
        back_position=predecessor.loc[change_right_step].position,
        back_speed=predecessor.loc[change_right_step].speed,
        back_max_acceleration=vtype.max_acceleration,
        back_min_gap=vtype.min_gap,
        step_length=s.step_length,
    )

    # the platoon only changes to the left lane once there is enough space
    # behind the interferer, like in the check above
    assert is_gap_safe(
        front_position=interferer.loc[change_left_step].position,
        front_speed=interferer.loc[change_left_step].speed,
        front_max_deceleration=vtype.max_deceleration,
        front_length=vtype.length,
        back_position=leader.loc[change_left_step].position,
        back_speed=leader.loc[change_left_step].speed,
        back_max_acceleration=vtype.max_acceleration,
        back_min_gap=vtype.min_gap,
        step_length=s.step_length,
    )
    # the platoon only changes back to the right lane after vehicle 2 did that
    assert change_right_step > interferer.lane.diff().idxmin()
    # vehicle 1 eventually also overtakes vehicle 2
    assert (platoon_end.position > interferer.position).any()


@pytest.mark.parametrize("size", PLATOON_SIZE)
@pytest.mark.parametrize("cacc_spacing", CACC_SPACING)
def test_lc_model_CACC_conflict_leader(size: int, cacc_spacing: float):
    """
    A vehicle wants to change left, the platoon wants to change right --> conflict.
    """

    # create simulation environment
    s = Simulator(
        number_of_lanes=4,
        number_of_vehicles=size,
        penetration_rate=1,
        record_vehicle_traces=True,
        result_base_filename=f"test_lc_models_CACC_conflict_leader_size{size}_spacing{cacc_spacing}",
        record_prefilled=True,
        max_step=120,
        start_as_platoon=True,
        pre_fill=True,
        random_desired_speed=False,
        depart_desired=True,
        desired_speed=30,
        cacc_spacing=cacc_spacing,
    )
    platoon_top_position = s._vehicles[0].position

    # add preceding (slow) vehicle
    s._add_vehicle(
        vid=size,
        vtype=vtype,
        depart_position=platoon_top_position + 50,
        arrival_position=s.road_length,
        desired_speed=20,
        depart_lane=0,
        depart_speed=20,
        depart_time=0,
        communication_range=None,
    )

    # add another (interfering) vehicle
    s._add_vehicle(
        vid=size + 1,
        vtype=vtype,
        depart_position=vtype.length,
        arrival_position=s.road_length,
        desired_speed=31,
        depart_lane=2,
        depart_speed=31,
        depart_time=1,
        communication_range=None,
    )

    assert len(s._vehicles) == size + 2
    s._last_vehicle_id = size + 1

    # run the simulation to record the trace file
    s.run()

    # read vehicle trace file
    traces = pd.read_csv(
        f"{s._result_base_filename}_vehicle_traces.csv"
    ).sort_values(["step", "id"])
    assert not traces.empty

    platoon = (
        traces.query(f"id < {size}")
        .set_index("step")
        .assign(platoon_rank=lambda df: size - df.id)
    )
    leader = platoon.query("id == 0")
    followers = platoon.query(f"0 < id < {size}")
    platoon_end = platoon.query(f"id == {size - 1}")
    predecessor = traces.query(f"id == {size}").set_index("step")
    interferer = traces.query(f"id == {size+1}").set_index("step")

    # all platoon vehicles start in lane 0
    assert (platoon.loc[0].lane == 0).all()
    # preceding vehicle stays in lane 0 for the whole simulation
    assert (predecessor.lane == 0).all()
    # all platoon followers have the same lane and speed as the leader
    assert followers.groupby("id").lane.apply(lambda x: x == leader.lane).all()
    assert followers.groupby("id").speed.apply(lambda x: x == leader.speed).all()
    # the platoon maintains its order through the whole simulation
    assert (
        (
            platoon.sort_values(["step", "platoon_rank"]).reset_index()
            == platoon.sort_values(["step", "position"]).reset_index()
        )
        .all()
        .all()
    )

    # now we can use vehicle `size` as the end of the platoon

    # the platoon moves to lane 1 at some point
    assert (platoon_end.lane == 1).any()
    # the platoon lane-changes left and right exactly once each
    assert (
        platoon_end.lane.diff() == 1
    ).sum() == 1, "the platoon lane-changes left and right exactly once each"
    assert (
        platoon_end.lane.diff() == -1
    ).sum() == 1, "the platoon lane-changes left and right exactly once each"
    # the platoon first changes left and then right
    change_left_step = platoon_end.lane.diff().idxmax()
    change_right_step = platoon_end.lane.diff().idxmin()
    assert change_left_step < change_right_step
    # when changing right, the platoon is faster than the (earlier) predecessor
    assert (
        platoon_end.loc[change_right_step].speed
        > predecessor.loc[change_right_step].speed
    )
    # when changing right, the platoon is in front of the (earlier) predecessor
    assert (
        platoon_end.loc[change_right_step].position
        > predecessor.loc[change_right_step].position
    )
    # when changing right there is enough headway the vehicles
    assert is_gap_safe(
        front_position=platoon_end.loc[change_right_step].position,
        front_speed=platoon_end.loc[change_right_step].speed,
        front_max_deceleration=vtype.max_deceleration,
        front_length=vtype.length,
        back_position=predecessor.loc[change_right_step].position,
        back_speed=predecessor.loc[change_right_step].speed,
        back_max_acceleration=vtype.max_acceleration,
        back_min_gap=vtype.min_gap,
        step_length=s.step_length,
    )

    # find all steps where platoon and interferer are on the same lane
    same_lane_steps = interferer.loc[(interferer.lane == leader.lane)].index
    df_same_lane = leader.loc[same_lane_steps].merge(
        interferer.loc[same_lane_steps],
        left_index=True,
        right_index=True,
        suffixes=["_leader", "_interferer"],
    )

    # the platoon only changes to the left lane once there is enough space
    # behind the interferer, like in the check above
    def myfunc(row):
        return is_gap_safe(
            front_position=row.position_interferer,
            front_speed=row.speed_interferer,
            front_max_deceleration=vtype.max_deceleration,
            front_length=vtype.length,
            back_position=row.position_leader,
            back_speed=row.speed_leader,
            back_max_acceleration=vtype.max_acceleration,
            back_min_gap=vtype.min_gap,
            step_length=s.step_length,
        )

    assert df_same_lane.apply(myfunc, axis=1, raw=False, result_type="reduce").all()
    # the platoon changes back to the right lane before vehicle 2 did that
    assert change_right_step < interferer.lane.diff().idxmin()
    # platoon does not overtake the interferer
    assert (
        platoon_end.query(f"step >= {change_right_step}").position
        < interferer.query(f"step >= {change_right_step}").position
    ).all()


@pytest.mark.parametrize("size", [10, 15, 20])
@pytest.mark.parametrize("cacc_spacing", CACC_SPACING)
def test_lc_model_CACC_conflict_members(size: int, cacc_spacing: float):
    """
    A vehicle wants to change left, the platoon wants to change right --> conflict.
    """

    # create simulation environment
    s = Simulator(
        number_of_lanes=4,
        number_of_vehicles=size,
        penetration_rate=1,
        record_vehicle_traces=True,
        result_base_filename=f"test_lc_models_CACC_conflict_members_size{size}_spacing{cacc_spacing}",
        record_prefilled=True,
        max_step=120,
        start_as_platoon=True,
        pre_fill=True,
        random_desired_speed=False,
        depart_desired=True,
        desired_speed=30,
        cacc_spacing=cacc_spacing,
    )
    platoon_top_position = s._vehicles[0].position

    # add preceding (slow) vehicle
    s._add_vehicle(
        vid=size,
        vtype=vtype,
        depart_position=platoon_top_position + 40,
        arrival_position=s.road_length,
        desired_speed=20,
        depart_lane=0,
        depart_speed=20,
        depart_time=0,
        communication_range=None,
    )

    # add another (interfering) vehicle
    s._add_vehicle(
        vid=size + 1,
        vtype=vtype,
        depart_position=vtype.length,
        arrival_position=s.road_length,
        desired_speed=31,
        depart_lane=2,
        depart_speed=31,
        depart_time=1,
        communication_range=None,
    )

    assert len(s._vehicles) == size + 2
    s._last_vehicle_id = size + 1

    # run the simulation to record the trace file
    s.run()

    # read vehicle trace file
    traces = pd.read_csv(
        f"{s._result_base_filename}_vehicle_traces.csv"
    ).sort_values(["step", "id"])
    assert not traces.empty

    platoon = (
        traces.query(f"id < {size}")
        .set_index("step")
        .assign(platoon_rank=lambda df: size - df.id)
    )
    leader = platoon.query("id == 0")
    followers = platoon.query(f"0 < id < {size}")
    platoon_end = platoon.query(f"id == {size - 1}")
    predecessor = traces.query(f"id == {size}").set_index("step")
    interferer = traces.query(f"id == {size+1}").set_index("step")

    # all platoon vehicles start in lane 0
    assert (platoon.loc[0].lane == 0).all()
    # preceding vehicle stays in lane 0 for the whole simulation
    assert (predecessor.lane == 0).all()
    # all platoon followers have the same lane and speed as the leader
    assert followers.groupby("id").lane.apply(lambda x: x == leader.lane).all()
    assert followers.groupby("id").speed.apply(lambda x: x == leader.speed).all()
    # the platoon maintains its order through the whole simulation
    assert (
        (
            platoon.sort_values(["step", "platoon_rank"]).reset_index()
            == platoon.sort_values(["step", "position"]).reset_index()
        )
        .all()
        .all()
    )

    # now we can use vehicle `size` as the end of the platoon

    # the platoon moves to lane 1 at some point
    assert (platoon_end.lane == 1).any()
    # the platoon lane-changes left and right exactly once each
    assert (
        platoon_end.lane.diff() == 1
    ).sum() == 1, "the platoon lane-changes left and right exactly once each"
    assert (
        platoon_end.lane.diff() == -1
    ).sum() == 1, "the platoon lane-changes left and right exactly once each"
    # the platoon first changes left and then right
    change_left_step = platoon_end.lane.diff().idxmax()
    change_right_step = platoon_end.lane.diff().idxmin()
    assert change_left_step < change_right_step
    # when changing right, the platoon is faster than the (earlier) predecessor
    assert (
        platoon_end.loc[change_right_step].speed
        > predecessor.loc[change_right_step].speed
    )
    # when changing right, the platoon is in front of the (earlier) predecessor
    assert (
        platoon_end.loc[change_right_step].position
        > predecessor.loc[change_right_step].position
    )
    # when changing right there is enough headway the vehicles
    assert is_gap_safe(
        front_position=platoon_end.loc[change_right_step].position,
        front_speed=platoon_end.loc[change_right_step].speed,
        front_max_deceleration=vtype.max_deceleration,
        front_length=vtype.length,
        back_position=predecessor.loc[change_right_step].position,
        back_speed=predecessor.loc[change_right_step].speed,
        back_max_acceleration=vtype.max_acceleration,
        back_min_gap=vtype.min_gap,
        step_length=s.step_length,
    )  # or platoon_end.lane != predecessor.lane

    # find all steps where platoon and interferer are on the same lane
    same_lane_steps = interferer.loc[(interferer.lane == leader.lane)].index
    df_same_lane = leader.loc[same_lane_steps].merge(
        interferer.loc[same_lane_steps],
        left_index=True,
        right_index=True,
        suffixes=["_leader", "_interferer"],
    )

    # the platoon only changes to the left lane once there is enough space
    # behind the interferer, like in the check above
    def myfunc(row):
        return is_gap_safe(
            front_position=row.position_interferer,
            front_speed=row.speed_interferer,
            front_max_deceleration=vtype.max_deceleration,
            front_length=vtype.length,
            back_position=row.position_leader,
            back_speed=row.speed_leader,
            back_max_acceleration=vtype.max_acceleration,
            back_min_gap=vtype.min_gap,
            step_length=s.step_length,
        )

    assert df_same_lane.apply(myfunc, axis=1, raw=False, result_type="reduce").all()
    # the platoon changes back to the right lane before vehicle 2 did that
    assert change_right_step < interferer.lane.diff().idxmin()
    # platoon does not overtake the interferer
    assert (
        platoon_end.query(f"step >= {change_right_step}").position
        < interferer.query(f"step >= {change_right_step}").position
    ).all()
