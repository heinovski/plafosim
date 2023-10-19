#
# Copyright (c) 2020-2023 Julian Heinovski <heinovski@ccs-labs.org>
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

import numpy as np
import pandas as pd
import pytest

from plafosim.mobility import CF_Model, is_gap_safe
from plafosim.simulator import Simulator, _desired_headway_time, vtype

SPEED = [20, 30, 40]
HEADWAY_TIME = [1.0, 1.25, 1.5]
PLATOON_SIZE = [2, 3, 4]
CACC_SPACING = [5.0, 4.0, 5.5]


@pytest.mark.parametrize("penetration_rate", [0, 1], ids=["HUMAN", "ACC"])
@pytest.mark.parametrize("headway_time", HEADWAY_TIME)
@pytest.mark.parametrize("slow_speed", SPEED)
def test_cf_models_blocked(
    penetration_rate: float, headway_time: float, slow_speed: float
):
    """
    Two vehicles (a slow and a fast one) that drive on one lane.
    """

    if penetration_rate == 0 and headway_time != _desired_headway_time:
        pytest.skip(f"The Human CF model can only be configured with {_desired_headway_time}s desired headway time.")

    # create simulation environment
    s = Simulator(
        number_of_lanes=1,
        number_of_vehicles=2,
        penetration_rate=penetration_rate,
        record_vehicle_traces=True,
        result_base_filename=f"test_cf_models_blocked_penetration{penetration_rate}_headway{headway_time}_speed{slow_speed}",
        max_step=100,
        acc_headway_time=headway_time,
    )

    # add first (slow) vehicle
    s._add_vehicle(
        vid=0,
        vtype=vtype,
        depart_position=250,
        arrival_position=s.road_length,
        desired_speed=slow_speed,
        depart_lane=0,
        depart_speed=slow_speed,
        depart_time=0,
        communication_range=None,
    )

    # add second (fast) vehicle
    s._add_vehicle(
        vid=1,
        vtype=vtype,
        depart_position=10,
        arrival_position=s.road_length,
        desired_speed=slow_speed + 10,
        depart_lane=0,
        depart_speed=slow_speed + 5,
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
    traces = pd.read_csv(f"{s._result_base_filename}_vehicle_traces.csv").sort_values(["step", "id"])
    assert not traces.empty

    # same desired speed over entire simulation
    assert (traces.groupby("id").desiredSpeed.std() == 0).all()
    # same cf target speed over entire simulation
    assert (traces.groupby("id").cfTargetSpeed.std() == 0).all()
    # vehicle 0 has the same speed in the entire simulation
    assert traces.query("id == 0").speed.std() == 0
    # (constant) speed of vehicle 0
    speed_vehicle0 = (
        traces
        .set_index(["id", "step"])
        .loc[(0, traces.step.min()), "speed"]
    )
    # vehicle 0 is driving at its desired speed
    assert speed_vehicle0 == traces.query("id == 0").cfTargetSpeed.min()

    # vehicle 1 reaches its desired speed for the first time
    first_step_desired = traces.query("id == 1 and speed == cfTargetSpeed").step.min()
    # there is a step where vehicle 1 reaches its desired speed
    assert not np.isnan(first_step_desired)
    # vehicle 1 did not start with desired speed
    assert first_step_desired > traces.step.min()

    # the first step vehicle 1 brakes
    first_step_braking = traces.query("id == 1")[
        traces.query("id == 1").speed.pct_change() < 0
    ].step.min()
    # vehicle 1 is not blocked until starting to brake
    assert not traces.query(f"id == 1 and step < {first_step_braking}").blocked.any()

    # vehicle 1 remains blocked from braking until the end
    assert traces.query(f"id == 1 and step >= {first_step_braking}").blocked.all()

    # the first step vehicle 1 (approx.) reaches the speed of vehicle 0
    first_step_speed_vehicle0 = (
        traces
        .query(f"id == 1 and abs(speed - {speed_vehicle0}) < 0.01")
        .step
        .min()
    )
    # the speed is reached (approx.) at all
    assert not np.isnan(first_step_speed_vehicle0)
    # the speed is reached after the vehicle reaches its desired speed
    assert first_step_desired < first_step_speed_vehicle0

    # vehicle 1 sticks with the (slow) speed (of vehicle 1)
    assert (
        traces.query(f"id == 1 and step >= {first_step_speed_vehicle0}").speed.std()
        < 0.01
    ).all()

    # correct order of vehicles at the end
    assert (
        traces.query(f"step == {traces.step.max()}")
        .sort_values("id", ascending=False)
        .position.is_monotonic
    )


@pytest.mark.parametrize("size", PLATOON_SIZE)
@pytest.mark.parametrize("cacc_spacing", CACC_SPACING)
def test_cf_model_CACC(size: int, cacc_spacing: float):
    """
    A platoon driving on one lane.
    This is just the vehicles in the platoon.
    """

    # create simulation environment
    s = Simulator(
        number_of_lanes=1,
        number_of_vehicles=size,
        penetration_rate=1,
        record_vehicle_traces=True,
        result_base_filename=f"test_cf_model_CACC_size{size}_spacing{cacc_spacing}",
        max_step=100,
        start_as_platoon=True,
        pre_fill=True,
        random_desired_speed=True,
        update_desired_speed=False,
        depart_desired=True,
        desired_speed=30,
        cacc_spacing=cacc_spacing,
    )

    assert len(s._vehicles) == size
    s._last_vehicle_id = size - 1

    # run the simulation to record the trace file
    s.run()

    # read vehicle trace file
    traces = pd.read_csv(f"{s._result_base_filename}_vehicle_traces.csv").sort_values(["step", "id"])
    assert not traces.empty

    # leader ACC
    assert (traces.query("id == 0").cfModel == CF_Model.ACC.name).all()
    # follower CACC
    assert (traces.query("id > 0").cfModel == CF_Model.CACC.name).all()

    # same desired speed over entire simulation
    assert (traces.groupby("id").desiredSpeed.std() == 0).all()
    # same cf target speed over entire simulation
    assert (traces.groupby("id").cfTargetSpeed.std() == 0).all()
    # same speed over entire simulation
    assert (traces.groupby("id").speed.std() == 0).all()
    # same desired and target speed
    assert traces.desiredSpeed.min() == traces.cfTargetSpeed.min()
    # same speed and target speed
    assert traces.speed.min() == traces.cfTargetSpeed.min()

    # all followers have the same speed as the leader during the entire simulation
    leader_speed = traces.query("id == 0").set_index("step").speed
    assert (
        traces.set_index("step")
        .groupby("id")
        .speed.apply(lambda x: x == leader_speed)
        .all()
    )

    follower_gaps = (
        traces
        .groupby("step")
        .apply(lambda g: g.sort_values("position", ascending=False))
        .assign(gap=lambda g: g.shift(1).position - vtype.length - g.position)
        .query("id > 0")
        .groupby("id")
        .gap
    )
    # all followers maintain a constant gap during the entire simulation
    assert follower_gaps.apply(lambda x: np.isclose(x.std(), 0)).all()
    # the gap matches the target gap of cacc spacing
    assert follower_gaps.apply(lambda x: np.isclose(x.mean(), cacc_spacing)).all()

    # the platoon maintains its order through the whole simulation
    assert (
        traces
        .groupby("step")
        .apply(lambda g: g.sort_values("position", ascending=False))
        .reset_index(drop=True)
        .groupby("step")
        .id
        .apply(lambda x: x == np.sort(traces.id.unique()))
        .all()
    )


@pytest.mark.parametrize("size", PLATOON_SIZE)
@pytest.mark.parametrize("cacc_spacing", CACC_SPACING)
@pytest.mark.parametrize("headway_time", HEADWAY_TIME)
@pytest.mark.parametrize("slow_speed", SPEED)
def test_cf_model_CACC_blocked(
    size: int, cacc_spacing: float, headway_time: float, slow_speed: float
):
    """
    A platoon driving on one lane and one slower vehicle in front.
    """

    # create simulation environment
    s = Simulator(
        number_of_lanes=1,
        number_of_vehicles=size,
        penetration_rate=1,
        record_vehicle_traces=True,
        result_base_filename=f"test_cf_model_CACC_blocked_size{size}_spacing{cacc_spacing}_headway{headway_time}_speed{slow_speed}",
        max_step=100,
        start_as_platoon=True,
        pre_fill=True,
        random_desired_speed=False,
        update_desired_speed=False,
        depart_desired=True,
        desired_speed=slow_speed + 10,
        max_desired_speed=slow_speed + 20,
        cacc_spacing=cacc_spacing,
        acc_headway_time=headway_time,
    )

    # add preceding (slow) vehicle
    s._add_vehicle(
        vid=size,
        vtype=vtype,
        depart_position=200,
        arrival_position=s.road_length,
        desired_speed=slow_speed,
        depart_lane=0,
        depart_speed=slow_speed,
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
    predecessor = traces.query(f"id == {size}").set_index("step")

    # leader ACC
    assert (leader.cfModel == CF_Model.ACC.name).all()
    # follower CACC
    assert (followers.cfModel == CF_Model.CACC.name).all()

    # all platoon followers have the same speed as the leader
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
    # predecessor has the same speed in the entire simulation
    assert predecessor.speed.std() == 0
    # (constant) speed of vehicle 0
    speed_predecessor = predecessor.loc[predecessor.reset_index().step.min(), "speed"]

    # the first step the leader brakes
    first_step_braking = leader[leader.speed.pct_change() < 0].reset_index().step.min()
    # the vehicles are not blocked until starting to brake
    assert not traces.query(f"step < {first_step_braking}").blocked.any()

    # the leader remains blocked from braking until the end
    assert leader.reset_index().query(f"step >= {first_step_braking}").blocked.all()
    # the followers are never blocked
    assert not followers.blocked.any()

    # the first step the leader (approx.) reaches the speed of the slow vehicle
    first_step_speed_vehicle = (
        leader
        .query(f"abs(speed - {speed_predecessor}) < 0.01")
        .reset_index()
        .step
        .min()
    )
    # the speed is reached (approx.) at all
    assert not np.isnan(first_step_speed_vehicle)

    # the platoon sticks with the (slow) speed
    assert (
        traces.query(f"id == 0 and step >= {first_step_speed_vehicle}").speed.std()
        < 0.01
    ).all()

    # correct order of vehicles at the end
    assert (leader.position < predecessor.position).all()
