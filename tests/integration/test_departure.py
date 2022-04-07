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
import numpy as np
import pandas as pd
import pytest

from plafosim.simulator import Simulator

MARKER_SYSTEM_EXIT = pytest.mark.xfail(raises=SystemExit, strict=True)
VEHICLES = [
    pytest.param(0, marks=MARKER_SYSTEM_EXIT),
    *range(1, 5),
]
INTERVAL = [
    pytest.param(0, marks=MARKER_SYSTEM_EXIT),
    *range(2, 5),
]
RATE = [
    pytest.param(0, marks=MARKER_SYSTEM_EXIT),
    *range(300, 1800, 300),
]
PROBABILITY = [
    pytest.param(0, marks=MARKER_SYSTEM_EXIT),
    *np.arange(0.25, 1.0, 0.25),
]


@pytest.mark.parametrize("vehicles", VEHICLES)
@pytest.mark.parametrize("interval", INTERVAL)
def test_depart_method_interval(vehicles: int, interval: int, step_length: int = 1):
    """
    """

    # create simulation environment
    s = Simulator(
        number_of_vehicles=vehicles,
        number_of_lanes=1,
        result_base_filename=f"test_depart_method_interval_vehicles{vehicles}_interval{interval}",
        record_vehicle_trips=True,
        road_length=1000,
        ramp_interval=1000,
        random_desired_speed=False,
        desired_speed=30,
        random_depart_lane=False,
        random_depart_speed=False,
        random_depart_position=False,
        depart_desired=True,
        depart_method="interval",
        depart_interval=interval,
        depart_flow=False,
        step_length=step_length,
    )

    # run the simulation to record the trace file
    s.run()

    # read vehicle trace file
    trips = pd.read_csv(f"{s._result_base_filename}_vehicle_trips.csv").sort_values("id", ascending=True)
    assert not trips.empty

    # correct number of vehicles
    assert trips.id.count() == vehicles
    # increasing id
    assert trips.id.is_monotonic
    # increasing depart time
    assert trips.depart.is_monotonic

    # correct depart interval
    assert ((trips.depart - trips.shift(1).depart).dropna() == interval).all()
    # correct depart position (vehicle length)
    assert (trips.departPos == 4).all()
    # correct depart lane
    assert (trips.departLane == 0).all()
    # correct depart speed
    assert (trips.departSpeed == 30).all()


@pytest.mark.parametrize("interval", INTERVAL)
def test_depart_method_interval_flow(interval: int, step_length: int = 1):
    """
    """

    # create simulation environment
    s = Simulator(
        number_of_lanes=1,
        result_base_filename=f"test_depart_method_interval_flow_interval{interval}",
        record_vehicle_traces=True,
        road_length=1000,
        ramp_interval=1000,
        random_desired_speed=False,
        desired_speed=30,
        random_depart_lane=False,
        random_depart_speed=False,
        random_depart_position=False,
        depart_desired=True,
        depart_method="interval",
        depart_interval=interval,
        depart_flow=True,
        max_step=180,
        step_length=step_length,
    )

    # run the simulation to record the trace file
    s.run()

    # read vehicle trace file
    traces = pd.read_csv(f"{s._result_base_filename}_vehicle_traces.csv").sort_values("id", ascending=True)
    assert not traces.empty

    # calculate depature of all vehicles
    spawn_step = traces.groupby('id')['step'].min()
    departure = traces.set_index(['id', 'step']).loc[list(zip(spawn_step.index, spawn_step.values))].reset_index().sort_values('id')

    # correct number of vehicles
    roughly = s._max_step / interval
    assert np.floor(roughly) <= departure.id.count() == np.ceil(roughly)
    # increasing id
    assert departure.id.is_monotonic
    # increasing depart time
    assert departure.step.is_monotonic

    # correct depart interval
    assert ((departure.step - departure.shift(1).step).dropna() == interval).all()
    # correct depart position (vehicle length)
    assert (departure.position == 4).all()
    # correct depart lane
    assert (departure.lane == 0).all()
    # correct depart speed
    assert (departure.speed == 30).all()


@pytest.mark.parametrize("vehicles", VEHICLES)
@pytest.mark.parametrize("rate", RATE)
def test_depart_method_rate(vehicles: int, rate: int, step_length: int = 1):
    """
    """

    # create simulation environment
    s = Simulator(
        number_of_vehicles=vehicles,
        number_of_lanes=1,
        result_base_filename=f"test_depart_method_rate_vehicles{vehicles}_rate{rate}",
        record_vehicle_trips=True,
        road_length=1000,
        ramp_interval=1000,
        random_desired_speed=False,
        desired_speed=30,
        random_depart_lane=False,
        random_depart_speed=False,
        random_depart_position=False,
        depart_desired=True,
        depart_method="rate",
        depart_rate=rate,
        depart_flow=False,
        step_length=step_length,
    )

    # run the simulation to record the trace file
    s.run()

    # read vehicle trace file
    trips = pd.read_csv(f"{s._result_base_filename}_vehicle_trips.csv").sort_values("id", ascending=True)
    assert not trips.empty

    # correct number of vehicles
    assert trips.id.count() == vehicles
    # increasing id
    assert trips.id.is_monotonic
    # increasing depart time
    assert trips.depart.is_monotonic

    # correct depart interval
    interval = 3600 / rate
    assert ((trips.depart - trips.shift(1).depart).dropna() >= int(interval)).all()
    # correct depart position (vehicle length)
    assert (trips.departPos == 4).all()
    # correct depart lane
    assert (trips.departLane == 0).all()
    # correct depart speed
    assert (trips.departSpeed == 30).all()


@pytest.mark.parametrize("rate", RATE)
def test_depart_method_rate_flow(rate: int, step_length: int = 1):
    """
    """

    # create simulation environment
    s = Simulator(
        number_of_lanes=1,
        result_base_filename=f"test_depart_method_rate_flow_rate{rate}",
        record_vehicle_traces=True,
        road_length=1000,
        ramp_interval=1000,
        random_desired_speed=False,
        desired_speed=30,
        random_depart_lane=False,
        random_depart_speed=False,
        random_depart_position=False,
        depart_desired=True,
        depart_method="rate",
        depart_rate=rate,
        depart_flow=True,
        max_step=180,
        step_length=step_length,
    )

    # run the simulation to record the trace file
    s.run()

    # read vehicle trace file
    traces = pd.read_csv(f"{s._result_base_filename}_vehicle_traces.csv").sort_values("id", ascending=True)
    assert not traces.empty

    # calculate depature of all vehicles
    spawn_step = traces.groupby('id')['step'].min()
    departure = traces.set_index(['id', 'step']).loc[list(zip(spawn_step.index, spawn_step.values))].reset_index().sort_values('id')

    # correct number of vehicles
    assert departure.id.count() == int(s._max_step / 3600 * rate)
    # increasing id
    assert departure.id.is_monotonic
    # increasing depart time
    assert departure.step.is_monotonic

    # correct depart interval
    interval = 3600 / rate
    assert ((departure.step - departure.shift(1).step).dropna() >= int(interval)).all()
    # correct depart position (vehicle length)
    assert (departure.position == 4).all()
    # correct depart lane
    assert (departure.lane == 0).all()
    # correct depart speed
    assert (departure.speed == 30).all()


@pytest.mark.parametrize("vehicles", VEHICLES)
@pytest.mark.parametrize("probability", PROBABILITY)
def test_depart_method_probability(vehicles: int, probability: float, step_length: int = 1):
    """
    """

    # create simulation environment
    s = Simulator(
        number_of_vehicles=vehicles,
        number_of_lanes=1,
        result_base_filename=f"test_depart_method_probability_vehicles{vehicles}_probability{probability}",
        record_vehicle_trips=True,
        road_length=1000,
        ramp_interval=1000,
        random_desired_speed=False,
        desired_speed=30,
        random_depart_lane=False,
        random_depart_speed=False,
        random_depart_position=False,
        depart_desired=True,
        depart_method="probability",
        depart_probability=probability,
        depart_flow=False,
        step_length=step_length,
    )

    # run the simulation to record the trace file
    s.run()

    # read vehicle trace file
    trips = pd.read_csv(f"{s._result_base_filename}_vehicle_trips.csv").sort_values("id", ascending=True)
    assert not trips.empty

    # correct number of vehicles
    assert trips.id.count() == vehicles
    # increasing id
    assert trips.id.is_monotonic
    # increasing depart time
    assert trips.depart.is_monotonic

    # correct depart position (vehicle length)
    assert (trips.departPos == 4).all()
    # correct depart lane
    assert (trips.departLane == 0).all()
    # correct depart speed
    assert (trips.departSpeed == 30).all()

# TODO fix these tests
#@pytest.mark.parametrize("probability", PROBABILITY)
#def test_depart_method_probability_flow(probability: float, step_length: int = 1):
#    """
#    """
#
#    # create simulation environment
#    s = Simulator(
#        number_of_lanes=1,
#        result_base_filename=f"test_depart_method_probability_flow_probability{probability}",
#        record_vehicle_traces=True,
#        road_length=1000,
#        ramp_interval=1000,
#        random_desired_speed=False,
#        desired_speed=30,
#        random_depart_lane=False,
#        random_depart_speed=False,
#        random_depart_position=False,
#        depart_desired=True,
#        depart_method="probability",
#        depart_probability=probability,
#        depart_flow=True,
#        max_step=180,
#        step_length=step_length,
#    )
#
#    # run the simulation to record the trace file
#    s.run()
#
#    # read vehicle trace file
#    traces = pd.read_csv(f"{s._result_base_filename}_vehicle_traces.csv").sort_values("id", ascending=True)
#    assert not traces.empty
#
#    # calculate depature of all vehicles
#    spawn_step = traces.groupby('id')['step'].min()
#    departure = traces.set_index(['id', 'step']).loc[list(zip(spawn_step.index, spawn_step.values))].reset_index().sort_values('id')
#
#    # correct number of vehicles
#    assert departure.id.count() == int(s._max_step * probability)
#    # increasing id
#    assert departure.id.is_monotonic
#    # increasing depart time
#    assert departure.step.is_monotonic
#
#    # correct depart position (vehicle length)
#    assert (departure.position == 4).all()
#    # correct depart lane
#    assert (departure.lane == 0).all()
#    # correct depart speed
#    assert (departure.speed == 30).all()

# TODO multiple ramps for all


# TODO depart method number
#@pytest.mark.parametrize("vehicles", VEHICLES)
#def test_depart_method_number(vehicles: int, step_length: int = 1):
#    """
#    """
#
#    vehicles *= 100
#
#    # create simulation environment
#    s = Simulator(
#        number_of_vehicles=vehicles,
#        number_of_lanes=1,
#        result_base_filename=f"test_depart_method_number_vehicles{vehicles}",
#        record_vehicle_trips=True,
#        road_length=1000,
#        ramp_interval=1000,
#        random_desired_speed=False,
#        desired_speed=30,
#        random_depart_lane=False,
#        random_depart_speed=False,
#        random_depart_position=False,
#        depart_desired=True,
#        depart_method="number",
#        depart_flow=True,
#        step_length=step_length,
#        max_step=1800,
#    )
#
#    # run the simulation to record the trace file
#    s.run()
#
#    # read vehicle trace file
#    trips = pd.read_csv(f"{s._result_base_filename}_vehicle_traces.csv").sort_values("id", ascending=True)
#    assert not trips.empty
#    trips = trips.groupby('id').first
#
#    # correct number of vehicles
#    assert trips.id.count() == vehicles
#    # increasing id
#    assert trips.id.is_monotonic
#    # increasing depart time
#    assert trips.depart.is_monotonic
#
#    # correct depart interval
#    interval = s._max_step / vehicles
#    assert ((trips.depart - trips.shift(1).depart).dropna() >= int(interval)).all()
#    # correct depart position (vehicle length)
#    assert (trips.departPos == 4).all()
#    # correct depart lane
#    assert (trips.departLane == 0).all()
#    # correct depart speed
#    assert (trips.departSpeed == 30).all()
