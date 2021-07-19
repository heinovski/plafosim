#!/usr/bin/env python3
#
# Copyright (c) 2020-2021 Julian Heinovski <heinovski@ccs-labs.org>
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

import argparse
import logging
from distutils.util import strtobool
from timeit import default_timer as timer

from src.plafosim import Simulator
from src.plafosim import VERSION


class CustomFormatter(
    argparse.ArgumentDefaultsHelpFormatter,
    argparse.RawDescriptionHelpFormatter,
    argparse.MetavarTypeHelpFormatter,
):
    """Metaclass combining multiple formatter classes for argparse"""

    pass


def main():

    # parse some parameters
    parser = argparse.ArgumentParser(
        formatter_class=CustomFormatter,
        allow_abbrev=False,
        description="""
    Platoon Formation Simulator (PlaFoSim) -- A simple simulator for platoon formation.

    Copyright (c) 2020-2021 Julian Heinovski <heinovski@ccs-labs.org>
    This program comes with ABSOLUTELY NO WARRANTY.
    This is free software, and you are welcome to redistribute it under certain conditions.
    """,
    )

    # miscellaneous
    parser.add_argument(
        "--version",
        action="version",
        version=f"%(prog)s {VERSION}"
    )

    # road network properties
    road = parser.add_argument_group("road network properties")
    road.add_argument(
        "--road-length",
        type=int,
        default=100,
        help="The length of the road in km",
    )
    road.add_argument(
        "--lanes",
        type=int,
        default=3,
        help="The number of lanes",
    )
    road.add_argument(
        "--ramp-interval",
        type=int,
        default=5000,
        help="The distance between any two on-/off-ramps in m",
    )
    road.add_argument(
        "--pre-fill",
        type=lambda x: bool(strtobool(x)),
        default=False,
        choices=(True, False),
        help="Whether to fill the road network with vehicles using random positions and given vehicle number/density before the simulation starts",
    )

    # vehicle properties
    vehicle = parser.add_argument_group("vehicle properties")
    vehicle.add_argument(
        "--vehicles",
        type=int,
        default=100,
        help="The (maximum) number of vehicles that are in the simulation at once",
    )
    vehicle.add_argument(
        "--density",
        type=float,
        default=0,
        help="The (maximum) density (i.e., number of vehicles per km per lane) of vehicles that are in the simulation at once. Overrides --vehicles.",
    )
    vehicle.add_argument(
        "--max-speed",
        type=float,
        default=55,
        help="The maximum possible driving speed in m/s",
    )
    vehicle.add_argument(
        "--acc-headway-time",
        type=float,
        default=1.0,
        help="The headway time to be used for the ACC in s",
    )
    vehicle.add_argument(
        "--cacc-spacing",
        type=float,
        default=5.0,
        help="The constant spacing to be used for the CACC in m",
    )
    vehicle.add_argument(
        "--penetration",
        type=float,
        default=1.0,
        help="Penetration rate of vehicles with platooning capabilities",
    )

    # trip properties
    trip = parser.add_argument_group("trip properties")
    trip.add_argument(
        "--random-depart-position",
        type=lambda x: bool(strtobool(x)),
        default=False,
        choices=(True, False),
        help="Whether to use a random depart position for every vehicle instead of 0 m",
    )
    trip.add_argument(
        "--random-depart-lane",
        type=lambda x: bool(strtobool(x)),
        default=False,
        choices=(True, False),
        help="Whether to use a random depart lane for every vehicle instead of lane 0",
    )
    trip.add_argument(
        "--desired-speed",
        type=float,
        default=36.0,
        help="The desired driving speed im m/s",
    )
    trip.add_argument(
        "--random-desired-speed",
        type=lambda x: bool(strtobool(x)),
        default=True,
        choices=(True, False),
        help="Whether to pick a random (normally distributed) desired driving speed",
    )
    trip.add_argument(
        "--speed-variation",
        type=float,
        default=0.1,
        help="The deviation from the desired driving speed in ratio",
    )
    trip.add_argument(
        "--min-desired-speed",
        type=float,
        default=22.0,
        help="The minimum desired driving speed im m/s",
    )
    trip.add_argument(
        "--max-desired-speed",
        type=float,
        default=50.0,
        help="The maximum desired driving speed im m/s",
    )
    trip.add_argument(
        "--random-depart-speed",
        type=lambda x: bool(strtobool(x)),
        default=False,
        choices=(True, False),
        help="Whether to use a random depart speed for every vehicle instead of 0 m/s",
    )
    trip.add_argument(
        "--depart-desired",
        type=lambda x: bool(strtobool(x)),
        default=False,
        choices=(True, False),
        help="Whether the vehicle should depart with its desired speed. Overrides --random-depart-speed",
    )
    trip.add_argument(
        "--depart-flow",
        type=lambda x: bool(strtobool(x)),
        default=False,
        choices=(True, False),
        help="Whether to spawn vehicles in a continuous flow",
    )
    trip.add_argument(
        "--depart-method",
        type=str,
        choices=("interval", "probability", "rate", "number"),
        default="interval",
        help="The departure method of vehicles",
    )
    trip.add_argument(
        "--depart-time-interval",
        type=int,
        default=1,
        help="The interval between two vehicle departures in s for depart method 'interval'",
    )
    trip.add_argument(
        "--depart-probability",
        type=float,
        default=1.0,
        help="The probability of departure per time step for depart method 'probability'",
    )
    trip.add_argument(
        "--depart-rate",
        type=int,
        default=3600,
        help="The rate of departure in vehicles per hour for depart method 'rate'",
    )
    trip.add_argument(
        "--random-arrival-position",
        type=lambda x: bool(strtobool(x)),
        default=False,
        choices=(True, False),
        help="Whether to use a random arrival position for every vehicle instead of the end of the road",
    )
    trip.add_argument(
        "--minimum-trip-length",
        type=int,
        default=0,
        help="The minimum trip length for a vehicle in km",
    )
    trip.add_argument(
        "--maximum-trip-length",
        type=int,
        default=-1,
        help="The maximum trip length for a vehicle in km",
    )

    # communication properties
    communication = parser.add_argument_group("communication properties")
    communication.add_argument(
        "--communication-range",
        type=int,
        default=1000,
        help="The maximum communication range between two vehicles in m. A value of -1 disables the communication range check",
    )

    # platoon properties
    platoon = parser.add_argument_group("platoon properties")
    platoon.add_argument(
        "--start-as-platoon",
        type=lambda x: bool(strtobool(x)),
        default=False,
        choices=(True, False),
        help="Whether vehicles should automatically start as one platoon",
    )
    platoon.add_argument(
        "--reduced-air-drag",
        type=lambda x: bool(strtobool(x)),
        default=True,
        choices=(True, False),
        help="Whether the reduced air drag due to platooning should be considered in the emissions calculation",
    )
    platoon.add_argument(
        "--maximum-teleport-distance",
        type=int,
        default=2000,
        help="The maximum teleport distance in m. A value of -1 disables the check",
    )
    platoon.add_argument(
        "--maximum-approach-time",
        type=int,
        default=60,
        help="The maximum time for approaching a platoon during a join maneuver in s. A value of -1 disables the check",
    )
    platoon.add_argument(
        "--delay-teleports",
        type=lambda x: bool(strtobool(x)),
        default=True,
        choices=(True, False),
        help="Whether teleports (i.e., during a join maneuver) should be delayed by the time for approaching the target platoon",
    )
    platoon.add_argument(
        "--update-desired-speed",
        type=lambda x: bool(strtobool(x)),
        default=True,
        choices=(True, False),
        help="Whether to update the platoon's desired driving speed to the average speed of all members after the formation changed",
    )

    # formation properties
    formation = parser.add_argument_group("formation properties")
    formation.add_argument(
        "--formation-algorithm",
        type=str,
        default=None,
        choices=["speedposition"],
        help="The formation algorithm to use",
    )
    formation.add_argument(
        "--formation-strategy",
        type=str,
        default="distributed",
        choices=["distributed", "centralized"],
        help="The formation strategy to use",
    )
    formation.add_argument(
        "--formation-centralized-kind",
        type=str,
        default="greedy",
        choices=["greedy", "optimal"],
        help="The kind of the centralized formation",
    )
    formation.add_argument(
        "--execution-interval",
        type=int,
        default=60,
        help="The interval between two iterations of a formation algorithm in s",
    )
    formation.add_argument(
        "--alpha",
        type=float,
        default=0.5,
        help="The weight of the speed deviation in comparison to the position deviation",
    )
    formation.add_argument(
        "--speed-deviation-threshold",
        type=float,
        default=-1,
        help="The maximum allowed (relative) deviation from the desired speed for considering neighbors as candidates. A value of -1 disables the threshold",
    )
    formation.add_argument(
        "--position-deviation-threshold",
        type=int,
        default=2000,
        help="The maximum allowed absolute deviation from the current position for considering neighbors as candidates. A value of -1 disables the threshold",
    )
    formation.add_argument(
        "--solver-time-limit",
        type=int,
        default=60,
        help="The time limit for the optimal solver per assignment problem in s. Influences the quality of the solution.",
    )

    # infrastructure properties
    infrastructures = parser.add_argument_group("infrastructure properties")
    infrastructures.add_argument(
        "--infrastructures",
        type=int,
        default=0,
        help="The number of infrastructures",
    )

    # simulation properties
    simulation = parser.add_argument_group("simulation properties")
    simulation.add_argument(
        "--step-length",
        type=int, default=1,
        help="The step length in s",
    )
    simulation.add_argument(
        "--time-limit",
        type=float,
        default=1.0,
        help="The simulation limit in h",
    )
    simulation.add_argument(
        "--actions",
        type=lambda x: bool(strtobool(x)),
        default=True,
        choices=(True, False),
        help="Whether to enable actions",
    )
    simulation.add_argument(
        "--lane-changes",
        type=lambda x: bool(strtobool(x)),
        default=True,
        choices=(True, False),
        help="Whether to enable lane changes",
    )
    simulation.add_argument(
        "--collisions",
        type=lambda x: bool(strtobool(x)),
        default=True,
        choices=(True, False),
        help="Whether to enable collision checks",
    )
    simulation.add_argument(
        "--random-seed",
        type=int,
        default=-1,
        help="The seed (>=0) for the random number generator instead of the current system time",
    )
    simulation.add_argument(
        "--log-level",
        type=str,
        default="warn",
        choices=["error", "warn", "info", "debug", "trace"],
        help="The minimum level of logs to be printed",
    )

    # gui properties
    gui = parser.add_argument_group("gui properties")
    gui.add_argument(
        "--gui",
        action="store_true",
        help="Enable a live SUMO GUI"
    )
    gui.add_argument(
        "--gui-delay",
        type=int,
        default=0,
        help="The delay used in every simulation step to visualize the current network state in ms",
    )
    gui.add_argument(
        "--track-vehicle",
        type=int,
        default=-1,
        help="The id of a vehicle to track in the gui",
    )
    gui.add_argument(
        "--sumo-config",
        type=str,
        default="sumocfg/freeway.sumo.cfg",
        help="The name of the SUMO config file",
    )

    # result recording properties
    results = parser.add_argument_group("result recording properties")
    results.add_argument(
        "--result-base-filename",
        type=str,
        default="results",
        help="The base filename of the result files",
    )
    results.add_argument(
        "--record-simulation-trace",
        type=lambda x: bool(strtobool(x)),
        default=False,
        choices=(True, False),
        help="Whether to record a continuous simulation trace",
    )
    results.add_argument(
        "--record-end-trace",
        type=lambda x: bool(strtobool(x)),
        default=True,
        choices=(True, False),
        help="Whether to record another trace item at the trip end",
    )
    results.add_argument(
        "--record-vehicle-trips",
        type=lambda x: bool(strtobool(x)),
        default=False,
        choices=(True, False),
        help="Whether to record vehicle trips",
    )
    results.add_argument(
        "--record-vehicle-emissions",
        type=lambda x: bool(strtobool(x)),
        default=False,
        choices=(True, False),
        help="Whether to record vehicle emissions",
    )
    results.add_argument(
        "--record-vehicle-traces",
        type=lambda x: bool(strtobool(x)),
        default=False,
        choices=(True, False),
        help="Whether to record continuous vehicles traces",
    )
    results.add_argument(
        "--record-vehicle-changes",
        type=lambda x: bool(strtobool(x)),
        default=False,
        choices=(True, False),
        help="Whether to record vehicle lane changes",
    )
    results.add_argument(
        "--record-emission-traces",
        type=lambda x: bool(strtobool(x)),
        default=False,
        choices=(True, False),
        help="Whether to record continuous emission traces",
    )
    results.add_argument(
        "--record-platoon-trips",
        type=lambda x: bool(strtobool(x)),
        default=False,
        choices=(True, False),
        help="Whether to record platoon trips",
    )
    results.add_argument(
        "--record-platoon-maneuvers",
        type=lambda x: bool(strtobool(x)),
        default=False,
        choices=(True, False),
        help="Whether to record platoon maneuvers",
    )
    results.add_argument(
        "--record-platoon-formation",
        type=lambda x: bool(strtobool(x)),
        default=False,
        choices=(True, False),
        help="Whether to record platoon formation results",
    )
    results.add_argument(
        "--record-platoon-traces",
        type=lambda x: bool(strtobool(x)),
        default=False,
        choices=(True, False),
        help="Whether to record continuous platoon traces",
    )
    results.add_argument(
        "--record-platoon-changes",
        type=lambda x: bool(strtobool(x)),
        default=False,
        choices=(True, False),
        help="Whether to record platoon lane changes",
    )
    results.add_argument(
        "--record-infrastructure-assignments",
        type=lambda x: bool(strtobool(x)),
        default=False,
        choices=(True, False),
        help="Whether to record infrastructure assignments",
    )
    results.add_argument(
        "--record-prefilled",
        type=lambda x: bool(strtobool(x)),
        default=False,
        choices=(True, False),
        help="Whether to record results for pre-filled vehicles",
    )

    args = parser.parse_args()

    start_time = timer()

    simulator = Simulator(
        args.road_length * 1000,
        args.lanes,
        args.ramp_interval,
        args.pre_fill,
        args.vehicles,
        args.density,
        args.max_speed,
        args.acc_headway_time,
        args.cacc_spacing,
        args.penetration,
        args.random_depart_position,
        args.random_depart_lane,
        args.desired_speed,
        args.random_desired_speed,
        args.speed_variation,
        args.min_desired_speed,
        args.max_desired_speed,
        args.random_depart_speed,
        args.depart_desired,
        args.depart_flow,
        args.depart_method,
        args.depart_time_interval,
        args.depart_probability,
        args.depart_rate,
        args.random_arrival_position,
        args.minimum_trip_length * 1000,
        args.maximum_trip_length * 1000,
        args.communication_range,
        args.start_as_platoon,
        args.reduced_air_drag,
        args.maximum_teleport_distance,
        args.maximum_approach_time,
        args.delay_teleports,
        args.update_desired_speed,
        args.formation_algorithm,
        args.formation_strategy,
        args.formation_centralized_kind,
        args.execution_interval,
        args.alpha,
        args.speed_deviation_threshold,
        args.position_deviation_threshold,
        args.solver_time_limit * 1000,
        args.infrastructures,
        args.step_length,
        round(args.time_limit * 60 * 60),
        args.actions,
        args.lane_changes,
        args.collisions,
        args.random_seed,
        getattr(logging, args.log_level.upper(), 5),  # implicitly use trace level
        args.gui,
        args.gui_delay / 1000,
        args.track_vehicle,
        args.sumo_config,
        args.result_base_filename,
        args.record_simulation_trace,
        args.record_end_trace,
        args.record_vehicle_trips,
        args.record_vehicle_emissions,
        args.record_vehicle_traces,
        args.record_vehicle_changes,
        args.record_emission_traces,
        args.record_platoon_trips,
        args.record_platoon_maneuvers,
        args.record_platoon_formation,
        args.record_platoon_traces,
        args.record_platoon_changes,
        args.record_infrastructure_assignments,
        args.record_prefilled,
    )

    steps = simulator.run()

    end_time = timer()
    run_time = end_time - start_time

    print(f"The simulation took {run_time} seconds ({(steps / run_time)} step/s)")


if __name__ == "__main__":
    main()
