#!/usr/bin/env python3
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

import argparse
import logging
import pickle
import sys
import textwrap
from distutils.util import strtobool
from timeit import default_timer as timer

from plafosim import __version__
from plafosim.algorithms.speed_position import SpeedPosition
from plafosim.simulator import DEFAULTS, Simulator
from plafosim.util import find_resource


class CustomFormatter(
    argparse.ArgumentDefaultsHelpFormatter,
    argparse.RawDescriptionHelpFormatter,
    argparse.MetavarTypeHelpFormatter,
):
    """Metaclass combining multiple formatter classes for argparse"""

    pass


# TODO duplicated code with trace replay
def parse_args() -> (argparse.Namespace, argparse._ArgumentGroup):

    # parse some parameters
    parser = argparse.ArgumentParser(
        formatter_class=CustomFormatter,
        allow_abbrev=False,
        description=textwrap.dedent(f"""\
            Platoon Formation Simulator (PlaFoSim) -- Version {__version__}.
            A simple and scalable simulator for platoon formation.

            Copyright (c) 2020-2022 Julian Heinovski <heinovski@ccs-labs.org>
            This program comes with ABSOLUTELY NO WARRANTY.
            This is free software, and you are welcome to redistribute it under certain conditions.

            If you are working with PlaFoSim, please cite the following paper:

            Julian Heinovski, Dominik S. Buse and Falko Dressler,
            "Scalable Simulation of Platoon Formation Maneuvers with PlaFoSim,"
            Proceedings of 13th IEEE Vehicular Networking Conference (VNC 2021),
            Poster Session, Virtual Conference, November 2021.
        """),
    )

    # miscellaneous
    parser.add_argument(
        "-C", "--citation",
        action="version",
        help="show the citation information and exit",
        version=textwrap.dedent("""
            @inproceedings{heinovski2021scalable,
                author = {Heinovski, Julian and Buse, Dominik S. and Dressler, Falko},
                title = {{Scalable Simulation of Platoon Formation Maneuvers with PlaFoSim}},
                publisher = {IEEE},
                issn = {2157-9865},
                isbn = {978-1-66544-450-7},
                address = {Virtual Conference},
                booktitle = {13th IEEE Vehicular Networking Conference (VNC 2021), Poster Session},
                month = {11},
                year = {2021},
            }
        """),
    )
    parser.add_argument(
        "-V", "--version",
        action="version",
        version=f"%(prog)s {__version__}",
    )
    parser.add_argument(
        "-d", "--default",
        action="store_true",
        help="run a simulation with the default configuration and exit",
    )
    parser.add_argument(
        "-n", "--dry-run",
        action="store_true",
        help="show the current configuration and exit",
    )
    parser.add_argument(
        "--save-snapshot",
        type=str,
        default=None,
        metavar="FILE",
        help="Save a snapshot of the scenario to FILE and exit",
    )
    parser.add_argument(
        "--load-snapshot",
        type=str,
        default=None,
        metavar="FILE",
        help="Load a snapshot of the scenario from FILE and run the simulation",
    )

    # road network properties
    road = parser.add_argument_group("road network properties")
    road.add_argument(
        "--road-length",
        type=int,
        default=int(DEFAULTS['road_length'] / 1000),  # m -> km
        help="The length of the road in km",
    )
    road.add_argument(
        "--lanes",
        type=int,
        dest='number_of_lanes',
        default=DEFAULTS['lanes'],
        help="The number of lanes",
    )
    road.add_argument(
        "--ramp-interval",
        type=int,
        default=int(DEFAULTS['ramp_interval'] / 1000),  # m -> km
        help="The distance between any two on-/off-ramps in km",
    )
    road.add_argument(
        "--pre-fill",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['pre_fill'],
        choices=(True, False),
        help="Whether to fill the road network with vehicles using random positions and given vehicle number/density before the simulation starts",
    )

    # vehicle properties
    vehicle = parser.add_argument_group("vehicle properties")
    vehicle.add_argument(
        "--vehicles",
        type=int,
        dest='number_of_vehicles',
        default=DEFAULTS['vehicles'],
        help="The (maximum) number of vehicles that are in the simulation at once. Is used for pre-fill, without a depart flow, and for some depart methods. A value of -1 disables this value.",
    )
    vehicle.add_argument(
        "--density",
        type=float,
        dest='vehicle_density',
        default=DEFAULTS['vehicle_density'],
        help="The (maximum) density (i.e., number of vehicles per km per lane) of vehicles that are in the simulation at once. Overrides --vehicles but behaves similarly. A value of -1 disables this value",
    )
    vehicle.add_argument(
        "--max-speed",
        type=float,
        default=DEFAULTS['max_speed'],
        help="The maximum possible driving speed in m/s",
    )
    vehicle.add_argument(
        "--acc-headway-time",
        type=float,
        default=DEFAULTS['acc_headway_time'],
        help="The headway time to be used for the ACC in s",
    )
    vehicle.add_argument(
        "--cacc-spacing",
        type=float,
        default=DEFAULTS['cacc_spacing'],
        help="The constant spacing to be used for the CACC in m",
    )
    vehicle.add_argument(
        "--penetration",
        type=float,
        dest='penetration_rate',
        default=DEFAULTS['penetration_rate'],
        help="Penetration rate of vehicles with platooning capabilities",
    )

    # trip properties
    trip = parser.add_argument_group("trip properties")
    trip.add_argument(
        "--random-depart-position",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['random_depart_position'],
        choices=(True, False),
        help="Whether to use a random depart position for every vehicle instead of 0 m",
    )
    trip.add_argument(
        "--desired-speed",
        type=float,
        default=DEFAULTS['desired_speed'],
        help="The desired driving speed im m/s",
    )
    trip.add_argument(
        "--random-desired-speed",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['random_desired_speed'],
        choices=(True, False),
        help="Whether to pick a random (normally distributed) desired driving speed",
    )
    trip.add_argument(
        "--speed-variation",
        type=float,
        default=DEFAULTS['speed_variation'],
        help="The deviation from the desired driving speed in ratio",
    )
    trip.add_argument(
        "--min-desired-speed",
        type=float,
        default=DEFAULTS['min_desired_speed'],
        help="The minimum desired driving speed im m/s",
    )
    trip.add_argument(
        "--max-desired-speed",
        type=float,
        default=DEFAULTS['max_desired_speed'],
        help="The maximum desired driving speed im m/s",
    )
    trip.add_argument(
        "--random-depart-speed",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['random_depart_speed'],
        choices=(True, False),
        help="Whether to use a random depart speed for every vehicle instead of 0 m/s",
    )
    trip.add_argument(
        "--depart-desired",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['depart_desired'],
        choices=(True, False),
        help="Whether the vehicle should depart with its desired speed. Overrides --random-depart-speed",
    )
    trip.add_argument(
        "--depart-flow",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['depart_flow'],
        choices=(True, False),
        help="Whether to spawn vehicles in a continuous flow or as fixed number of vehicles",
    )
    trip.add_argument(
        "--depart-method",
        type=str,
        choices=("interval", "probability", "rate", "number"),
        default=DEFAULTS['depart_method'],
        help="The departure method of vehicles. Can be limited when depart-flow is disabled.",
    )
    trip.add_argument(
        "--depart-interval",
        type=float,
        default=DEFAULTS['depart_interval'],
        help="The interval between two vehicle departures in s for depart method 'interval'",
    )
    trip.add_argument(
        "--depart-probability",
        type=float,
        default=DEFAULTS['depart_probability'],
        help="The probability of departure per time step for depart method 'probability'",
    )
    trip.add_argument(
        "--depart-rate",
        type=int,
        default=DEFAULTS['depart_rate'],
        help="The rate of departure in vehicles per hour for depart method 'rate'",
    )
    trip.add_argument(
        "--random-arrival-position",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['random_arrival_position'],
        choices=(True, False),
        help="Whether to use a random arrival position for every vehicle instead of the end of the road",
    )
    trip.add_argument(
        "--minimum-trip-length",
        type=int,
        default=int(DEFAULTS['minimum_trip_length'] / 1000),  # m -> km
        help="The minimum trip length for a vehicle in km",
    )
    trip.add_argument(
        "--maximum-trip-length",
        type=int,
        default=int(DEFAULTS['maximum_trip_length'] / 1000),  # m -> km
        help="The maximum trip length for a vehicle in km",
    )

    # communication properties
    communication = parser.add_argument_group("communication properties")
    communication.add_argument(
        "--communication-range",
        type=int,
        default=DEFAULTS['communication_range'],
        help="The maximum communication range between two vehicles in m. A value of -1 disables the communication range check",
    )

    # platoon properties
    platoon = parser.add_argument_group("platoon properties")
    platoon.add_argument(
        "--start-as-platoon",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['start_as_platoon'],
        choices=(True, False),
        help="Whether vehicles should automatically start as one platoon",
    )
    platoon.add_argument(
        "--reduced-air-drag",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['reduced_air_drag'],
        choices=(True, False),
        help="Whether the reduced air drag due to platooning should be considered in the emissions calculation",
    )
    platoon.add_argument(
        "--maximum-teleport-distance",
        type=int,
        default=DEFAULTS['maximum_teleport_distance'],
        help="The maximum teleport distance in m. A value of -1 disables the check",
    )
    platoon.add_argument(
        "--maximum-approach-time",
        type=int,
        default=DEFAULTS['maximum_approach_time'],
        help="The maximum time for approaching a platoon during a join maneuver in s. A value of -1 disables the check",
    )
    platoon.add_argument(
        "--delay-teleports",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['delay_teleports'],
        choices=(True, False),
        help="Whether teleports (i.e., during a join maneuver) should be delayed by the time for approaching the target platoon",
    )
    platoon.add_argument(
        "--update-desired-speed",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['update_desired_speed'],
        choices=(True, False),
        help="Whether to update the platoon's desired driving speed to the average speed of all members after the formation changed",
    )

    # formation properties
    formation = parser.add_argument_group("formation properties")
    formation.add_argument(
        "--formation-algorithm",
        type=str,
        default=DEFAULTS['formation_algorithm'],
        # TODO use enum
        choices=[SpeedPosition.__name__],
        help="The formation algorithm to use",
    )
    formation.add_argument(
        "--formation-strategy",
        type=str,
        default=DEFAULTS['formation_strategy'],
        choices=["distributed", "centralized"],
        help="The formation strategy to use",
    )
    formation.add_argument(
        "--execution-interval",
        type=int,
        default=DEFAULTS['execution_interval'],
        help="The interval between two iterations of a formation algorithm in s",
    )

    # formation algorithm specific properties
    ## speed position
    SpeedPosition.add_parser_argument_group(parser)

    # infrastructure properties
    infrastructures = parser.add_argument_group("infrastructure properties")
    infrastructures.add_argument(
        "--infrastructures",
        type=int,
        dest='number_of_infrastructures',
        default=DEFAULTS['infrastructures'],
        help="The number of infrastructures",
    )

    # simulation properties
    simulation = parser.add_argument_group("simulation properties")
    simulation.add_argument(
        "--step-length",
        type=int,
        default=DEFAULTS['step_length'],
        help="The step length in s",
    )
    simulation.add_argument(
        "--time-limit",
        type=float,
        dest='max_step',
        default=float(DEFAULTS['max_step'] / 3600),  # s -> h
        help="The simulation limit in h",
    )
    simulation.add_argument(
        "--actions",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['actions'],
        choices=(True, False),
        help="Whether to enable actions",
    )
    simulation.add_argument(
        "--collisions",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['collisions'],
        choices=(True, False),
        help="Whether to enable collision checks",
    )
    simulation.add_argument(
        "--random-seed",
        type=int,
        default=DEFAULTS['random_seed'],
        help="The seed (>=0) for the random number generator instead of the current system time",
    )
    simulation.add_argument(
        "--log-level",
        type=str,
        default=logging.getLevelName(DEFAULTS['log_level']).lower(),
        choices=["error", "warn", "info", "debug", "trace"],
        help="The minimum level of logs to be printed",
    )
    simulation.add_argument(
        "--progress",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['progress'],
        choices=(True, False),
        help="Whether to enable the (simulation) progress bar",
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
        default=int(DEFAULTS['gui_delay'] * 1000),  # s -> ms
        help="The delay used in every simulation step to visualize the current network state in ms",
    )
    gui.add_argument(
        "--track-vehicle",
        type=int,
        dest='gui_track_vehicle',
        default=DEFAULTS['gui_track_vehicle'],
        help="The id of a vehicle to track in the gui",
    )
    gui.add_argument(
        "--sumo-config",
        type=find_resource,
        default=DEFAULTS['sumo_config'],
        help="The name of the SUMO config file",
    )
    gui.add_argument(
        "--gui-play",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['gui_play'],
        choices=(True, False),
        help="Whether to start the simulation immediately",
    )
    gui.add_argument(
        "--gui-start",
        type=int,
        default=DEFAULTS['gui_start'],
        help="The time to connect to the GUI in s",
    )
    gui.add_argument(
        "--draw-ramps",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['draw_ramps'],
        choices=(True, False),
        help="Whether to draw on-/off-ramps",
    )
    gui.add_argument(
        "--draw-ramp-labels",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['draw_ramp_labels'],
        choices=(True, False),
        help="Whether to draw labels for on-/off-ramps",
    )
    gui.add_argument(
        "--draw-road-end",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['draw_road_end'],
        choices=(True, False),
        help="Whether to draw the end of the road",
    )
    gui.add_argument(
        "--draw-road-end-label",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['draw_road_end_label'],
        choices=(True, False),
        help="Whether to draw a label for the end of the road",
    )
    gui.add_argument(
        "--draw-infrastructures",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['draw_infrastructures'],
        choices=(True, False),
        help="Whether to draw infrastructures",
    )
    gui.add_argument(
        "--draw-infrastructure-labels",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['draw_infrastructure_labels'],
        choices=(True, False),
        help="Whether to draw labels for infrastructures",
    )

    # result recording properties
    results = parser.add_argument_group("result recording properties")
    results.add_argument(
        "--result-base-filename",
        type=str,
        default=DEFAULTS['result_base_filename'],
        help="The base filename of the result files",
    )
    results.add_argument(
        "--record-simulation-trace",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['record_simulation_trace'],
        choices=(True, False),
        help="Whether to record a continuous simulation trace",
    )
    results.add_argument(
        "--record-end-trace",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['record_end_trace'],
        choices=(True, False),
        help="Whether to record another trace item at the trip end",
    )
    results.add_argument(
        "--record-vehicle-trips",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['record_vehicle_trips'],
        choices=(True, False),
        help="Whether to record vehicle trips",
    )
    results.add_argument(
        "--record-vehicle-emissions",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['record_vehicle_emissions'],
        choices=(True, False),
        help="Whether to record vehicle emissions",
    )
    results.add_argument(
        "--record-vehicle-traces",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['record_vehicle_traces'],
        choices=(True, False),
        help="Whether to record continuous vehicles traces",
    )
    results.add_argument(
        "--record-vehicle-changes",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['record_vehicle_changes'],
        choices=(True, False),
        help="Whether to record vehicle lane changes",
    )
    results.add_argument(
        "--record-emission-traces",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['record_emission_traces'],
        choices=(True, False),
        help="Whether to record continuous emission traces",
    )
    results.add_argument(
        "--record-platoon-trips",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['record_platoon_trips'],
        choices=(True, False),
        help="Whether to record platoon trips",
    )
    results.add_argument(
        "--record-platoon-maneuvers",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['record_platoon_maneuvers'],
        choices=(True, False),
        help="Whether to record platoon maneuvers",
    )
    results.add_argument(
        "--record-platoon-formation",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['record_platoon_formation'],
        choices=(True, False),
        help="Whether to record platoon formation results",
    )
    results.add_argument(
        "--record-platoon-traces",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['record_platoon_traces'],
        choices=(True, False),
        help="Whether to record continuous platoon traces",
    )
    results.add_argument(
        "--record-platoon-changes",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['record_platoon_changes'],
        choices=(True, False),
        help="Whether to record platoon lane changes",
    )
    results.add_argument(
        "--record-infrastructure-assignments",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['record_infrastructure_assignments'],
        choices=(True, False),
        help="Whether to record infrastructure assignments",
    )
    results.add_argument(
        "--record-prefilled",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['record_prefilled'],
        choices=(True, False),
        help="Whether to record results for pre-filled vehicles",
    )

    # print usage without any arguments
    if len(sys.argv) < 2:
        # no argument has been passed
        print(
            parser.description,
            '\n',
            parser.format_usage(),
            sep='',
            end='',
        )
        sys.exit(0)

    args = parser.parse_args()

    # transform argument values into correct units
    args.road_length *= 1000  # km -> m
    args.ramp_interval *= 1000  # km -> m
    args.minimum_trip_length *= 1000  # km -> m
    args.maximum_trip_length *= 1000  # km -> m
    args.solver_time_limit *= 1000  # s -> ms
    args.max_step *= 3600  # h -> s
    args.gui_delay /= 1000  # ms -> s

    return args, gui


def load_snapshot(snapshot_filename: str) -> Simulator:
    assert snapshot_filename

    with open(snapshot_filename, "rb") as f:
        # load saved state
        simulator = pickle.load(f)
        assert isinstance(simulator, Simulator)

    return simulator


def save_snapshot(simulator: Simulator, snapshot_filename: str):
    assert isinstance(simulator, Simulator)
    assert snapshot_filename

    with open(snapshot_filename, "wb") as f:
        pickle.dump(simulator, f)


def create_simulator(**kwargs: dict) -> Simulator:
    assert kwargs

    # prepare keyword arguments for simulator
    kwargs.pop('load_snapshot')
    kwargs.pop('save_snapshot')
    kwargs['log_level'] = getattr(logging, kwargs['log_level'].upper(), 5)
    kwargs['max_step'] = int(kwargs['max_step'])

    # create new simulator
    return Simulator(**kwargs)


def main():

    # get argument values (and gui argument group)
    args, gui = parse_args()

    if args.default:
        print("Running with default configuration...")

    if args.dry_run:
        print(f"Current configuration:\n{dict(sorted(vars(args).items()))}")
        return

    simulator = None
    if args.load_snapshot:
        # load snapshot
        simulator = load_snapshot(snapshot_filename=args.load_snapshot)

        # allow to override the loaded gui parameters
        simulator.__dict__.update(
            {
                f"_{k}": v
                for k, v in vars(args).items()
                if k in [x.dest for x in gui._group_actions]
            }
        )
        print(f"Loaded a snapshot of the simulation from {args.load_snapshot}. Running simulation with the loaded state...")
    else:
        # create new simulator
        simulator = create_simulator(**vars(args))

    if args.save_snapshot:
        if args.load_snapshot:
            sys.exit("ERROR: Saving a loaded snapshot does not make sense!")
        # save snapshot
        save_snapshot(simulator, snapshot_filename=args.save_snapshot)
        print(f"Saved a snapshot of the simulation to {args.save_snapshot}. Exiting...")
        return

    # execute simulation
    start_time = timer()

    steps = simulator.run()

    end_time = timer()
    run_time = end_time - start_time

    print(f"The simulation took {run_time} seconds ({(steps / run_time)} step/s)")


if __name__ == "__main__":
    sys.exit(main())
