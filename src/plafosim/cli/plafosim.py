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

import argparse
import json
import logging
import pickle
import sys
from distutils.util import strtobool
from signal import SIGINT, signal
from timeit import default_timer as timer

from plafosim import CustomFormatter, __citation__, __description__, __version__
from plafosim.algorithms import *  # noqa 401
from plafosim.simulator import DEFAULTS, Simulator
from plafosim.util import find_resource

__epilog__ = """\
Examples:
  # Configure a 100km freeway with ramps at every 10km
  plafosim --road-length 100 --ramp-interval 10

  # Configure random (normally distributed) desired driving speed of 130km/h
  plafosim --random-desired-speed true --desired-speed 36

  # Configure random trips for 500 vehicles
  plafosim --vehicles 500 --random-depart-position true --random-arrival-position true --depart-desired true

  # Pre fill the freeway with 1000 vehicles
  plafosim --vehicles 1000 --pre-fill true

  # Configure 50% of the vehicles with Advanced Cruise Control (ACC) and a headway time of 1.5s
  plafosim --penetration 0.5 --acc-headway-time 1.5

  # Enable a simple, distributed platoon formation algorithm [1] in order to form platoons every 30s
  plafosim --formation-algorithm SpeedPosition --formation-strategy distributed --execution-interval 30
"""


def format_help(parser: argparse.ArgumentParser, groups=None) -> str:
    """
    Format help message for argument groups.

    Taken from https://stackoverflow.com/a/40730878.
    """

    formatter = parser._get_formatter()

    # usage
    formatter.add_usage(
        parser.usage,
        parser._actions,
        parser._mutually_exclusive_groups,
    )

    # description
    formatter.add_text(parser.description)

    if not groups:
        groups = parser._action_groups

    # positionals, optionals and user-defined groups
    for action_group in groups:
        formatter.start_section(action_group.title)
        formatter.add_text(action_group.description)
        formatter.add_arguments(action_group._group_actions)
        formatter.end_section()

    # determine help from format above
    return formatter.format_help()


# TODO duplicated code with trace replay
def parse_args() -> (argparse.Namespace, argparse._ArgumentGroup):
    """
    Parse arguments given to this module.

    Returns
    -------
    args : argparse.Namespace
        The namespace of parsed arguments and corresponding values.
    g_gui : argparse._ArgumentGroup
        The specific argument group for the GUI properties.
    """

    # parse some parameters
    parser = argparse.ArgumentParser(
        formatter_class=CustomFormatter,
        allow_abbrev=False,
        description=__description__,
        add_help=False,
        epilog=__epilog__,
    )

    # miscellaneous
    parser.add_argument(
        '-h', '--help',
        action='store_true',
        default=argparse.SUPPRESS,
        help='show this help message and exit',
    )
    parser.add_argument(
        "-C", "--citation",
        action="version",
        help="show the citation information (bibtex) and exit",
        version=__citation__,
    )
    parser.add_argument(
        "-V", "--version",
        action="version",
        version=f"plafosim {__version__}",
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
        "-q", "--quiet",
        action="count",
        default=0,
        help=f"The amount of verbosity levels to be removed for printing the logs to stdout. The starting level is {logging.getLevelName(DEFAULTS['log_level'])}.",
    )
    parser.add_argument(
        "-v", "--verbosity",
        action="count",
        default=0,
        help=f"The amount of verbosity levels to be added for printing the logs to stdout. The starting level is {logging.getLevelName(DEFAULTS['log_level'])}.",
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

    # custom help messages
    g_help = parser.add_argument_group("help messages")
    g_help.add_argument(
        "--help-all",
        action='store_true',
        default=argparse.SUPPRESS,
        help="show complete help message and exit",
    )

    # road network properties
    g_road = parser.add_argument_group("road network properties")
    g_help.add_argument(
        "--help-road",
        action='store_true',
        default=argparse.SUPPRESS,
        help="show help message for road network properties and exit",
    )
    g_road.add_argument(
        "--road-length",
        type=int,
        default=int(DEFAULTS['road_length'] / 1000),  # m -> km
        help="The length of the road in km",
    )
    g_road.add_argument(
        "--lanes",
        type=int,
        dest='number_of_lanes',
        default=DEFAULTS['lanes'],
        help="The number of lanes",
    )
    g_road.add_argument(
        "--ramp-interval",
        type=int,
        default=int(DEFAULTS['ramp_interval'] / 1000),  # m -> km
        help="The distance between any two on-/off-ramps in km",
    )
    g_road.add_argument(
        "--pre-fill",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['pre_fill'],
        choices=(True, False),
        help="Whether to fill the road network with vehicles using random positions and given vehicle number/density before the simulation starts",
    )

    # vehicle properties
    g_vehicles = parser.add_argument_group("vehicle properties")
    g_help.add_argument(
        "--help-vehicles",
        action='store_true',
        default=argparse.SUPPRESS,
        help="show help message for vehicle properties and exit",
    )
    g_vehicles.add_argument(
        "--vehicles",
        type=int,
        dest='number_of_vehicles',
        default=DEFAULTS['vehicles'],
        help="The (maximum) number of vehicles that are in the simulation at once. Is used for pre-fill, without a departure flow, and for some departure methods. A value of -1 disables this value.",
    )
    g_vehicles.add_argument(
        "--density",
        type=float,
        dest='vehicle_density',
        default=DEFAULTS['vehicle_density'],
        help="The (maximum) density (i.e., number of vehicles per km per lane) of vehicles that are in the simulation at once. Overrides --vehicles but behaves similarly. A value of -1 disables this value",
    )
    g_vehicles.add_argument(
        "--max-speed",
        type=float,
        default=DEFAULTS['max_speed'],
        help="The maximum possible driving speed in m/s",
    )
    g_vehicles.add_argument(
        "--acc-headway-time",
        type=float,
        default=DEFAULTS['acc_headway_time'],
        help="The headway time to be used for the ACC in s",
    )
    g_vehicles.add_argument(
        "--cacc-spacing",
        type=float,
        default=DEFAULTS['cacc_spacing'],
        help="The constant spacing to be used for the CACC in m",
    )
    g_vehicles.add_argument(
        "--penetration",
        type=float,
        dest='penetration_rate',
        default=DEFAULTS['penetration_rate'],
        help="Penetration rate of vehicles with platooning capabilities",
    )

    # trip properties
    g_trips = parser.add_argument_group("trip properties")
    g_help.add_argument(
        "--help-trips",
        action='store_true',
        default=argparse.SUPPRESS,
        help="show help message for trip properties and exit",
    )
    g_trips.add_argument(
        "--random-depart-position",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['random_depart_position'],
        choices=(True, False),
        help="Whether to use a random departure position for every vehicle instead of 0 m",
    )
    g_trips.add_argument(
        "--depart-all-lanes",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['depart_all_lanes'],
        choices=(True, False),
        help="Whether vehicles are allowed to depart on all lanes"
    )
    g_trips.add_argument(
        "--desired-speed",
        type=float,
        default=DEFAULTS['desired_speed'],
        help="The desired driving speed im m/s",
    )
    g_trips.add_argument(
        "--random-desired-speed",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['random_desired_speed'],
        choices=(True, False),
        help="Whether to pick a random (normally distributed) desired driving speed",
    )
    g_trips.add_argument(
        "--speed-variation",
        type=float,
        default=DEFAULTS['speed_variation'],
        help="The deviation from the desired driving speed in ratio",
    )
    g_trips.add_argument(
        "--min-desired-speed",
        type=float,
        default=DEFAULTS['min_desired_speed'],
        help="The minimum desired driving speed im m/s",
    )
    g_trips.add_argument(
        "--max-desired-speed",
        type=float,
        default=DEFAULTS['max_desired_speed'],
        help="The maximum desired driving speed im m/s",
    )
    g_trips.add_argument(
        "--random-depart-speed",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['random_depart_speed'],
        choices=(True, False),
        help="Whether to use a random departure speed for every vehicle instead of 0 m/s",
    )
    g_trips.add_argument(
        "--depart-desired",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['depart_desired'],
        choices=(True, False),
        help="Whether the vehicle should depart with its desired speed. Overrides --random-depart-speed",
    )
    g_trips.add_argument(
        "--depart-flow",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['depart_flow'],
        choices=(True, False),
        help="Whether to spawn vehicles in a continuous flow or as fixed number of vehicles",
    )
    g_trips.add_argument(
        "--depart-method",
        type=str,
        choices=("interval", "probability", "rate", "number"),
        default=DEFAULTS['depart_method'],
        help="The departure method of vehicles. Can be limited when depart-flow is disabled.",
    )
    g_trips.add_argument(
        "--depart-interval",
        type=float,
        default=DEFAULTS['depart_interval'],
        help="The interval between two vehicles' departures in s for departure method 'interval'",
    )
    g_trips.add_argument(
        "--depart-probability",
        type=float,
        default=DEFAULTS['depart_probability'],
        help="The probability of departure per time step for departure method 'probability'",
    )
    g_trips.add_argument(
        "--depart-rate",
        type=int,
        default=DEFAULTS['depart_rate'],
        help="The rate of departure in vehicles per hour for departure method 'rate'",
    )
    g_trips.add_argument(
        "--random-arrival-position",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['random_arrival_position'],
        choices=(True, False),
        help="Whether to use a random arrival position for every vehicle instead of the end of the road",
    )
    g_trips.add_argument(
        "--minimum-trip-length",
        type=int,
        default=int(DEFAULTS['minimum_trip_length'] / 1000),  # m -> km
        help="The minimum trip length for a vehicle in km",
    )
    g_trips.add_argument(
        "--maximum-trip-length",
        type=int,
        default=int(DEFAULTS['maximum_trip_length'] / 1000),  # m -> km
        help="The maximum trip length for a vehicle in km",
    )

    # communication properties
    g_communication = parser.add_argument_group("communication properties")
    g_help.add_argument(
        "--help-communication",
        action='store_true',
        default=argparse.SUPPRESS,
        help="show help message for communication properties and exit",
    )
    g_communication.add_argument(
        "--communication-range",
        type=int,
        default=DEFAULTS['communication_range'],
        help="The maximum communication range between two vehicles in m. A value of -1 disables the communication range check",
    )
    g_communication.add_argument(
        "--distributed-platoon-knowledge",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['distributed_platoon_knowledge'],
        choices=(True, False),
        help="Whether the distributed approach should have perfect platoon knowledge (e.g., platoon role)."
    )
    g_communication.add_argument(
        "--distributed-maneuver-knowledge",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['distributed_maneuver_knowledge'],
        choices=(True, False),
        help="Whether the distributed approach should have perfect maneuver knowledge."
    )

    # platoon properties
    g_platoon = parser.add_argument_group("platoon properties")
    g_help.add_argument(
        "--help-platoon",
        action='store_true',
        default=argparse.SUPPRESS,
        help="show help message for platoon properties and exit",
    )
    g_platoon.add_argument(
        "--start-as-platoon",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['start_as_platoon'],
        choices=(True, False),
        help="Whether vehicles should automatically start as one platoon",
    )
    g_platoon.add_argument(
        "--reduced-air-drag",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['reduced_air_drag'],
        choices=(True, False),
        help="Whether the reduced air drag due to platooning should be considered in the emissions calculation",
    )
    g_platoon.add_argument(
        "--maximum-teleport-distance",
        type=int,
        default=DEFAULTS['maximum_teleport_distance'],
        help="The maximum teleport distance in m. A value of -1 disables the check",
    )
    g_platoon.add_argument(
        "--maximum-approach-time",
        type=int,
        default=DEFAULTS['maximum_approach_time'],
        help="The maximum time for approaching a platoon during a join maneuver in s. A value of -1 disables the check",
    )
    g_platoon.add_argument(
        "--delay-teleports",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['delay_teleports'],
        choices=(True, False),
        help="Whether teleports (i.e., during a join maneuver) should be delayed by the time for approaching the target platoon",
    )
    g_platoon.add_argument(
        "--update-desired-speed",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['update_desired_speed'],
        choices=(True, False),
        help="Whether to update the platoon's desired driving speed to the average speed of all members after the formation changed",
    )

    # formation properties
    g_formation = parser.add_argument_group("formation properties")
    g_help.add_argument(
        "--help-formation",
        action='store_true',
        default=argparse.SUPPRESS,
        help="show help message for formation properties and exit",
    )
    g_formation.add_argument(
        "--formation-algorithm",
        type=str,
        default=DEFAULTS['formation_algorithm'],
        choices=globals()['algorithms'],
        help="The formation algorithm to use",
    )
    g_formation.add_argument(
        "--formation-strategy",
        type=str,
        default=DEFAULTS['formation_strategy'],
        choices=["distributed", "centralized"],
        help="The formation strategy to use",
    )
    g_formation.add_argument(
        "--execution-interval",
        type=int,
        default=DEFAULTS['execution_interval'],
        help="The interval between two iterations of a formation algorithm in s",
    )

    # infrastructure properties
    g_infrastructure = parser.add_argument_group("infrastructure properties")
    g_help.add_argument(
        "--help-infrastructure",
        action='store_true',
        default=argparse.SUPPRESS,
        help="show help message for infrastructure properties and exit",
    )
    g_infrastructure.add_argument(
        "--infrastructures",
        type=int,
        dest='number_of_infrastructures',
        default=DEFAULTS['infrastructures'],
        help="The number of infrastructures",
    )

    # simulation properties
    g_simulation = parser.add_argument_group("simulation properties")
    g_help.add_argument(
        "--help-simulation",
        action='store_true',
        default=argparse.SUPPRESS,
        help="show help message for simulation properties and exit",
    )
    g_simulation.add_argument(
        "--step-length",
        type=float,
        default=DEFAULTS['step_length'],
        help="The step length in s",
    )
    g_simulation.add_argument(
        "--time-limit",
        type=float,
        dest='max_step',
        default=float(DEFAULTS['max_step'] / 3600),  # s -> h
        help="The simulation limit in h",
    )
    g_simulation.add_argument(
        "--actions",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['actions'],
        choices=(True, False),
        help="Whether to enable actions of vehicles and infrastructures",
    )
    g_simulation.add_argument(
        "--collisions",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['collisions'],
        choices=(True, False),
        help="Whether to enable checks for collision among vehicles",
    )
    g_simulation.add_argument(
        "--random-seed",
        type=int,
        default=DEFAULTS['random_seed'],
        help="The seed (>=0) for the random number generator. A value of -1 uses the current system time",
    )
    g_simulation.add_argument(
        "--progress",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['progress'],
        choices=(True, False),
        help="Whether to enable the (simulation) progress bar",
    )

    # GUI properties
    g_gui = parser.add_argument_group("GUI properties")
    g_help.add_argument(
        "--help-gui",
        action='store_true',
        default=argparse.SUPPRESS,
        help="show help message for GUI properties and exit",
    )
    g_gui.add_argument(
        "--gui",
        action="store_true",
        help="Enable a live SUMO GUI"
    )
    g_gui.add_argument(
        "--gui-delay",
        type=int,
        default=int(DEFAULTS['gui_delay'] * 1000),  # s -> ms
        help="The delay used in every simulation step to visualize the current network state in ms",
    )
    g_gui.add_argument(
        "--track-vehicle",
        type=int,
        dest='gui_track_vehicle',
        default=DEFAULTS['gui_track_vehicle'],
        help="The id of a vehicle to track in the GUI",
    )
    g_gui.add_argument(
        "--sumo-config",
        type=find_resource,
        default=DEFAULTS['sumo_config'],
        help="The name of the SUMO config file",
    )
    g_gui.add_argument(
        "--gui-play",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['gui_play'],
        choices=(True, False),
        help="Whether to start the simulation immediately",
    )
    g_gui.add_argument(
        "--gui-start",
        type=int,
        default=DEFAULTS['gui_start'],
        help="The time to connect to the GUI in s",
    )
    g_gui.add_argument(
        "--draw-ramps",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['draw_ramps'],
        choices=(True, False),
        help="Whether to draw on-/off-ramps",
    )
    g_gui.add_argument(
        "--draw-ramp-labels",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['draw_ramp_labels'],
        choices=(True, False),
        help="Whether to draw labels for on-/off-ramps",
    )
    g_gui.add_argument(
        "--draw-road-end",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['draw_road_end'],
        choices=(True, False),
        help="Whether to draw the end of the road",
    )
    g_gui.add_argument(
        "--draw-road-end-label",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['draw_road_end_label'],
        choices=(True, False),
        help="Whether to draw a label for the end of the road",
    )
    g_gui.add_argument(
        "--draw-infrastructures",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['draw_infrastructures'],
        choices=(True, False),
        help="Whether to draw infrastructures",
    )
    g_gui.add_argument(
        "--draw-infrastructure-labels",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['draw_infrastructure_labels'],
        choices=(True, False),
        help="Whether to draw labels for infrastructures",
    )
    g_gui.add_argument(
        '--screenshot-file',
        type=str,
        default=None,
        dest='screenshot_filename',
        help="The name of the screenshot file",
    )

    # result recording properties
    g_results = parser.add_argument_group("result recording properties")
    g_help.add_argument(
        "--help-results",
        action='store_true',
        default=argparse.SUPPRESS,
        help="show help message for result recording properties and exit",
    )
    g_results.add_argument(
        "--result-base-filename",
        type=str,
        default=DEFAULTS['result_base_filename'],
        help="The base filename of the result files",
    )
    g_results.add_argument(
        "--record-simulation-trace",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['record_simulation_trace'],
        choices=(True, False),
        help="Whether to record a continuous simulation trace",
    )
    g_results.add_argument(
        "--record-end-trace",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['record_end_trace'],
        choices=(True, False),
        help="Whether to record another trace item at the trip end",
    )
    g_results.add_argument(
        "--record-vehicle-trips",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['record_vehicle_trips'],
        choices=(True, False),
        help="Whether to record vehicle trips",
    )
    g_results.add_argument(
        "--record-vehicle-emissions",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['record_vehicle_emissions'],
        choices=(True, False),
        help="Whether to record vehicle emissions",
    )
    g_results.add_argument(
        "--record-vehicle-traces",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['record_vehicle_traces'],
        choices=(True, False),
        help="Whether to record continuous vehicles traces",
    )
    g_results.add_argument(
        "--record-vehicle-changes",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['record_vehicle_changes'],
        choices=(True, False),
        help="Whether to record vehicle lane changes",
    )
    g_results.add_argument(
        "--record-emission-traces",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['record_emission_traces'],
        choices=(True, False),
        help="Whether to record continuous emission traces",
    )
    g_results.add_argument(
        "--record-platoon-trips",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['record_platoon_trips'],
        choices=(True, False),
        help="Whether to record platoon trips",
    )
    g_results.add_argument(
        "--record-platoon-maneuvers",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['record_platoon_maneuvers'],
        choices=(True, False),
        help="Whether to record platoon maneuvers",
    )
    g_results.add_argument(
        "--record-platoon-formation",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['record_platoon_formation'],
        choices=(True, False),
        help="Whether to record platoon formation results",
    )
    g_results.add_argument(
        "--record-platoon-traces",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['record_platoon_traces'],
        choices=(True, False),
        help="Whether to record continuous platoon traces",
    )
    g_results.add_argument(
        "--record-vehicle-platoon-traces",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['record_vehicle_platoon_traces'],
        choices=(True, False),
        help="Whether to record continuous vehicle platoon traces",
    )
    g_results.add_argument(
        "--record-platoon-changes",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['record_platoon_changes'],
        choices=(True, False),
        help="Whether to record platoon lane changes",
    )
    g_results.add_argument(
        "--record-vehicle-teleports",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['record_vehicle_teleports'],
        choices=(True, False),
        help="Whether to record vehicle teleports",
    )
    g_results.add_argument(
        "--record-prefilled",
        type=lambda x: bool(strtobool(x)),
        default=DEFAULTS['record_prefilled'],
        choices=(True, False),
        help="Whether to record results for pre-filled vehicles",
    )

    # formation algorithm specific properties
    for algorithm in globals()['algorithms']:
        globals()[f'group_{algorithm}'] = globals()[algorithm].add_parser_argument_group(parser)

        g_help.add_argument(
            f"--help-{algorithm}",
            action='store_true',
            default=argparse.SUPPRESS,
            help=f"show help message for {algorithm} formation algorithm and exit",
        )

    # print usage without any arguments
    if len(sys.argv) < 2:
        # no argument has been passed
        print(
            parser.format_usage(),
            parser.description,
            parser.epilog,
            sep='\n',
            end='',
        )
        parser.exit()

    # parse the arguments
    args = parser.parse_args()

    # simple help
    misc_groups = parser._action_groups[:2]
    if 'help' in args and args.help:
        print(format_help(parser, misc_groups + [g_help]), end='')
        parser.exit()
    # complete help
    elif 'help_all' in args and args.help_all:
        print(format_help(parser), end='')
        parser.exit()
    # road network properties
    elif 'help_road' in args and args.help_road:
        print(format_help(parser, misc_groups + [g_road]), end='')
        parser.exit()
    # vehicle properties
    elif 'help_vehicles' in args and args.help_vehicles:
        print(format_help(parser, misc_groups + [g_vehicles]), end='')
        parser.exit()
    # trip properties
    elif 'help_trips' in args and args.help_trips:
        print(format_help(parser, misc_groups + [g_trips]), end='')
        parser.exit()
    # communication properties
    elif 'help_communication' in args and args.help_communication:
        print(format_help(parser, misc_groups + [g_communication]), end='')
        parser.exit()
    # platoon properties
    elif 'help_platoon' in args and args.help_platoon:
        print(format_help(parser, misc_groups + [g_platoon]), end='')
        parser.exit()
    # formation properties
    elif 'help_formation' in args and args.help_formation:
        print(format_help(parser, misc_groups + [g_formation]), end='')
        parser.exit()
    # infrastructure properties
    elif 'help_infrastructure' in args and args.help_infrastructure:
        print(format_help(parser, misc_groups + [g_infrastructure]), end='')
        parser.exit()
    # simulation properties
    elif 'help_simulation' in args and args.help_simulation:
        print(format_help(parser, misc_groups + [g_simulation]), end='')
        parser.exit()
    # GUI properties
    elif 'help_gui' in args and args.help_gui:
        print(format_help(parser, misc_groups + [g_gui]), end='')
        parser.exit()
    # result recording properties
    elif 'help_results' in args and args.help_results:
        print(format_help(parser, misc_groups + [g_results]), end='')
        parser.exit()
    # formation algorithm specific properties
    else:
        for algorithm in globals()['algorithms']:
            if f'help_{algorithm}' in args and getattr(args, f'help_{algorithm}'):
                print(format_help(parser, misc_groups + [globals()[f'group_{algorithm}']]), end='')
                parser.exit()

    # transform argument values into correct units
    args.road_length *= 1000  # km -> m
    args.ramp_interval *= 1000  # km -> m
    args.minimum_trip_length *= 1000  # km -> m
    args.maximum_trip_length *= 1000  # km -> m
    args.max_step *= 3600  # h -> s
    args.gui_delay /= 1000  # ms -> s

    return args, g_gui


def load_snapshot(snapshot_filename: str) -> Simulator:
    """
    Load a simulator object from a snapshot file.

    Parameters
    ----------
    snapshot_filename : str
        The name of the file containing the snapshot

    Returns
    -------
    Simulator : The loaded simulator object
    """

    assert snapshot_filename

    with open(snapshot_filename, "rb") as f:
        # load saved state
        simulator = pickle.load(f)
        assert isinstance(simulator, Simulator)

    return simulator


def save_snapshot(simulator: Simulator, snapshot_filename: str):
    """
    Store a simulator object to a snapshot file.

    Parameters
    ----------
    simulator : Simulator
        The simulator object to store
    snapshot_filename : str
        The name of the file for storing the snapshot
    """

    assert isinstance(simulator, Simulator)
    assert snapshot_filename

    with open(snapshot_filename, "wb") as f:
        pickle.dump(simulator, f)


def create_simulator(**kwargs: dict) -> Simulator:
    """
    Create a simulator object from given keyword arguments.

    Parameters
    ----------
    kwargs : dict
        The dictionary of keyword arguments to use for the creation

    Returns
    -------
    Simulator : The created simulator object
    """

    assert kwargs

    # prepare keyword arguments for simulator
    kwargs.pop('load_snapshot')
    kwargs.pop('save_snapshot')
    kwargs['log_level'] = logging.getLevelName(max(DEFAULTS['log_level'] - ((kwargs['verbosity'] - kwargs['quiet']) * 10), 5))
    kwargs['max_step'] = int(kwargs['max_step'])

    # create new simulator
    return Simulator(**kwargs)


def main():
    """
    The main entry point of PlaFoSim.
    """

    # get argument values (and GUI argument group)
    args, g_gui = parse_args()

    if args.default:
        print("Running with default configuration...")

    if args.dry_run:
        print(f"Current configuration:\n{json.dumps(dict(sorted(vars(args).items())),indent=2)}")
        return

    simulator = None
    if args.load_snapshot:
        # load snapshot
        simulator = load_snapshot(snapshot_filename=args.load_snapshot)

        # allow to override the loaded GUI parameters
        simulator.__dict__.update(
            {
                f"_{k}": v
                for k, v in vars(args).items()
                if k in [x.dest for x in g_gui._group_actions]
            }
        )
        print(f"Loaded a snapshot of the simulation from {args.load_snapshot}. Running simulation with the loaded state...")
    else:
        # create new simulator
        simulator = create_simulator(**vars(args))

    if args.save_snapshot:
        if args.load_snapshot:
            sys.exit(f"ERROR [{__name__}]: Saving a loaded snapshot does not make sense!")
        # save snapshot
        save_snapshot(simulator, snapshot_filename=args.save_snapshot)
        print(f"Saved a snapshot of the simulation to {args.save_snapshot}. Exiting...")
        return

    def handler(signal, frame):
        simulator.stop("SIGINT or CTRL-C detected! Stopping simulation...")
        exit(1)

    signal(SIGINT, handler)

    # execute simulation
    start_time = timer()

    steps = simulator.run()

    end_time = timer()
    run_time = end_time - start_time

    print(f"The simulation took {run_time} seconds ({(steps / run_time)} step/s)")


if __name__ == "__main__":
    sys.exit(main())
