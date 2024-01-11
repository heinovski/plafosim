#
# Copyright (c) 2020-2024 Julian Heinovski <heinovski@ccs-labs.org>
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
import logging
import sys
import time

import numpy as np
import pandas as pd
from tqdm import tqdm

from plafosim import CustomFormatter, __citation__, __description__, __version__
from plafosim.gui import (
    add_gui_vehicle,
    change_gui_vehicle_color,
    check_and_prepare_gui,
    close_gui,
    gui_step,
    move_gui_vehicle,
    prune_vehicles,
    start_gui,
)
from plafosim.simulator import DEFAULTS
from plafosim.util import find_resource, hex2rgb

LOG = logging.getLogger(__name__)


# TODO duplicated code with main script
def parse_args() -> argparse.Namespace:
    """
    Parse arguments given to this module.

    Returns
    -------
    argparse.Namespace
        The namespace of parsed arguments and corresponding values.
    """

    # parse some parameters
    parser = argparse.ArgumentParser(
        formatter_class=CustomFormatter,
        allow_abbrev=False,
        description=__description__,
    )

    # miscellaneous
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
        "-q", "--quiet",
        action="count",
        default=0,
        help=f"The amount of verbosity levels to be removed for printing the logs to the CLI. The starting level is {logging.getLevelName(DEFAULTS['log_level'])}.",
    )
    parser.add_argument(
        "-v", "--verbosity",
        action="count",
        default=0,
        help=f"The amount of verbosity levels to be added for printing the logs to the CLI. The starting level is {logging.getLevelName(DEFAULTS['log_level'])}.",
    )

    # functionality
    parser.add_argument(
        'trace_file',
        type=str,
        help="The name of the vehicle trace file"
    )
    parser.add_argument(
        "--sumo-config",
        type=find_resource,
        default=DEFAULTS['sumo_config'],
        help="The name of the SUMO configuration file",
    )
    parser.add_argument(
        '--gui-delay',
        type=int,
        default=0,
        help="The delay used in every simulation step to visualize the current network state in ms",
    )
    parser.add_argument(
        '--track-vehicle',
        type=int,
        default=-1,
        help="The id of a vehicle to track in the GUI",
    )
    parser.add_argument(
        '--start',
        type=float,
        default=0,
        help="The first step to re-play from the trace file",
    )
    parser.add_argument(
        '--end',
        type=float,
        default=-1,
        help="The last step to re-play from the trace file. -1 is no limit",
    )
    parser.add_argument(
        '--screenshot-file',
        type=str,
        default=None,
        help="The name of the screenshot file",
    )

    # print usage without any arguments
    if len(sys.argv) < 2:
        # no argument has been passed
        print(
            parser.format_usage(),
            parser.description,
            sep='\n',
            end='',
        )
        parser.exit()

    return parser.parse_args()


def main():
    """
    The main entry point of PlaFoSim's trace replay.
    """

    args = parse_args()

    log_level = logging.getLevelName(max(DEFAULTS['log_level'] - ((args.verbosity - args.quiet) * 10), 5))
    # TODO add custom filter that prepends the log entry with the step time
    logging.basicConfig(level=log_level.upper(), format="%(levelname)s [%(name)s]: %(message)s")

    check_and_prepare_gui()

    LOG.info("Replaying vehicle trace...")

    traces = pd.read_csv(args.trace_file)

    step_length = np.diff(traces.step.unique()[:2])[0]

    start_gui(config=args.sumo_config, step_length=step_length)

    if args.track_vehicle >= 0:
        traces = traces.set_index('step')
        # limit trace to steps where tracked vehicle is present
        ego_vehicle_traces = traces[traces.id == args.track_vehicle]
        traces = traces.loc[ego_vehicle_traces.index]
        # limit trace to vehicles which are actually visible (200m radius around tracked vehicle)
        traces = traces[abs(traces.position - ego_vehicle_traces.position) <= 200]
        traces = traces.reset_index('step')
    assert not traces.empty

    min_step = max(traces.step.min(), args.start)
    max_step = min(traces.step.max(), args.end) if args.end != -1 else traces.step.max()
    LOG.debug(f"Running from {min_step}s to {max_step}s with step length {step_length}s...")

    if min_step > 0:
        gui_step(min_step)

    import traci
    for step in tqdm(np.arange(min_step, max_step, step_length), desc="Trace progress", unit='step'):
        # simulate vehicles from trace file
        for vehicle in traces[traces.step == step].itertuples():
            color = hex2rgb(vehicle.color) if 'color' in traces else (0, 255, 0)  # allow traces without color
            if str(vehicle.id) not in traci.vehicle.getIDList():
                add_gui_vehicle(
                    vehicle.id,
                    vehicle.position,
                    vehicle.lane,
                    vehicle.speed,
                    color=color,
                    track=vehicle.id == args.track_vehicle,
                )
            move_gui_vehicle(vehicle.id, vehicle.position, vehicle.lane, vehicle.speed)
            change_gui_vehicle_color(vehicle.id, color)

        # remove vehicles not in trace file (keep vehicles in trace file)
        prune_vehicles(keep_vids=list(traces.loc[traces.step == step]['id']))

        # sleep for visualization
        time.sleep(args.gui_delay / 1000)

        gui_step(step + step_length, screenshot_filename=args.screenshot_file)

    # end of file
    LOG.info("Reached end of trace file")

    # remove all vehicles
    prune_vehicles(keep_vids=[])

    close_gui()


if __name__ == "__main__":
    sys.exit(main())
