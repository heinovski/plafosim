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
import sys
import textwrap
import time

import pandas
from tqdm import tqdm

from plafosim import __version__
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


class CustomFormatter(
    argparse.ArgumentDefaultsHelpFormatter,
    argparse.RawDescriptionHelpFormatter,
    argparse.MetavarTypeHelpFormatter,
):
    """Metaclass combining multiple formatter classes for argparse"""

    pass


# TODO duplicated code with main script
def parse_args() -> argparse.Namespace:
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
        help="The name of the SUMO config file",
    )
    parser.add_argument(
        '--gui-delay',
        type=int,
        default=0,
        help="The delay used in every simulation step to visualize the current network state in ms"
    )
    parser.add_argument(
        '--track-vehicle',
        type=int,
        default=-1,
        help="The id of a vehicle to track in the gui"
    )
    parser.add_argument(
        '--log-level',
        type=str,
        default=logging.getLevelName(DEFAULTS['log_level']).lower(),
        choices=["error", "warn", "info", "debug", "trace"],
        help="The minimum level of logs to be printed",
    )
    parser.add_argument(
        '--start',
        type=int,
        default=0,
        help="The first step to re-play from the trace file"
    )
    parser.add_argument(
        '--end',
        type=int,
        default=-1,
        help="The last step to re-play from the trace file. -1 is no limit"
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

    return parser.parse_args()


def main():
    args = parse_args()

    # TODO add custom filter that prepends the log entry with the step time
    logging.basicConfig(level=args.log_level.upper(), format="%(levelname)s [%(name)s]: %(message)s")

    check_and_prepare_gui()
    start_gui(config=args.sumo_config)

    LOG.info("Replaying vehicle trace...")

    traces = pandas.read_csv(args.trace_file).astype({'step': int})
    assert not traces.empty

    min_step = max(traces.step.min(), args.start)
    max_step = min(traces.step.max(), args.end) if args.end != -1 else traces.step.max()
    LOG.debug(f"Running from {min_step}s to {max_step}s...")

    if min_step > 0:
        gui_step(min_step)  # TODO consider various step lengths

    import traci
    for step in tqdm(range(min_step, max_step), desc="Trace progress", unit='step'):
        # simulate vehicles from trace file
        for vehicle in traces.loc[traces.step == step].itertuples():
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

        gui_step(step + 1)  # TODO consider various step lengths

    # end of file
    LOG.info("Reached end of trace file")

    # remove all vehicles
    prune_vehicles(keep_vids=[])

    close_gui()


if __name__ == "__main__":
    sys.exit(main())
