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
import os
import pandas
import random
import sys
import time

from tqdm import tqdm

if 'SUMO_HOME' not in os.environ:
    sys.exit("please declare environment variable 'SUMO_HOME'")

tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
sys.path.append(tools)

import traci  # noqa 402


class CustomFormatter(argparse.ArgumentDefaultsHelpFormatter,
                      argparse.RawDescriptionHelpFormatter,
                      argparse.MetavarTypeHelpFormatter):
    """Metaclass combining multiple formatter classes for argparse"""
    pass


def parse_args() -> argparse.Namespace:
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

    parser.add_argument(
        'trace_file',
        type=str,
        help="The name of the vehicle trace file"
    )
    parser.add_argument(
        '--sumo-config',
        type=str,
        default="sumocfg/freeway.sumo.cfg",
        help="The name of the SUMO config file"
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
        default="warn",
        choices=["warn", "info", "debug"],
        help="Whether to enable debug output"
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

    return parser.parse_args()


def add_vehicle(vid: str, position: str, speed: str, lane: str, track_vid: int):
    logging.debug(f"Adding vehicle {vid} at {position},{lane} with {speed}")
    traci.vehicle.add(vid, 'route', departPos=float(position), departSpeed=speed, departLane=lane, typeID='vehicle')
    traci.vehicle.setColor(vid, (random.randrange(0, 255, 1), random.randrange(0, 255, 1), random.randrange(0, 255, 1)))
    traci.vehicle.setSpeedMode(vid, 0)
    traci.vehicle.setLaneChangeMode(vid, 0)
    # track vehicle
    if vid == str(track_vid):
        traci.gui.trackVehicle("View #0", vid)
        traci.gui.setZoom("View #0", 1000000)


def move_vehicle(vid: str, position: str, speed: str, lane: str):
    logging.debug(f"Moving vehicle {vid} to {position},{lane} with {speed}")
    traci.vehicle.setSpeed(vid, float(speed))
    traci.vehicle.moveTo(vid, pos=float(position), laneID=f'edge_0_0_{lane}')
    # traci.vehicle.moveToXY(vehID=str(vid), x=position, y=traci.vehicle.getPosition3D(str(vid))[1], lane=lane, edgeID='')


def remove_vehicle(vid: str):
    logging.debug(f"Removing vehicle {vid}")
    traci.vehicle.remove(vid, 2)


def main():
    args = parse_args()

    # TODO add custom filter that prepends the log entry with the step time
    logging.basicConfig(level=getattr(logging, args.log_level.upper(), None), format="%(levelname)s: %(message)s")

    sumoBinary = os.path.join(os.environ['SUMO_HOME'], 'bin/sumo-gui')
    sumoCmd = [sumoBinary, '-Q', '-c', args.sumo_config, '--collision.mingap-factor', '0', '--collision.action', 'none']

    traci.start(sumoCmd)

    logging.info("Replaying vehicle trace")

    traces = pandas.read_csv(args.trace_file)
    assert not traces.empty

    min_step = max(traces.step.min(), args.start)
    max_step = min(traces.step.max(), args.end) if args.end != -1 else traces.step.max()
    traci.simulationStep(min_step)

    for step in tqdm(range(min_step, max_step), desc="Trace progress", unit='step'):
        # simulate vehicles from trace file
        for vehicle in traces.loc[traces.step == step].itertuples():
            if str(vehicle.id) not in traci.vehicle.getIDList():
                add_vehicle(str(vehicle.id), str(vehicle.position), str(vehicle.speed), str(vehicle.lane), args.track_vehicle)
            move_vehicle(str(vehicle.id), str(vehicle.position), str(vehicle.speed), str(vehicle.lane))

        traci.simulationStep(step)

        # remove vehicles not in trace file
        for vid in traci.vehicle.getIDList():
            if int(vid) not in list(traces.loc[traces.step == step]['id']):
                remove_vehicle(vid)

        # sleep for visualization
        time.sleep(args.gui_delay / 1000)

    # end of file
    logging.info("Reached end of trace file")

    # remove all vehicles
    for vid in traci.vehicle.getIDList():
        remove_vehicle(vid)

    traci.close(False)


if __name__ == "__main__":
    main()
