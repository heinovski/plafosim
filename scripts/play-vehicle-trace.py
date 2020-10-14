#!/usr/bin/env python3
#
# Copyright (c) 2020 Julian Heinovski <heinovski@ccs-labs.org>
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
import sys
import time

from random import randrange

if 'SUMO_HOME' not in os.environ:
    sys.exit("please declare environment variable 'SUMO_HOME'")


class CustomFormatter(argparse.ArgumentDefaultsHelpFormatter,
                      argparse.RawDescriptionHelpFormatter,
                      argparse.MetavarTypeHelpFormatter):
    """Metaclass combining multiple formatter classes for argparse"""
    pass


parser = argparse.ArgumentParser(formatter_class=CustomFormatter, description="")
parser.add_argument('trace_file', type=str, help="The name of the vehicle trace file")
parser.add_argument('sumo_config', type=str, help="The name of the SUMO config file")
parser.add_argument('--method', type=str, default='pandas', choices=('pandas, read'), help="The method to use for reading the trace file")
parser.add_argument('--gui-delay', type=int, default=0,
                    help="The delay used in every simulation step to visualize the current network state in ms")
parser.add_argument('--track-vehicle', type=int, default=-1, help="The id of a vehicle to track in the gui")
args = parser.parse_args()

tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
sys.path.append(tools)

import traci

sumoBinary = os.path.join(os.environ['SUMO_HOME'], 'bin/sumo-gui')
sumoCmd = [sumoBinary, '-Q', '-c', args.sumo_config, '--collision.action', 'warn']

traci.start(sumoCmd)


def add_vehicle(vid: str, position: str, speed: str, lane: str):
    logging.debug(f"Adding vehicle {vid} at {position},{lane} with {speed}")
    traci.vehicle.add(vid, 'route', departPos=float(position), departSpeed=speed, departLane=lane, typeID='vehicle')
    traci.vehicle.setColor(vid, (randrange(0, 255, 1), randrange(0, 255, 1), randrange(0, 255, 1)))
    traci.vehicle.setSpeedMode(vid, 0)
    traci.vehicle.setLaneChangeMode(vid, 0)
    # track vehicle
    if vid == str(args.track_vehicle):
        traci.gui.trackVehicle("View #0", vid)
        traci.gui.setZoom("View #0", 1000000)


def move_vehicle(vid: str, position: str, speed: str, lane: str):
    logging.debug(f"Moving vehicle {vid} to {position},{lane} with {speed}")
    traci.vehicle.setSpeed(vid, float(speed))
    traci.vehicle.moveTo(vid, pos=float(position), laneID='edge_0_0_%s' % lane)
    # traci.vehicle.moveToXY(vehID=str(vid), x=position, y=traci.vehicle.getPosition3D(str(vid))[1], lane=lane, edgeID='')


def remove_vehicle(vid: str):
    logging.debug(f"Removing vehicle {vid}")
    traci.vehicle.remove(vid, 2)


def use_pandas():
    import pandas
    traces = pandas.read_csv(args.trace_file)

    step = traces.step.min()
    traci.simulationStep(step)

    while step <= traces.step.max():
        logging.info(f"Current step: {step}")  # TODO use tqdm

        # simulate vehicles from trace file
        for vehicle in traces.loc[traces.step == step].itertuples():
            if str(vehicle.id) not in traci.vehicle.getIDList():
                add_vehicle(str(vehicle.id), str(vehicle.position), str(vehicle.speed), str(vehicle.lane))
            move_vehicle(str(vehicle.id), str(vehicle.position), str(vehicle.speed), str(vehicle.lane))

        traci.simulationStep(step)

        # remove vehicles not in trace file
        for vid in traci.vehicle.getIDList():
            if int(vid) not in list(traces.loc[traces.step == step]['id']):
                remove_vehicle(vid)

        # sleep for visualization
        time.sleep(args.gui_delay / 1000)

        step += 1


def use_read():
    step = 0

    with open(args.trace_file, 'r') as file:
        for line in file:
            lstep, vid, position, lane, speed, duration, routeLength = line.strip(' ').split(',')
            if lstep == "step":
                continue
            if int(lstep) < step:
                logging.critical("Step number is not increasing!")
                exit(1)
            elif int(lstep) > step:
                # next step
                traci.simulationStep(step)
                step = int(lstep)
                logging.info(f"Current step: {step}")  # TODO use tqdm

            # simulate vehicles from trace file
            if vid not in traci.vehicle.getIDList():
                add_vehicle(vid, position, speed, lane)
            move_vehicle(vid, position, speed, lane)

            # TODO remove vehicles that arrived

logging.info("Replaying vehicle trace")

if args.method == "pandas":
    use_pandas()
else:
    use_read()

# end of file
logging.info("Reached end of trace file")

# remove all vehicles
for vid in traci.vehicle.getIDList():
    remove_vehicle(vid)

traci.close(False)
