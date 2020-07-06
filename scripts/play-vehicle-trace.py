#!/usr/bin/python3
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
import os
import sys

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
args = parser.parse_args()

tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
sys.path.append(tools)

import traci

sumoBinary = os.path.join(os.environ['SUMO_HOME'], 'bin/sumo-gui')
sumoCmd = [sumoBinary, "-c", args.sumo_config]

traci.start(sumoCmd)


def add_vehicle(vid: str, position: str, speed: str, lane: str):
    traci.vehicle.add(vid, 'route', departPos=float(position), departSpeed=speed, departLane=lane, typeID='vehicle')
    traci.vehicle.setColor(vid, (randrange(0, 255, 1), randrange(0, 255, 1), randrange(0, 255, 1)))
    traci.vehicle.setSpeedMode(vid, 0)
    traci.vehicle.setLaneChangeMode(vid, 0)


def move_vehicle(vid: str, position: str, speed: str, lane: str):
    traci.vehicle.setSpeed(vid, float(speed))
    traci.vehicle.moveTo(vid, pos=float(position), laneID='edge_0_0_%s' % lane)
    # traci.vehicle.moveToXY(vehID=str(vid), x=position, y=traci.vehicle.getPosition3D(str(vid))[1], lane=lane, edgeID='')


def use_read():
    step = 0

    with open(args.trace_file, 'r') as file:
        for line in file:
            lstep, vid, position, lane, speed, duration, routeLength = line.strip(' ').split(',')
            if lstep == "step":
                continue
            if int(lstep) < step:
                print("not increasing step number")
                exit(1)
            elif int(lstep) > step:
                # next step
                traci.simulationStep(step)
                step = int(lstep)
                if step % 600 == 0:
                    print("Current step:", step)
            if vid not in traci.vehicle.getIDList():
                add_vehicle(vid, position, speed, lane)
            move_vehicle(vid, position, speed, lane)
        # TODO remove vehicles that arrived


def use_pandas():
    import pandas
    traces = pandas.read_csv(args.trace_file)

    step = traces.step.min()
    traci.simulationStep(step)

    while step <= traces.step.max():
        if step % 600 == 0:
            print("Current step:", step)
        for vehicle in traces.loc[traces.step == step].itertuples():
            if str(vehicle.id) not in traci.vehicle.getIDList():
                add_vehicle(str(vehicle.id), str(vehicle.position), str(vehicle.speed), str(vehicle.lane))
            move_vehicle(str(vehicle.id), str(vehicle.position), str(vehicle.speed), str(vehicle.lane))
        # TODO remove vehicles that arrived
        step += 1
        traci.simulationStep(step)


if args.method == "read":
    use_read()
else:
    use_pandas()

traci.close(False)
