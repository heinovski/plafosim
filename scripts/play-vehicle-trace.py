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
import os
import sys

if 'SUMO_HOME' not in os.environ:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import argparse


class CustomFormatter(argparse.ArgumentDefaultsHelpFormatter,
                      argparse.RawDescriptionHelpFormatter,
                      argparse.MetavarTypeHelpFormatter):
    """Metaclass combining multiple formatter classes for argparse"""
    pass


parser = argparse.ArgumentParser(formatter_class=CustomFormatter, description="")
parser.add_argument('trace_file', type=str, help="The name of the vehicle trace file")
parser.add_argument('sumo_config', type=str, help="The name of the SUMO config file")
args = parser.parse_args()

tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
sys.path.append(tools)

import traci

sumoBinary = os.path.join(os.environ['SUMO_HOME'], 'bin/sumo-gui')
sumoCmd = [sumoBinary, "-c", args.sumo_config]

import pandas

traces = pandas.read_csv(args.trace_file)

traci.start(sumoCmd)

step = traces.step.min()
traci.simulationStep(step)

from random import randrange

while step <= traces.step.max():
    for vehicle in traces.loc[traces.step == step].itertuples():
        if str(vehicle.id) not in traci.vehicle.getIDList():
            traci.vehicle.add(str(vehicle.id), 'route', departPos=str(vehicle.position), departSpeed=str(vehicle.speed), departLane=str(vehicle.lane), typeID='vehicle')
            traci.vehicle.setColor(str(vehicle.id), (randrange(0, 255, 1), randrange(0, 255, 1), randrange(0, 255, 1)))
            traci.vehicle.setSpeedMode(str(vehicle.id), 0)
            traci.vehicle.setLaneChangeMode(str(vehicle.id), 0)
        traci.vehicle.moveTo(vehID=str(vehicle.id), pos=vehicle.position, laneID='edge_0_0_0')
        # traci.vehicle.moveToXY(vehID=str(vehicle.id), x=vehicle.position, y=traci.vehicle.getPosition3D(str(vehicle.id))[1], lane=vehicle.lane, edgeID='')
    step += 1
    traci.simulationStep(step+1)

traci.close(False)
