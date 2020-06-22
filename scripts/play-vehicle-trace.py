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

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
    sumoBinary = os.path.join(os.environ['SUMO_HOME'], 'bin/sumo-gui')
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import traci
# TODO load via command line
sumoCmd = [sumoBinary, "-c", "cfg/freeway.sumo.cfg"]

import pandas
# TODO load via command line
traces = pandas.read_csv("results_vehicle_traces.csv")

traci.start(sumoCmd)

step = traces.step.min()
traci.simulationStep(step)

while step <= traces.step.max():
    for vehicle in traces.loc[traces.step == step].itertuples():
        if str(vehicle.id) not in traci.vehicle.getIDList():
            traci.vehicle.add(str(vehicle.id), 'route', departPos=str(vehicle.position), departSpeed=str(vehicle.speed), departLane=str(vehicle.lane), typeID='vehicle')
        traci.vehicle.moveTo(vehID=str(vehicle.id), pos=vehicle.position, laneID='edge_0_0_0')
        # traci.vehicle.moveToXY(vehID=str(vehicle.id), x=vehicle.position, y=traci.vehicle.getPosition3D(str(vehicle.id))[1], lane=vehicle.lane, edgeID='')
    step += 1
    traci.simulationStep(step+1)

traci.close(False)
