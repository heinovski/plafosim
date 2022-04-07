#!/usr/bin/env python3
#
# Copyright (c) 2018 Michele Segata <segata@ccs-labs.org>
# Copyright (c) 2020-2022 Julian Heinovski <heinovski@ccs-labs.org>
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with this program.  If not, see http://www.gnu.org/licenses/.
#

import argparse
import os
import random
import sys

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import sumolib  # noqa 402
import traci  # noqa 402
from plexe import ACC, Plexe  # noqa 402

# vehicle length
LENGTH = 4
# cruising speed
SPEED = 36

SEED = 1337
ARRIVAL = 5000

INTERVAL = 3


def start_sumo(config_file, gui=False, experiment="acc"):
    arguments = [
        "--device.emissions.deterministic",
        "--device.fcd.deterministic",
        "--emission-output", f"{experiment}-emissions.xml",
        "--fcd-output", f"{experiment}-traces.xml",
        "--lanechange.duration", "0",
        "--lanechange-output", f"{experiment}-changes.xml",
        "--seed", str(SEED),
        "--tripinfo-output", f"{experiment}-trips.xml",
        "-c"]
    sumo_cmd = [sumolib.checkBinary('sumo-gui' if gui else 'sumo')]
    arguments.append(config_file)
    sumo_cmd.extend(arguments)
    traci.start(sumo_cmd)


def add_vehicles(plexe, n):
    # add a platoon of n vehicles
    for i in range(n):
        vid = "v.%d" % i
        traci.vehicle.add(vid, "route", departPos="4", departLane="first", departSpeed=str(0), typeID="acc", arrivalPos=str(ARRIVAL), depart=i * INTERVAL)
        if n == 1:
            desired_speed = SPEED
        else:
            desired_speed = SPEED * random.normalvariate(1.0, 0.1)
            desired_speed = max(desired_speed, 22)
            desired_speed = min(desired_speed, 50)
        plexe.set_cc_desired_speed(vid, desired_speed)
        plexe.set_acc_headway_time(vid, 1.0)
        traci.vehicle.setSpeedMode(vid, 0)
        plexe.set_active_controller(vid, ACC)


def main(sumo_config, n_vehicles, experiment):
    random.seed(SEED)
    gui = False
    start_sumo(sumo_config, gui, experiment)
    plexe = Plexe()
    traci.addStepListener(plexe)
    step = 0

    max_step = 1.1 * 3600 / traci.simulation.getDeltaT()

    # create vehicles and track the joiner
    add_vehicles(plexe, n_vehicles)

    while step <= max_step:

        # check for arrival
        running_vehicles = list(traci.vehicle.getIDList())

        if step > 0 and not running_vehicles:
            print("\nNo more vehicles in the simulation")
            break

        traci.simulationStep()

        step += 1

    # print("closing connection")
    traci.close()


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--experiment', type=str, help="The name of the experiment to use for all result files")
    parser.add_argument('--vehicles', type=int, default=100, help="The number of vehicles to simulate.")
    parser.add_argument('--sumo-config', type=str, default="src/plafosim/sumocfg/freeway-acc.sumo.cfg", help="The name of the SUMO config file")
    args = parser.parse_args()

    main(args.sumo_config, args.vehicles, args.experiment)
