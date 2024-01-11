#!/usr/bin/env python3
#
# Copyright (c) 2018 Michele Segata <segata@ccs-labs.org>
# Copyright (c) 2020-2024 Julian Heinovski <heinovski@ccs-labs.org>
#
# SPDX-License-Identifier: GPL-3.0-or-later
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
from plexe import ACC, CACC, Plexe  # noqa 402

# vehicle length
LENGTH = 5
# inter-vehicle distance
DISTANCE = 5
# cruising speed
SPEED = 36

LEADER = "v.0"

SEED = 1337
ARRIVAL = 5000


def start_sumo(config_file, gui=False, experiment="cacc"):
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
        traci.vehicle.add(vid, "route", departPos=str((n - i) * (DISTANCE + LENGTH) - DISTANCE), departLane="0", departSpeed=str(SPEED), typeID="cacc", arrivalPos=str(ARRIVAL))
        plexe.set_path_cacc_parameters(vid, DISTANCE, 2, 1, 0.5)
        plexe.set_cc_desired_speed(vid, SPEED)
        plexe.set_acc_headway_time(vid, 1.5)
        plexe.set_fixed_lane(vid, 0, safe=False)
        traci.vehicle.setSpeedMode(vid, 0)
        plexe.use_controller_acceleration(vid, False)
        if i == 0:
            plexe.set_active_controller(vid, ACC)
        else:
            plexe.set_active_controller(vid, CACC)
            plexe.enable_auto_feed(vid, True, LEADER, "v.%d" % (i - 1))


def main(sumo_config, n_vehicles, experiment):
    random.seed(SEED)
    gui = False
    start_sumo(sumo_config, gui, experiment)
    plexe = Plexe()
    traci.addStepListener(plexe)
    step = 0

    max_step = 3600 / traci.simulation.getDeltaT()

    # create vehicles and track the joiner
    add_vehicles(plexe, n_vehicles)
    if gui:
        traci.gui.trackVehicle("View #0", LEADER)
        traci.gui.setZoom("View #0", 20000)

    while step <= max_step:

        # check for arrival
        running_vehicles = list(traci.vehicle.getIDList())

        if list(traci.simulation.getArrivedIDList()) and not running_vehicles:
            print("\nNo more vehicles in the simulation")
            break

        arrived_vehicles = []
        for vid in sorted(running_vehicles, key=lambda x: int(x.split('.')[1])):
            if traci.vehicle.getDrivingDistance(vid, "edge_0_0", ARRIVAL) / SPEED <= traci.simulation.getDeltaT():
                # this vehicle will arrive within the next step
                arrived_vehicles.append(vid)

        if arrived_vehicles:
            # print(arrived_vehicles)
            last_arrived_vid = int(arrived_vehicles[-1].split('.')[1])
            new_leader_vid = "v.%d" % (last_arrived_vid + 1)
            # print(f"new leader {new_leader_vid}")
            for vid in sorted(running_vehicles, key=lambda x: int(x.split('.')[1])):
                if vid == new_leader_vid or vid in arrived_vehicles:
                    # disable autofeeding
                    # print(f"disabling autofeeding for {vid}")
                    plexe.enable_auto_feed(vid, False, None, None)
                    # print(f"setting acc for {vid}")
                    plexe.set_active_controller(vid, ACC)
                else:
                    if last_arrived_vid < n_vehicles - 1:
                        # set new leader
                        plexe.enable_auto_feed(vid, True, new_leader_vid, "v.%d" % (int(vid.split('.')[1]) - 1))
                        # print(f"changed leader for {vid}")

        traci.simulationStep()

        step += 1

    # print("closing connection")
    traci.close()


if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--experiment', type=str, help="The name of the experiment to use for all result files")
    parser.add_argument('--vehicles', type=int, default=100, help="The number of vehicles to simulate.")
    parser.add_argument('--sumo-config', type=str, default="src/plafosim/sumocfg/freeway-cacc.sumo.cfg", help="The name of the SUMO config file")
    args = parser.parse_args()

    main(args.sumo_config, args.vehicles, args.experiment)
