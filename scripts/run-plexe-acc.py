#!/usr/bin/env python3
#
# Copyright (c) 2018 Michele Segata <segata@ccs-labs.org>
# Copyright (c) 2020-2021 Julian Heinovski <heinovski@ccs-labs.org>
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
import sys
import random

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

import sumolib  # noqa 402
import traci  # noqa 402
from plexe import Plexe, ACC  # noqa 402

# vehicle length
LENGTH = 4
# cruising speed
SPEED = 36

SEED = 1337
ARRIVAL = 100000

INTERVAL = 3
desired_speeds = {
    0: 37.55812347405618,
    1: 36.63503061403787,
    2: 32.586163253998194,
    3: 30.698558623549044,
    4: 35.33090153878709,
    5: 34.427083177080654,
    6: 41.51127460282562,
    7: 37.0821596040927,
    8: 38.66124582344685,
    9: 40.722578784255035,
    10: 36.41207716998421,
    11: 31.245896385883643,
    12: 38.27969773713686,
    13: 33.38326233097797,
    14: 30.39388160621202,
    15: 34.001076183212405,
    16: 39.83393580290082,
    17: 38.736528314176084,
    18: 36.38080487990444,
    19: 30.505736982908665,
    20: 33.26540568911981,
    21: 35.690663896938226,
    22: 33.93878872953212,
    23: 34.39375456851971,
    24: 38.44534208575163,
    25: 36.58784907505515,
    26: 34.32898690238759,
    27: 37.26716473222217,
    28: 33.17943122510846,
    29: 37.66052981369303,
    30: 37.56299903244568,
    31: 44.29868579309281,
    32: 28.048987388300617,
    33: 35.39429485078965,
    34: 37.79956844633962,
    35: 37.720848977803165,
    36: 42.02753916245895,
    37: 38.98070615408151,
    38: 35.40389093916573,
    39: 38.124697032783935,
    40: 29.872234556157753,
    41: 36.413356025192435,
    42: 36.042526293949074,
    43: 32.848176345375315,
    44: 38.28054643781074,
    45: 42.306772732197736,
    46: 36.08790093529052,
    47: 37.72997579200879,
    48: 43.3119219700726,
    49: 27.596961644281624,
    50: 42.48611760736578,
    51: 41.7679454543631,
    52: 30.887400353431627,
    53: 33.04230098962789,
    54: 42.80732046029295,
    55: 33.36188657664464,
    56: 39.51324769178734,
    57: 32.473641711056175,
    58: 33.635131772625776,
    59: 42.35074969890409,
    60: 36.51003765742793,
    61: 29.947496345129128,
    62: 37.560403033200174,
    63: 36.538517130007534,
    64: 34.02846860335428,
    65: 31.503495948988846,
    66: 44.1220177817637,
    67: 35.23155972365842,
    68: 37.32160611522373,
    69: 40.002627262960765,
    70: 31.67674782086084,
    71: 33.902076259336205,
    72: 41.95251091197894,
    73: 34.83685584649217,
    74: 35.66309719679696,
    75: 40.11995639901386,
    76: 37.3683801353529,
    77: 37.29565797821834,
    78: 35.34683604884972,
    79: 37.250630225667386,
    80: 37.61562748511611,
    81: 40.696586885339606,
    82: 31.570972271180704,
    83: 34.93909554374687,
    84: 39.19143416333266,
    85: 35.88625164508288,
    86: 37.50895940807839,
    87: 34.672924634226575,
    88: 38.46630378480085,
    89: 35.256735372034406,
    90: 43.16318123575888,
    91: 36.55380657259627,
    92: 36.48416747529461,
    93: 39.43900165005688,
    94: 39.25864288173296,
    95: 37.18330353220335,
    96: 29.44550922929806,
    97: 32.0920538819291,
    98: 36.49747517237148,
    99: 36.10973844658279,
}


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
            desired_speed = desired_speeds[i]
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
    parser.add_argument('--sumo-config', type=str, default="sumocfg/freeway-acc.sumo.cfg", help="The name of the SUMO config file")
    args = parser.parse_args()

    main(args.sumo_config, args.vehicles, args.experiment)
