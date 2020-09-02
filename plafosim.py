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
from distutils.util import strtobool
from timeit import default_timer as timer

from src.plafosim.simulator import Simulator


class CustomFormatter(argparse.ArgumentDefaultsHelpFormatter,
                      argparse.RawDescriptionHelpFormatter,
                      argparse.MetavarTypeHelpFormatter):
    """Metaclass combining multiple formatter classes for argparse"""
    pass


def main():

    start_time = timer()

    # parse some parameters
    parser = argparse.ArgumentParser(formatter_class=CustomFormatter, description="""
    Platoon Formation Simulator (PlaFoSim) -- A simple simulator for platoon formation.

    Copyright (c) 2020 Julian Heinovski <heinovski@ccs-labs.org>
    This program comes with ABSOLUTELY NO WARRANTY.
    This is free software, and you are welcome to redistribute it under certain conditions.
    """)

    # road network properties
    road = parser.add_argument_group('road network properties')
    road.add_argument('--road-length', type=int, default=100, help="The length of the road in km")
    road.add_argument('--lanes', type=int, default=4, help="The number of lanes")
    road.add_argument('--depart-interval', type=int, default=1000,
                      help="The distance between departure positions (on-ramps) in m")
    road.add_argument('--arrival-interval', type=int, default=1000,
                      help="The distance between arrival positions (off-ramps) in m")
    # vehicle properties
    vehicle = parser.add_argument_group('vehicle properties')
    vehicle.add_argument('--vehicles', type=int, default=100, help="The number of vehicles")
    vehicle.add_argument('--max-speed', type=float, default=55, help="The maximum possible driving speed in m/s")
    vehicle.add_argument('--acc-headway-time', type=float, default=1.0,
                         help="The headway time to be used for the ACC in s")
    vehicle.add_argument('--collisions', type=lambda x: bool(strtobool(x)), default=True,
                         choices=(True, False), help="Whether to enable collision checks")
    vehicle.add_argument('--lane-changes', type=lambda x: bool(strtobool(x)), default=True,
                         choices=(True, False), help="Whether to enable lane changes")
    vehicle.add_argument('--penetration', type=float, default=1.0,
                         help="Penetration rate of vehicles with platooning capabilities")
    # trip properties
    trip = parser.add_argument_group('trip properties')
    trip.add_argument(
        '--random-depart-position',
        type=lambda x: bool(strtobool(x)),
        default=False,
        choices=(True, False),
        help="Whether to use a random depart position for every vehicle instead of 0 m")
    trip.add_argument(
        '--random-depart-lane',
        type=lambda x: bool(strtobool(x)),
        default=False,
        choices=(True, False),
        help="Whether to use a random depart lane for every vehicle instead of lane 0")
    trip.add_argument('--desired-speed', type=float, default=36.0, help="The desired driving speed im m/s")
    trip.add_argument(
        '--random-desired-speed',
        type=lambda x: bool(strtobool(x)),
        default=True,
        choices=(True, False),
        help="Whether to pick a random (normally distributed) desired driving speed")
    trip.add_argument('--speed-variation', type=float, default=0.1,
                      help="The devation from the desired driving speed in ratio")
    trip.add_argument('--min-desired-speed', type=float, default=22.0, help="The minimum desired driving speed im m/s")
    trip.add_argument('--max-desired-speed', type=float, default=50.0, help="The maximum desired driving speed im m/s")
    trip.add_argument(
        '--random-depart-speed',
        type=lambda x: bool(strtobool(x)),
        default=False,
        choices=(True, False),
        help="Whether to use a random depart speed for every vehicle instead of 0 m/s")
    trip.add_argument(
        '--depart-desired',
        type=lambda x: bool(strtobool(x)),
        default=False,
        choices=(True, False),
        help="Whether the vehicle should depart with its desired speed. Overrides random-depart-speed")
    trip.add_argument('--depart-method', type=str, choices=('interval', 'pobability'),
                      default='interval', help="The departure method of vehicles")
    trip.add_argument('--depart-time-interval', type=int, default=3,
                      help="The interval between two vehicle departures in s")
    trip.add_argument(
        '--random-arrival-position',
        type=lambda x: bool(strtobool(x)),
        default=False,
        choices=(True, False),
        help="Whether to use a random arrival position for every vehicle instead of the end of the road")
    # simulation properties
    simulation = parser.add_argument_group('simulation properties')
    simulation.add_argument('--step-length', type=int, default=1, help="The step length in s")
    simulation.add_argument('--time-limit', type=int, default=100, help="The simulation limit in h")
    simulation.add_argument('--random-seed', type=int, default=-1,
                            help="The seed (>=0) for the random number generator instead of the current system time")
    simulation.add_argument('--debug', type=lambda x: bool(strtobool(x)), default=False,
                            choices=(True, False), help="Whether to enable debug output")
    simulation.add_argument('--gui', type=lambda x: bool(strtobool(x)), default=False,
                            choices=(True, False), help="Whether to enable a live sumo gui")
    simulation.add_argument(
        '--gui-delay',
        type=int,
        default=0,
        help="The delay used in every simulation step to visualize the current network state in ms")
    simulation.add_argument(
        '--result-base-filename',
        type=str,
        default='results',
        help="The base filename of the result files")

    args = parser.parse_args()
    simulator = Simulator(
        args.road_length * 1000,
        args.lanes,
        args.lane_changes,
        args.collisions,
        args.step_length,
        args.random_seed,
        args.debug,
        args.gui,
        args.gui_delay / 1000,
        args.result_base_filename)
    max_step = args.time_limit * 60 * 60
    simulator.generate_vehicles(
        max_step,
        args.vehicles,
        args.penetration,
        args.depart_interval,
        args.arrival_interval,
        args.max_speed,
        args.acc_headway_time,
        args.random_depart_position,
        args.random_depart_lane,
        args.desired_speed,
        args.random_desired_speed,
        args.speed_variation,
        args.min_desired_speed,
        args.max_desired_speed,
        args.random_depart_speed,
        args.depart_desired,
        args.depart_method,
        args.depart_time_interval,
        args.random_arrival_position)
    # TODO log generation parameters
    simulator.run(max_step)

    end_time = timer()
    print("The simulation took %.4f seconds" % (end_time - start_time))


if __name__ == "__main__":
    main()
