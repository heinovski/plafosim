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

# NOTE: limits were set with SUMO 1.6.0 and seed 1337

import argparse
import re
import sys
from math import ceil

import matplotlib.pyplot as pl
import pandas
import seaborn


# Read parameters

class CustomFormatter(argparse.ArgumentDefaultsHelpFormatter,
                      argparse.RawDescriptionHelpFormatter,
                      argparse.MetavarTypeHelpFormatter):
    """Metaclass combining multiple formatter classes for argparse"""
    pass


parser = argparse.ArgumentParser(formatter_class=CustomFormatter, description="")
parser.add_argument('experiment', type=str,
                    help="The name of the experiment to use for all result files")
parser.add_argument('--vehicles', type=int, default=100,
                    help="The number of vehicles to compare.")
parser.add_argument('--desired-speed', type=float, default=36.0, help="The desired speed to use for the comparison")
parser.add_argument('--arrival-position', type=int, default=100000,
                    help="The arrival position to use for the comparison")
args = parser.parse_args()

error = False

# Read runtimes

runtimes = pandas.read_csv('%s_runtimes.csv' % args.experiment)
runtimes = runtimes.astype({'simulator': str})
runtimes = runtimes.set_index('simulator')

# Read trips/emissions

sumo_trips = pandas.read_csv('%s-trips.csv' % args.experiment)
sumo_trips = sumo_trips.rename(columns=lambda x: re.sub('tripinfo_', '', x))
sumo_trips = sumo_trips.rename(columns=lambda x: re.sub('emissions_', '', x))
sumo_trips = sumo_trips.rename(columns=lambda x: re.sub('_abs', '', x))
sumo_trips = sumo_trips.replace(r'static\.', '', regex=True)
sumo_trips = sumo_trips.replace(r'v\.', '', regex=True)
sumo_trips = sumo_trips.replace('edge_0_0_', '', regex=True)
sumo_trips = sumo_trips.astype({'arrivalLane': int, 'departLane': int, 'id': int, 'vType': str})
# add desired speed to data frame
sumo_trips = sumo_trips.assign(desiredSpeed=lambda x: x.speedFactor * args.desired_speed)
sumo_trips = sumo_trips.set_index('id').sort_index()
assert(sumo_trips.index.is_unique)
assert(len(sumo_trips.index == args.vehicles))

plafosim_trips = pandas.read_csv('%s_vehicle_trips.csv' % args.experiment)
plafosim_trips = plafosim_trips.set_index('id').sort_index()
assert(plafosim_trips.index.is_unique)
assert(len(plafosim_trips.index == args.vehicles))
# assert same vehicles
assert(list(sumo_trips.index) == list(plafosim_trips.index))

plafosim_emissions = pandas.read_csv('%s_vehicle_emissions.csv' % args.experiment)
plafosim_emissions = plafosim_emissions.set_index('id').sort_index()
assert(plafosim_emissions.index.is_unique)
assert(len(plafosim_emissions.index == args.vehicles))
# assert same vehicles
assert(list(sumo_trips.index) == list(plafosim_emissions.index))

# Read traces

sumo_traces = pandas.read_csv(
    '%s-traces.csv' %
    args.experiment,
    usecols=[
        'timestep_time',
        'vehicle_id',
        'vehicle_lane',
        'vehicle_pos',
        'vehicle_speed'])
sumo_traces.columns = ['step', 'id', 'lane', 'position', 'speed']
sumo_traces.dropna(inplace=True)
sumo_traces.replace(r'static\.', '', regex=True, inplace=True)
sumo_traces.replace(r'v\.', '', regex=True, inplace=True)
sumo_traces.replace('edge_0_0_', '', regex=True, inplace=True)
# Remove trace values that do not correspond to plafosim (timestep of 1.0s)
# TODO use average of values within 1.0s timestep?
sumo_traces = sumo_traces[sumo_traces.step.mod(1.0) == 0.0]
sumo_traces = sumo_traces.astype({'step': int, 'id': int, 'lane': int})
sumo_traces.sort_values(by='step', inplace=True)
assert(len(sumo_traces.id.unique()) == args.vehicles)

plafosim_traces = pandas.read_csv(
    '%s_vehicle_traces.csv' %
    args.experiment, usecols=[
        'step', 'id', 'position', 'lane', 'speed'])
plafosim_traces.sort_values(by='step', inplace=True)
assert(len(plafosim_traces.id.unique()) == args.vehicles)
# assert same vehicles
assert(sorted(list(sumo_traces.id.unique())) == sorted(list(plafosim_traces.id.unique())))

# Read lane-changes
try:
    sumo_changes = pandas.read_csv(
        '%s-changes.csv' %
        args.experiment,
        usecols=[
            'change_from',
            'change_id',
            'change_pos',
            'change_reason',
            'change_speed',
            'change_time',
            'change_to'])
    sumo_changes.columns = ['from', 'id', 'position', 'reason', 'speed', 'step', 'to']
    sumo_changes.dropna(inplace=True)
    sumo_changes.replace(r'static\.', '', regex=True, inplace=True)
    sumo_changes.replace(r'v\.', '', regex=True, inplace=True)
    sumo_changes.replace('edge_0_0_', '', regex=True, inplace=True)
    sumo_changes = sumo_changes.astype({'step': int, 'id': int, 'from': int, 'to': int})
    sumo_changes.sort_values(by='step', inplace=True)
    assert(len(sumo_changes.id.unique()) <= args.vehicles)
except pandas.errors.EmptyDataError:
    print("No changes detected for SUMO")

try:
    plafosim_changes = pandas.read_csv('%s_vehicle_changes.csv' % args.experiment)
    plafosim_changes.sort_values(by='step', inplace=True)
    assert(len(plafosim_changes.id.unique()) <= args.vehicles)
except pandas.errors.EmptyDataError:
    print("No changes detected for PlaFoSim")
# TODO use changes

# Evalute runtime

pl.figure()
pl.title("Runtime for %d Vehicles" % args.vehicles)
data = runtimes.reset_index().melt('simulator', var_name='kind').set_index('simulator')
# does not work because of imcompatibility between matplotlib and seaborn
# seaborn.scatterplot(data=data, x='kind', y='value', hue='simulator')
pl.scatter(data=data.loc['sumo'], x='kind', y='value', label='sumo')
pl.scatter(data=data.loc['plafosim'], x='kind', y='value', label='plafosim')
pl.ylabel("time [s]")
pl.legend()
pl.savefig('%s_runtime.png' % args.experiment)

# Evaluate trips

# metrics which should not be different among the simulators
trip_equal_labels = ['depart', 'departLane', 'departSpeed', 'arrivalPos']

trip_equality = abs(plafosim_trips[trip_equal_labels] - sumo_trips[trip_equal_labels]) <= 0.01  # for floats
failed_equality = trip_equality.mask(trip_equality).reset_index().melt('id').dropna()[['id', 'variable']]

if not failed_equality.empty:
    print("Some metrics are not equal!")
    failed_equality = failed_equality.set_index(['id', 'variable']).assign(
        sumo=sumo_trips.reset_index().melt('id').set_index(['id', 'variable']),
        plafosim=plafosim_trips.reset_index().melt('id').set_index(['id', 'variable'])
    ).reset_index()
    print(failed_equality.values)

# HACK for small diff in departPos and routeLength
trip_equal_labels = ['departPos', 'routeLength']

trip_equality = abs(plafosim_trips[trip_equal_labels] - sumo_trips[trip_equal_labels]) <= 0.11
failed_equality = trip_equality.mask(trip_equality).reset_index().melt('id').dropna()[['id', 'variable']]

if not failed_equality.empty:
    print("Some metrics are not equal!")
    failed_equality = failed_equality.set_index(['id', 'variable']).assign(
        sumo=sumo_trips.reset_index().melt('id').set_index(['id', 'variable']),
        plafosim=plafosim_trips.reset_index().melt('id').set_index(['id', 'variable'])
    ).reset_index()
    print(failed_equality.values)
# END HACK for small diff in departPos and routeLength

# metrics that can be different among the simulators
trip_diff_labels = ['desiredSpeed', 'arrival', 'arrivalLane', 'arrivalSpeed', 'duration', 'timeLoss']
diff_trips = plafosim_trips[trip_diff_labels] - sumo_trips[trip_diff_labels]

# Evaluate emissions

# TODO emissions
# emission_labels =['co', 'co2', 'hc', 'pmx', 'nox', 'fuel']
# print(sumo_trips.head())
# diff_emissions = plafosim_emissions[emission_labels] - sumo_trips[emission_labels]
# print(diff_emissions)

# Evaluate traces

sumo_traces = sumo_traces.set_index(['id', 'step'], drop=False).sort_index()
plafosim_traces = plafosim_traces.set_index(['id', 'step'], drop=False).sort_index()

plafosim_traces = plafosim_traces.assign(lifetime=lambda x: x.step - x.groupby(level='id').step.min(),
                                         diff_desired=lambda x: x.speed - plafosim_trips.desiredSpeed,
                                         diff_sumo_speed=lambda x: x.speed - sumo_traces.speed,
                                         diff_sumo_position=lambda x: x.position - sumo_traces.position,
                                         diff_sumo_lane=lambda x: abs(x.lane - sumo_traces.lane)
                                         ).reset_index(drop=True)

sumo_traces = sumo_traces.assign(
    lifetime=lambda x: x.step - x.groupby(level='id').step.min(),
    diff_desired=lambda x: x.speed - sumo_trips.speedFactor * args.desired_speed).reset_index(drop=True)

sumo_traces = sumo_traces.set_index(['id', 'lifetime']).sort_index()
plafosim_traces = plafosim_traces.set_index(['id', 'lifetime']).sort_index()

merged_traces = pandas.concat([sumo_traces, plafosim_traces], keys=[
                              'sumo', 'plafosim'], names=['simulator']).reset_index()

# Plotting

print("Plotting trips/emissions/traces...")

# boxplot with desired driving speed

pl.figure()
pl.title("Desired Driving Speed for %d Vehicles" % args.vehicles)
pl.boxplot([sumo_trips.desiredSpeed, plafosim_trips.desiredSpeed], showmeans=True, labels=['sumo', 'plasfosim'])
# seaborn.boxplot(x=['sumo', 'plafosim'], y=[sumo_trips.desiredSpeed, plafosim_trips.desiredSpeed], showmeans=True)
pl.xlabel("simulator")
pl.ylabel("speed [m/s]")
pl.savefig('%s_desired_speed.png' % args.experiment)

# ecdfplot with desired driving speed

fig, ax = pl.subplots()
pl.title("Desired Driving Speed for %d Vehicles" % args.vehicles)
seaborn.ecdfplot(
    sumo_trips.desiredSpeed,
    label='sumo',
    ax=ax
)
seaborn.ecdfplot(
    plafosim_trips.desiredSpeed,
    label='plafosim',
    ax=ax
)
pl.legend(title="simulator")
pl.xlabel("speed [m/s]")
pl.savefig('%s_desired_speed_dist.png' % args.experiment)

# diff labels

for label in trip_diff_labels:

    data = diff_trips[label]

    pl.figure()
    pl.title("Deviation to Sumo in %s for %d Vehicles" % (label, args.vehicles))
    pl.boxplot(data, showmeans=True)
    pl.savefig('%s_diff_%s_box.png' % (args.experiment, label))

    pl.figure()
    pl.title("Deviation to Sumo in %s for %d Vehicles" % (label, args.vehicles))
    seaborn.ecdfplot(
        data,
    )
    pl.savefig('%s_diff_%s_dist.png' % (args.experiment, label))

    # check limits for deviation to sumo
    d = data.describe()

    if args.experiment == "cc":
        if label == 'desiredSpeed':
            l_mean = 0.443730
            l_std = 5.233829
            l_min = -15.603038
            l_25 = -3.039654
            l_50 = 1.074574
            l_75 = 3.385923
            l_max = 13.190750
        elif label == 'arrival':
            l_mean = -35.45000
            l_std = 408.32962
            l_min = -1046.00000
            l_25 = -262.75000
            l_50 = -84.50000
            l_75 = 238.75000
            l_max = 1301.00000
        elif label == 'arrivalLane':
            l_mean = -0.01000
            l_std = 0.38912
            l_min = -1.00000
            l_25 = 0.00000
            l_50 = 0.00000
            l_75 = 0.00000
            l_max = 1.00000
        elif label == 'arrivalSpeed':
            l_mean = 0.440030
            l_std = 5.222097
            l_min = -15.593038
            l_25 = -3.089654
            l_50 = 1.164574
            l_75 = 3.445279
            l_max = 13.070750
        elif label == 'duration':
            l_mean = -35.45000
            l_std = 408.32962
            l_min = -1046.00000
            l_25 = -262.75000
            l_50 = -84.50000
            l_75 = 238.75000
            l_max = 1301.00000
        elif label == 'timeLoss':
            l_mean = 1.249953
            l_std = 5.675675
            l_min = -9.490804
            l_25 = -2.281073
            l_50 = 0.240259
            l_75 = 3.954419
            l_max = 26.179981
        else:
            sys.exit(f"Unknown label {label}!")
    elif args.experiment == "acc":
        if label == 'desiredSpeed':
            l_mean = 0.443730
            l_std = 5.233829
            l_min = -15.603038
            l_25 = -3.039654
            l_50 = 1.074574
            l_75 = 3.385923
            l_max = 13.190750
        elif label == 'arrival':
            l_mean = -109.61000
            l_std = 342.98137
            l_min = -1000.00000
            l_25 = -281.00000
            l_50 = -140.00000
            l_75 = 52.00000
            l_max = 1060.00000
        elif label == 'arrivalLane':
            l_mean = -0.150000
            l_std = 0.575159
            l_min = -2.000000
            l_25 = 0.000000
            l_50 = 0.000000
            l_75 = 0.000000
            l_max = 2.000000
        elif label == 'arrivalSpeed':
            l_mean = 0.450530
            l_std = 5.219383
            l_min = -15.593038
            l_25 = -3.089654
            l_50 = 1.164574
            l_75 = 3.445279
            l_max = 13.070750
        elif label == 'duration':
            l_mean = -109.61000
            l_std = 342.98137
            l_min = -1000.00000
            l_25 = -281.00000
            l_50 = -140.00000
            l_75 = 52.00000
            l_max = 1060.00000
        elif label == 'timeLoss':
            l_mean = -72.870947
            l_std = 121.586038
            l_min = -408.437759
            l_25 = -109.457136
            l_50 = -10.562786
            l_75 = 0.639944
            l_max = 55.501160
        else:
            sys.exit(f"Unknown label {label}!")
    elif args.experiment == "cacc":
        if label == 'desiredSpeed':
            l_mean = 0.0
            l_std = 0.0
            l_min = 0.0
            l_25 = 0.0
            l_50 = 0.0
            l_75 = 0.0
            l_max = 0.0
        elif label == 'arrival':
            # Plexe step size 1s
            # l_mean = -1.0
            # l_std = 0.0
            # l_min = -1.0
            # l_25 = -1.0
            # l_50 = -1.0
            # l_75 = -1.0
            # l_max = -1.0
            # Plexe step size 0.1s
            l_mean = -7.83700
            l_std = 7.57814
            l_min = -25.00000
            l_25 = -13.35000
            l_50 = -4.90000
            l_75 = -1.10000
            l_max = -0.20000
        elif label == 'arrivalLane':
            l_mean = 0.0
            l_std = 0.0
            l_min = 0.0
            l_25 = 0.0
            l_50 = 0.0
            l_75 = 0.0
            l_max = 0.0
        elif label == 'arrivalSpeed':
            # Plexe step size 1s
            # l_mean = 2.459800
            # l_std = 0.580517
            # l_min = 0.000000
            # l_25 = 2.020000
            # l_50 = 2.270000
            # l_75 = 2.545000
            # l_max = 3.370000
            # Plexe step size 0.1s
            l_mean = 14.697700
            l_std = 8.230113
            l_min = 0.000000
            l_25 = 7.065000
            l_50 = 17.480000
            l_75 = 22.235000
            l_max = 23.300000
        elif label == 'duration':
            # Plexe step size 1s
            # l_mean = -1.0
            # l_std = 0.0
            # l_min = -1.0
            # l_25 = -1.0
            # l_50 = -1.0
            # l_75 = -1.0
            # l_max = -1.0
            # Plexe step size 0.1s
            l_mean = -7.83700
            l_std = 7.57814
            l_min = -25.00000
            l_25 = -13.35000
            l_50 = -4.90000
            l_75 = -1.10000
            l_max = -0.20000
        elif label == 'timeLoss':
            # Plexe step size 1s
            # l_mean = -0.623567
            # l_std = 0.256691
            # l_min = -0.976667
            # l_25 = -0.774167
            # l_50 = -0.616667
            # l_75 = -0.451667
            # l_max = -0.286667
            # Plexe step size 0.1s
            l_mean = -7.810167
            l_std = 7.586818
            l_min = -24.976667
            l_25 = -13.324167
            l_50 = -4.881667
            l_75 = -1.089167
            l_max = -0.166667
        else:
            sys.exit(f"Unknown label {label}!")
    elif args.experiment == "cc_single":
        if label == 'desiredSpeed':
            l_mean = 0.0
            l_std = 0.0
            l_min = 0.0
            l_25 = 0.0
            l_50 = 0.0
            l_75 = 0.0
            l_max = 0.0
        elif label == 'arrival':
            l_mean = -1.0
            l_std = 0.0
            l_min = -1.0
            l_25 = -1.0
            l_50 = -1.0
            l_75 = -1.0
            l_max = -1.0
        elif label == 'arrivalLane':
            l_mean = 0.0
            l_std = 0.0
            l_min = 0.0
            l_25 = 0.0
            l_50 = 0.0
            l_75 = 0.0
            l_max = 0.0
        elif label == 'arrivalSpeed':
            l_mean = 0.0
            l_std = 0.0
            l_min = 0.0
            l_25 = 0.0
            l_50 = 0.0
            l_75 = 0.0
            l_max = 0.0
        elif label == 'duration':
            l_mean = -1.0
            l_std = 0.0
            l_min = -1.0
            l_25 = -1.0
            l_50 = -1.0
            l_75 = -1.0
            l_max = -1.0
        elif label == 'timeLoss':
            l_mean = -0.376667
            l_std = 0.0
            l_min = -0.376667
            l_25 = -0.376667
            l_50 = -0.376667
            l_75 = -0.376667
            l_max = -0.376667
        else:
            sys.exit(f"Unknown label {label}!")
    elif args.experiment == "acc_single":
        if label == 'desiredSpeed':
            l_mean = 0.0
            l_std = 0.0
            l_min = 0.0
            l_25 = 0.0
            l_50 = 0.0
            l_75 = 0.0
            l_max = 0.0
        elif label == 'arrival':
            l_mean = -1.0
            l_std = 0.0
            l_min = -1.0
            l_25 = -1.0
            l_50 = -1.0
            l_75 = -1.0
            l_max = -1.0
        elif label == 'arrivalLane':
            l_mean = 0.0
            l_std = 0.0
            l_min = 0.0
            l_25 = 0.0
            l_50 = 0.0
            l_75 = 0.0
            l_max = 0.0
        elif label == 'arrivalSpeed':
            l_mean = 0.0
            l_std = 0.0
            l_min = 0.0
            l_25 = 0.0
            l_50 = 0.0
            l_75 = 0.0
            l_max = 0.0
        elif label == 'duration':
            l_mean = -1.0
            l_std = 0.0
            l_min = -1.0
            l_25 = -1.0
            l_50 = -1.0
            l_75 = -1.0
            l_max = -1.0
        elif label == 'timeLoss':
            l_mean = -0.376667
            l_std = 0.0
            l_min = -0.376667
            l_25 = -0.376667
            l_50 = -0.376667
            l_75 = -0.376667
            l_max = -0.376667
        else:
            sys.exit(f"Unknown label {label}!")
    elif args.experiment == "cacc_single":
        if label == 'desiredSpeed':
            l_mean = 0.0
            l_std = 0.0
            l_min = 0.0
            l_25 = 0.0
            l_50 = 0.0
            l_75 = 0.0
            l_max = 0.0
        elif label == 'arrival':
            l_mean = -0.7
            l_std = 0.0
            l_min = -0.7
            l_25 = -0.7
            l_50 = -0.7
            l_75 = -0.7
            l_max = -0.7
        elif label == 'arrivalLane':
            l_mean = 0.0
            l_std = 0.0
            l_min = 0.0
            l_25 = 0.0
            l_50 = 0.0
            l_75 = 0.0
            l_max = 0.0
        elif label == 'arrivalSpeed':
            l_mean = -0.7
            l_std = 0.0
            l_min = -0.7
            l_25 = -0.7
            l_50 = -0.7
            l_75 = -0.7
            l_max = -0.7
        elif label == 'duration':
            l_mean = -0.7
            l_std = 0.0
            l_min = -0.7
            l_25 = -0.7
            l_50 = -0.7
            l_75 = -0.7
            l_max = -0.7
        elif label == 'timeLoss':
            l_mean = -0.666667
            l_std = 0.0
            l_min = -0.666667
            l_25 = -0.666667
            l_50 = -0.666667
            l_75 = -0.666667
            l_max = -0.666667
        else:
            sys.exit(f"Unknown label {label}!")
    else:
        sys.exit(f"Unknown experiment {args.experiment}!")

    if \
            abs(d['mean']) > ceil(abs(l_mean)) \
            or abs(d['std']) > ceil(abs(l_std)) \
            or abs(d['min']) > ceil(abs(l_min)) \
            or abs(d['25%']) > ceil(abs(l_25)) \
            or abs(d['50%']) > ceil(abs(l_50)) \
            or abs(d['75%']) > ceil(abs(l_75)) \
            or abs(d['max']) > ceil(abs(l_max)):
        error = True
        print("Deviation to Sumo in %s exceeded limits!" % label)
        print(d)

# TODO emissions

# for label in emission_labels:
#    idata = diff_emissions[label]
#
#    pl.figure()
#    pl.title("Deviation to Sumo in %s for %d Vehicles" % (label, args.vehicles))
#    pl.boxplot(data, showmeans=True)
#    pl.savefig('%s_diff_%s_box.png' % (args.experiment, label))
#
#    pl.figure()
#    pl.title("Deviation to Sumo in %s for %d Vehicles" % (label, args.vehicles))
#    seaborn.ecdfplot(
#        data,
#    )
#    pl.savefig('%s_diff_%s_dist.png' % (args.experiment, label))

# lifetime plots

lifetime_labels = ['speed', 'position', 'diff_desired', 'lane']

for label in lifetime_labels:

    data = merged_traces[label]

    fig, ax = pl.subplots()
    pl.title("Average %s for %d Vehicles" % (label, args.vehicles))
    # TODO check ci or disable bootstrapping
    seaborn.lineplot(
        data=merged_traces,
        x='lifetime',
        y=label,
        estimator='mean',
        n_boot=1,
        hue='simulator',
        ax=ax
    )
    pl.xlabel("trip duration [s]")

    if label == 'speed':
        ax.hlines(args.desired_speed, 0, merged_traces.lifetime.max(), color='black', label='desired')
    elif label == 'position':
        seaborn.lineplot(
            x=range(0, merged_traces.lifetime.max()),
            y=[step * args.desired_speed if step <= args.arrival_position / args.desired_speed else None for step in range(0, merged_traces.lifetime.max())],
            color='black',
            ax=ax)
    elif label == 'diff_desired':
        ax.hlines(0, 0, merged_traces.lifetime.max(), color='black', label='desired')

        # check limits for deviation in desired speed
        d = data.describe()

        if args.experiment == "cc":
            l_mean = -0.147831
            l_std = 1.657094
            l_min = -45.360000
            l_25 = -0.020000
            l_50 = 0.000000
            l_75 = 0.000000
            l_max = 0.180000
        elif args.experiment == "acc":
            l_mean = -0.834908
            l_std = 2.318284
            l_min = -45.360000
            l_25 = -0.170000
            l_50 = 0.000000
            l_75 = 0.000000
            l_max = 0.180000
        elif args.experiment == "cacc":
            # Plexe step size 1s
            # l_mean = -0.000103
            # l_std = 0.015260
            # l_min = -2.270000
            # l_25 = 0.000000
            # l_50 = 0.000000
            # l_75 = 0.000000
            # l_max = 0.000000
            # Plexe step size 0.1s
            l_mean = -0.046928
            l_std = 0.921540
            l_min = -23.670000
            l_25 = 0.000000
            l_50 = 0.000000
            l_75 = 0.000000
            l_max = 0.000000
        elif args.experiment == "cc_single":
            l_mean = -0.099641
            l_std = 1.569152
            l_min = -36.000000
            l_25 = 0.000000
            l_50 = 0.000000
            l_75 = 0.000000
            l_max = 0.000000
        elif args.experiment == "acc_single":
            l_mean = -0.099641
            l_std = 1.569152
            l_min = -36.000000
            l_25 = 0.000000
            l_50 = 0.000000
            l_75 = 0.000000
            l_max = 0.000000
        elif args.experiment == "cacc_single":
            l_mean = 0.0
            l_std = 0.0
            l_min = 0.0
            l_25 = 0.0
            l_50 = 0.0
            l_75 = 0.0
            l_max = 0.0
        else:
            sys.exit(f"Unknown experiment {args.experiment}!")

        if \
                abs(d['mean']) > ceil(abs(l_mean)) \
                or abs(d['std']) > ceil(abs(l_std)) \
                or abs(d['min']) > ceil(abs(l_min)) \
                or abs(d['25%']) > ceil(abs(l_25)) \
                or abs(d['50%']) > ceil(abs(l_50)) \
                or abs(d['75%']) > ceil(abs(l_75)) \
                or abs(d['max']) > ceil(abs(l_max)):
            error = True
            print("Deviation to Desired Speed exceeded limits!")
            print(d)
    elif label == 'lane':
        ax.hlines(0, 0, merged_traces.lifetime.max(), color='black', label='desired')
    else:
        sys.exit(f"Unknown label {label}!")

    fig.savefig('%s_%s.png' % (args.experiment, label))

lifetime_diff_labels = ['diff_sumo_speed', 'diff_sumo_position', 'diff_sumo_lane']

# diff lifetime plots

for label in lifetime_diff_labels:

    data = merged_traces[label]
    lal = re.sub('diff_sumo_', '', label)

    pl.figure()
    pl.title("Average Deviation to Sumo in %s during Trip for %d Vehicles" % (lal, args.vehicles))

    seaborn.lineplot(
        data=merged_traces,
        x='lifetime',
        y=label,
        estimator='mean',
        n_boot=1
    )
    pl.xlabel("trip duration [s]")

    pl.savefig('%s_diff_%s_line.png' % (args.experiment, lal))

    # check limits for deviation to sumo
    d = data.describe()

    if args.experiment == "cc":
        if label == 'diff_sumo_speed':
            l_mean = 0.418387
            l_std = 5.032619
            l_min = -16.465348
            l_25 = -2.988566
            l_50 = 1.060630
            l_75 = 3.386836
            l_max = 15.218686
        elif label == 'diff_sumo_position':
            l_mean = 563.898652
            l_std = 7178.357672
            l_min = -35919.143793
            l_25 = -3012.523815
            l_50 = 483.606814
            l_75 = 4206.556529
            l_max = 30583.823185
        elif label == 'diff_sumo_lane':
            l_mean = 0.329999
            l_std = 0.537777
            l_min = 0.000000
            l_25 = 0.000000
            l_50 = 0.000000
            l_75 = 1.000000
            l_max = 3.000000
        else:
            sys.exit(f"Unknown label {label}!")
    elif args.experiment == "acc":
        if label == 'diff_sumo_speed':
            l_mean = 1.364988
            l_std = 4.268770
            l_min = -15.593038
            l_25 = -1.572917
            l_50 = 1.562999
            l_75 = 3.491246
            l_max = 25.890403
        elif label == 'diff_sumo_position':
            l_mean = 1591.163997
            l_std = 5987.753318
            l_min = -29278.666037
            l_25 = -625.900128
            l_50 = 1193.414661
            l_75 = 4312.480553
            l_max = 29252.125562
        elif label == 'diff_sumo_lane':
            l_mean = 0.541795
            l_std = 0.744607
            l_min = 0.000000
            l_25 = 0.000000
            l_50 = 0.000000
            l_75 = 1.000000
            l_max = 3.000000
        else:
            sys.exit(f"Unknown label {label}!")
    elif args.experiment == "cacc":
        if label == 'diff_sumo_speed':
            # Plexe step size 1s
            # l_mean = 0.000205
            # l_std = 0.021581
            # l_min = 0.000000
            # l_25 = 0.000000
            # l_50 = 0.000000
            # l_75 = 0.000000
            # l_max = 2.270000
            # Plexe step size 0.1s
            l_mean = 0.039139
            l_std = 0.731613
            l_min = 0.000000
            l_25 = 0.000000
            l_50 = 0.000000
            l_75 = 0.000000
            l_max = 20.990000
        elif label == 'diff_sumo_position':
            # Plexe step size 1s
            # l_mean = 0.000205
            # l_std = 0.021581
            # l_min = 0.000000
            # l_25 = 0.000000
            # l_50 = 0.000000
            # l_75 = 0.000000
            # l_max = 2.270000
            # Plexe step size 0.1s
            l_mean = 0.241517
            l_std = 5.770070
            l_min = 0.000000
            l_25 = 0.000000
            l_50 = 0.000000
            l_75 = 0.000000
            l_max = 300.390000
        elif label == 'diff_sumo_lane':
            l_mean = 0.0
            l_std = 0.0
            l_min = 0.0
            l_25 = 0.0
            l_50 = 0.0
            l_75 = 0.0
            l_max = 0.0
        else:
            sys.exit(f"Unknown label {label}!")
    elif args.experiment == "cc_single":
        if label == 'diff_sumo_speed':
            l_mean = 0.0
            l_std = 0.0
            l_min = 0.0
            l_25 = 0.0
            l_50 = 0.0
            l_75 = 0.0
            l_max = 0.0
        elif label == 'diff_sumo_position':
            l_mean = 17.850180
            l_std = 0.784646
            l_min = -0.100000
            l_25 = 17.900000
            l_50 = 17.900000
            l_75 = 17.900000
            l_max = 17.900000
        elif label == 'diff_sumo_lane':
            l_mean = 0.0
            l_std = 0.0
            l_min = 0.0
            l_25 = 0.0
            l_50 = 0.0
            l_75 = 0.0
            l_max = 0.0
        else:
            sys.exit(f"Unknown label {label}!")
    elif args.experiment == "acc_single":
        if label == 'diff_sumo_speed':
            l_mean = 0.0
            l_std = 0.0
            l_min = 0.0
            l_25 = 0.0
            l_50 = 0.0
            l_75 = 0.0
            l_max = 0.0
        elif label == 'diff_sumo_position':
            l_mean = 17.850180
            l_std = 0.784646
            l_min = -0.100000
            l_25 = 17.900000
            l_50 = 17.900000
            l_75 = 17.900000
            l_max = 17.900000
        elif label == 'diff_sumo_lane':
            l_mean = 0.0
            l_std = 0.0
            l_min = 0.0
            l_25 = 0.0
            l_50 = 0.0
            l_75 = 0.0
            l_max = 0.0
        else:
            sys.exit(f"Unknown label {label}!")
    elif args.experiment == "cacc_single":
        if label == 'diff_sumo_speed':
            l_mean = 0.0
            l_std = 0.0
            l_min = 0.0
            l_25 = 0.0
            l_50 = 0.0
            l_75 = 0.0
            l_max = 0.0
        elif label == 'diff_sumo_position':
            l_mean = 0.0
            l_std = 0.0
            l_min = 0.0
            l_25 = 0.0
            l_50 = 0.0
            l_75 = 0.0
            l_max = 0.0
        elif label == 'diff_sumo_lane':
            l_mean = 0.0
            l_std = 0.0
            l_min = 0.0
            l_25 = 0.0
            l_50 = 0.0
            l_75 = 0.0
            l_max = 0.0
        else:
            sys.exit(f"Unknown label {label}!")
    else:
        sys.exit(f"Unknown experiment {args.experiment}!")

    if \
            abs(d['mean']) > ceil(abs(l_mean)) \
            or abs(d['std']) > ceil(abs(l_std)) \
            or abs(d['min']) > ceil(abs(l_min)) \
            or abs(d['25%']) > ceil(abs(l_25)) \
            or abs(d['50%']) > ceil(abs(l_50)) \
            or abs(d['75%']) > ceil(abs(l_75)) \
            or abs(d['max']) > ceil(abs(l_max)):
        error = True
        print("Deviation to Sumo in %s exceeded limits!" % lal)
        print(d)

if error:
    sys.exit("There was at least one deviation error!")
