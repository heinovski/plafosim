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
import matplotlib.pyplot as pl
import pandas
import re
import seaborn
import sys

from math import ceil


# Read parameters

class CustomFormatter(argparse.ArgumentDefaultsHelpFormatter,
                      argparse.RawDescriptionHelpFormatter,
                      argparse.MetavarTypeHelpFormatter):
    """Metaclass combining multiple formatter classes for argparse"""
    pass


parser = argparse.ArgumentParser(formatter_class=CustomFormatter, description="")
parser.add_argument('--experiment', type=str, default='cc',
                    help="The name of the experiment to use for all result files")
parser.add_argument('--desired-speed', type=float, default=36.0, help="The desired speed to use for the comparison")
parser.add_argument('--arrival-position', type=int, default=100000,
                    help="The arrival position to use for the comparison")
args = parser.parse_args()

# Read runtimes

runtimes = pandas.read_csv('runtimes_%s.csv' % args.experiment)
runtimes = runtimes.astype({'simulator': str})
runtimes = runtimes.set_index('simulator')

# Read trips/emissions

sumo_trips = pandas.read_csv('%s-trips.csv' % args.experiment)
sumo_trips = sumo_trips.rename(columns=lambda x: re.sub('tripinfo_', '', x))
sumo_trips = sumo_trips.rename(columns=lambda x: re.sub('emissions_', '', x))
sumo_trips = sumo_trips.rename(columns=lambda x: re.sub('_abs', '', x))
sumo_trips = sumo_trips.replace(r'static\.', '', regex=True)
sumo_trips = sumo_trips.replace('edge_0_0_', '', regex=True)
sumo_trips = sumo_trips.astype({'arrivalLane': int, 'departLane': int, 'id': int, 'vType': str})
# add desired speed to data frame
sumo_trips = sumo_trips.assign(desiredSpeed=lambda x: x.speedFactor * args.desired_speed)
sumo_trips = sumo_trips.set_index('id').sort_index()

plafosim_trips = pandas.read_csv('%s_vehicle_trips.csv' % args.experiment)
plafosim_trips = plafosim_trips.set_index('id').sort_index()

plafosim_emissions = pandas.read_csv('%s_vehicle_emissions.csv' % args.experiment)
plafosim_emissions = plafosim_emissions.set_index('id').sort_index()

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
sumo_traces.replace('edge_0_0_', '', regex=True, inplace=True)
sumo_traces = sumo_traces.astype({'step': int, 'id': int, 'lane': int})
sumo_traces.sort_values(by='step', inplace=True)

plafosim_traces = pandas.read_csv(
    '%s_vehicle_traces.csv' %
    args.experiment, usecols=[
        'step', 'id', 'position', 'lane', 'speed'])
plafosim_traces.sort_values(by='step', inplace=True)

# Read lane-changes

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
sumo_changes.replace('edge_0_0_', '', regex=True, inplace=True)
sumo_changes = sumo_changes.astype({'step': int, 'id': int, 'from': int, 'to': int})
sumo_changes.sort_values(by='step', inplace=True)

plafosim_changes = pandas.read_csv('%s_vehicle_changes.csv' % args.experiment)
plafosim_changes.sort_values(by='step', inplace=True)

# get the number of (finished) vehicles
ids = frozenset(sumo_trips.index).intersection(plafosim_trips.index)
number_of_vehicles = len(ids)
error = False

# Evalute runtime

pl.figure()
pl.title("Runtime for %d Vehicles" % number_of_vehicles)
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
pl.title("Desired Driving Speed for %d Vehicles" % number_of_vehicles)
pl.boxplot([sumo_trips.desiredSpeed, plafosim_trips.desiredSpeed], showmeans=True, labels=['sumo', 'plasfosim'])
# seaborn.boxplot(x=['sumo', 'plafosim'], y=[sumo_trips.desiredSpeed, plafosim_trips.desiredSpeed], showmeans=True)
pl.xlabel("simulator")
pl.ylabel("speed [m/s]")
pl.savefig('%s_desired_speed.png' % args.experiment)

# ecdfplot with desired driving speed

fig, ax = pl.subplots()
pl.title("Desired Driving Speed for %d Vehicles" % number_of_vehicles)
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
    pl.title("Deviation to Sumo in %s for %d Vehicles" % (label, number_of_vehicles))
    pl.boxplot(data, showmeans=True)
    pl.savefig('%s_diff_%s_box.png' % (args.experiment, label))

    pl.figure()
    pl.title("Deviation to Sumo in %s for %d Vehicles" % (label, number_of_vehicles))
    seaborn.ecdfplot(
        data,
    )
    pl.savefig('%s_diff_%s_dist.png' % (args.experiment, label))

    # check limits for deviation to sumo
    d = data.describe()

    if args.experiment == "acc":
        # limits were set for acc driving with seed 42 on commit 9491b3a748a72c2c9b3f5493dd4145173d5cd831
        if label == 'desiredSpeed':
            l_mean = 0.311689
            l_std = 5.129675
            l_min = -14.137094
            l_25 = -3.869720
            l_50 = 0.360809
            l_75 = 4.116360
            l_max = 11.989368
        elif label == 'arrival':
            l_mean = -125.050000
            l_std = 339.510354
            l_min = -1037.000000
            l_25 = -349.000000
            l_50 = -112.000000
            l_75 = 97.750000
            l_max = 696.000000
        elif label == 'arrivalLane':
            l_mean = -0.05
            l_std = 0.50
            l_min = -2.00
            l_25 = 0.00
            l_50 = 0.00
            l_75 = 0.00
            l_max = 1.00
        elif label == 'arrivalSpeed':
            l_mean = 0.193908
            l_std = 5.151659
            l_min = -14.367094
            l_25 = -3.934720
            l_50 = 0.350551
            l_75 = 3.908860
            l_max = 11.949368
        elif label == 'duration':
            l_mean = -125.050000
            l_std = 339.510354
            l_min = -1037.000000
            l_25 = -349.000000
            l_50 = -112.000000
            l_75 = 97.750000
            l_max = 696.000000
        elif label == 'timeLoss':
            l_mean = -97.581700
            l_std = 114.687015
            l_min = -439.840000
            l_25 = -186.165000
            l_50 = -36.635000
            l_75 = -3.697500
            l_max = 66.120000
    else:
        # limits were set for cc driving with seed 42 on commit 9491b3a748a72c2c9b3f5493dd4145173d5cd831
        assert(args.experiment == "cc")
        if label == 'desiredSpeed':
            l_mean = 0.311689
            l_std = 5.129675
            l_min = -14.137094
            l_25 = -3.869720
            l_50 = 0.360809
            l_75 = 4.116360
            l_max = 11.989368
        elif label == 'arrival':
            l_mean = -26.040000
            l_std = 406.590031
            l_min = -1039.000000
            l_25 = -299.500000
            l_50 = -22.000000
            l_75 = 316.250000
            l_max = 1109.000000
        elif label == 'arrivalLane':
            l_mean = -0.010000
            l_std = 0.481894
            l_min = -1.000000
            l_25 = 0.000000
            l_50 = 0.000000
            l_75 = 0.000000
            l_max = 2.000000
        elif label == 'arrivalSpeed':
            l_mean = 0.199089
            l_std = 5.154142
            l_min = -14.367094
            l_25 = -3.934720
            l_50 = 0.350551
            l_75 = 3.908860
            l_max = 11.949368
        elif label == 'duration':
            l_mean = -26.040000
            l_std = 406.590031
            l_min = -1039.000000
            l_25 = -299.500000
            l_50 = -22.000000
            l_75 = 316.250000
            l_max = 1109.000000
        elif label == 'timeLoss':
            l_mean = 1.406100
            l_std = 7.578794
            l_min = -8.580000
            l_25 = -2.685000
            l_50 = 0.425000
            l_75 = 3.320000
            l_max = 51.310000

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
#    pl.title("Deviation to Sumo in %s for %d Vehicles" % (label, number_of_vehicles))
#    pl.boxplot(data, showmeans=True)
#    pl.savefig('%s_diff_%s_box.png' % (args.experiment, label))
#
#    pl.figure()
#    pl.title("Deviation to Sumo in %s for %d Vehicles" % (label, number_of_vehicles))
#    seaborn.ecdfplot(
#        data,
#    )
#    pl.savefig('%s_diff_%s_dist.png' % (args.experiment, label))

# lifetime plots

lifetime_labels = ['speed', 'position', 'diff_desired', 'lane']

for label in lifetime_labels:

    data = merged_traces[label]

    fig, ax = pl.subplots()
    pl.title("Average %s for %d Vehicles" % (label, number_of_vehicles))
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

        if args.experiment == "acc":
            # limits were set for acc driving with seed 42 on commit 9491b3a748a72c2c9b3f5493dd4145173d5cd831
            l_mean = -0.831217
            l_std = 2.218281
            l_min = -44.640000
            l_25 = -0.610000
            l_50 = 0.000000
            l_75 = 0.000000
            l_max = 0.300000
        else:
            # limits were set for cc driving with seed 42 on commit 9491b3a748a72c2c9b3f5493dd4145173d5cd831
            assert(args.experiment == "cc")
            l_mean = -0.094490
            l_std = 1.789486
            l_min = -44.640000
            l_25 = 0.000000
            l_50 = 0.000000
            l_75 = 0.110000
            l_max = 0.300000
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

    fig.savefig('%s_%s.png' % (args.experiment, label))

lifetime_diff_labels = ['diff_sumo_speed', 'diff_sumo_position', 'diff_sumo_lane']

# diff lifetime plots

for label in lifetime_diff_labels:

    data = merged_traces[label]
    lal = re.sub('diff_sumo_', '', label)

    pl.figure()
    pl.title("Average Deviation to Sumo in %s during Trip for %d Vehicles" % (lal, number_of_vehicles))

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

    if args.experiment == "acc":
        # limits were set for acc driving with seed 42 on commit 9491b3a748a72c2c9b3f5493dd4145173d5cd831
        if label == 'diff_sumo_speed':
            l_mean = 1.500986
            l_std = 4.172065
            l_min = -14.367094
            l_25 = -1.222341
            l_50 = 1.384935
            l_75 = 4.213975
            l_max = 30.532657
        elif label == 'diff_sumo_position':
            l_mean = 1922.106740
            l_std = 6123.681019
            l_min = -20514.177876
            l_25 = -1308.425765
            l_50 = 1137.371442
            l_75 = 5073.995492
            l_max = 28698.190713
        elif label == 'diff_sumo_lane':
            l_mean = 0.605153
            l_std = 0.764974
            l_min = 0.000000
            l_25 = 0.000000
            l_50 = 0.000000
            l_75 = 1.000000
            l_max = 3.000000
    else:
        # limits were set for cc driving with seed 42 on commit 9491b3a748a72c2c9b3f5493dd4145173d5cd831
        assert(args.experiment == "cc")
        if label == 'diff_sumo_speed':
            l_mean = 0.229980
            l_std = 5.030584
            l_min = -16.420497
            l_25 = -3.778178
            l_50 = 0.174948
            l_75 = 3.822624
            l_max = 18.670178
        elif label == 'diff_sumo_position':
            l_mean = 384.018239
            l_std = 7294.801636
            l_min = -32664.608124
            l_25 = -3263.824954
            l_50 = 122.256943
            l_75 = 4197.033319
            l_max = 28742.518143
        elif label == 'diff_sumo_lane':
            l_mean = 0.319393
            l_std = 0.522768
            l_min = 0.000000
            l_25 = 0.000000
            l_50 = 0.000000
            l_75 = 1.000000
            l_max = 3.000000

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
