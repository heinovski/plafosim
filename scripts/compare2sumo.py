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
from scipy.stats import ks_2samp

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
parser.add_argument('--significance', type=float, default=0.05,
                    help="The significance level to use for the KS tests")
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

# Read emission traces

sumo_emission_traces = pandas.read_csv(
    f'{args.experiment}-emissions.csv',
    usecols=[
        'timestep_time',
        'vehicle_id',
        'vehicle_CO',
        'vehicle_CO2',
        'vehicle_HC',
        'vehicle_NOx',
        'vehicle_PMx',
        'vehicle_fuel',
    ]
)
sumo_emission_traces = sumo_emission_traces.rename(columns=lambda x: re.sub('timestep_time', 'step', x))
sumo_emission_traces = sumo_emission_traces.rename(columns=lambda x: re.sub('vehicle_', '', x))
sumo_emission_traces.dropna(inplace=True)
sumo_emission_traces.replace(r'static\.', '', regex=True, inplace=True)
sumo_emission_traces.replace(r'v\.', '', regex=True, inplace=True)
# Remove trace values that do not correspond to plafosim (timestep of 1.0s)
# TODO use average of values within 1.0s timestep?
sumo_emission_traces = sumo_emission_traces[sumo_emission_traces.step.mod(1.0) == 0.0]
sumo_emission_traces = sumo_emission_traces.astype({'step': int, 'id': int})
sumo_emission_traces.sort_values(by='step', inplace=True)
assert(len(sumo_emission_traces.id.unique()) == args.vehicles)

plafosim_emission_traces = pandas.read_csv(f'{args.experiment}_emission_traces.csv')
plafosim_emission_traces.sort_values(by='step', inplace=True)
assert(len(plafosim_emission_traces.id.unique()) == args.vehicles)
# assert same vehicles
assert(sorted(list(sumo_emission_traces.id.unique())) == sorted(list(plafosim_emission_traces.id.unique())))

# Evaluate runtime

pl.figure()
pl.title("Runtime for %d Vehicles" % args.vehicles)
data = runtimes.reset_index().melt('simulator', var_name='kind').set_index('simulator')
# does not work because of incompatibility between matplotlib and seaborn
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
# TODO add timeLoss after it is implemented correctly in plafosim
trip_diff_labels = ['desiredSpeed', 'arrival', 'arrivalLane', 'arrivalSpeed', 'duration']
diff_trips = plafosim_trips[trip_diff_labels] - sumo_trips[trip_diff_labels]

# Evaluate emissions

emission_labels = ['CO', 'CO2', 'HC', 'NOx', 'PMx', 'fuel']
diff_emissions = plafosim_emissions[emission_labels] - sumo_trips[emission_labels]

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

# Evaluate emission traces

sumo_emission_traces = sumo_emission_traces.set_index(['id', 'step'], drop=False).sort_index()
plafosim_emission_traces = plafosim_emission_traces.set_index(['id', 'step'], drop=False).sort_index()

plafosim_emission_traces = plafosim_emission_traces.assign(lifetime=lambda x: x.step - x.groupby(level='id').step.min(),
                                                           diff_sumo_CO=lambda x: x.CO - sumo_emission_traces.CO,
                                                           diff_sumo_CO2=lambda x: x.CO2 - sumo_emission_traces.CO2,
                                                           diff_sumo_HC=lambda x: x.HC - sumo_emission_traces.HC,
                                                           diff_sumo_NOx=lambda x: x.NOx - sumo_emission_traces.NOx,
                                                           diff_sumo_PMx=lambda x: x.PMx - sumo_emission_traces.PMx,
                                                           diff_sumo_fuel=lambda x: x.fuel - sumo_emission_traces.fuel
                                                           ).reset_index(drop=True)

sumo_emission_traces = sumo_emission_traces.assign(
    lifetime=lambda x: x.step - x.groupby(level='id').step.min()).reset_index(drop=True)

sumo_emission_traces = sumo_emission_traces.set_index(['id', 'lifetime']).sort_index()
plafosim_emission_traces = plafosim_emission_traces.set_index(['id', 'lifetime']).sort_index()

merged_emission_traces = pandas.concat([sumo_emission_traces, plafosim_emission_traces], keys=[
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

# check distributions of desired speed
print("Running sample test for desired speed...")
assert(ks_2samp(sumo_trips.desiredSpeed, plafosim_trips.desiredSpeed).pvalue >= args.significance)

# ecdfplot with desired driving speed

fig, ax = pl.subplots()
pl.title("Desired Driving Speed for %d Vehicles in m/s" % args.vehicles)
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

    print(f"Plotting diff in {label}...")
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
    print(f"Running sample test for {label}...")
    result = ks_2samp(plafosim_trips[label], sumo_trips[label])
    if result.pvalue < args.significance:
        print("Deviation to Sumo in %s exceeded limits!" % label)
        print(data.describe())
        error = True

# emissions

for label in emission_labels:
    data = diff_emissions[label]

    print(f"Plotting diff in {label}...")
    pl.figure()
    pl.title("Deviation to Sumo in %s for %d Vehicles in ml/mg" % (label, args.vehicles))
    pl.boxplot(data, showmeans=True)
    pl.savefig('%s_diff_%s_box.png' % (args.experiment, label))

    pl.figure()
    pl.title("Deviation to Sumo in %s for %d Vehicles in ml/mg" % (label, args.vehicles))
    seaborn.ecdfplot(
        data,
    )
    pl.savefig('%s_diff_%s_dist.png' % (args.experiment, label))

    # check limits for deviation to sumo
    print(f"Running sample test for {label}...")
    result = ks_2samp(plafosim_emissions[label], sumo_trips[label])
    if result.pvalue < args.significance:
        print("Deviation to Sumo in %s exceeded limits!" % label)
        print(data.describe())
        error = True

# lifetime plots

lifetime_labels = ['speed', 'position', 'diff_desired', 'lane']

for label in lifetime_labels:

    print(f"Plotting life time for {label}...")
    fig, ax = pl.subplots()
    pl.title("Average %s for %d Vehicles in m/s, m, m/s, lid" % (label, args.vehicles))
    seaborn.lineplot(
        data=merged_traces,
        x='lifetime',
        y=label,
        estimator='mean',
        n_boot=10,
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
            ax=ax,
            n_boot=10,
        )
    elif label == 'diff_desired':
        ax.hlines(0, 0, merged_traces.lifetime.max(), color='black', label='desired')
    elif label == 'lane':
        ax.hlines(0, 0, merged_traces.lifetime.max(), color='black', label='desired')
    else:
        sys.exit(f"Unknown label {label}!")

    fig.savefig('%s_%s.png' % (args.experiment, label))

    # check limits for deviation to sumo
    print(f"Running sample test for {label}...")
    result = ks_2samp(plafosim_traces[label], sumo_traces[label])
    if result.pvalue < args.significance:
        print("Deviation to Sumo in %s exceeded limits!" % label)
        print((sumo_traces[label] - plafosim_traces[label]).describe())
        # error = True  # FIXME enable

# emission lifetime

for label in emission_labels:

    print(f"Plotting life time for {label}...")
    fig, ax = pl.subplots()
    pl.title("Average %s for %d Vehicles in mg/ml" % (label, args.vehicles))
    seaborn.lineplot(
        data=merged_emission_traces,
        x='lifetime',
        y=label,
        estimator='mean',
        n_boot=10,
        hue='simulator',
        ax=ax
    )
    pl.xlabel("trip duration [s]")

    fig.savefig('%s_%s.png' % (args.experiment, label))

    # check limits for deviation to sumo
    print(f"Running sample test for {label}...")
    result = ks_2samp(plafosim_emission_traces[label], sumo_emission_traces[label])
    if result.pvalue < args.significance:
        print("Deviation to Sumo in %s exceeded limits!" % label)
        print((sumo_emission_traces[label] - plafosim_emission_traces[label]).describe())
        # error = True  # FIXME enable

lifetime_diff_labels = ['diff_sumo_speed', 'diff_sumo_position', 'diff_sumo_lane']

# diff lifetime plots

for label in lifetime_diff_labels:

    print(f"Plotting life time for {label}...")
    lal = re.sub('diff_sumo_', '', label)

    pl.figure()
    pl.title("Average Deviation to Sumo in %s during trip for %d Vehicles" % (lal, args.vehicles))

    seaborn.lineplot(
        data=merged_traces,
        x='lifetime',
        y=label,
        estimator='mean',
        n_boot=10,
    )
    pl.xlabel("trip duration [s]")

    pl.savefig('%s_diff_%s_line.png' % (args.experiment, lal))

    # NOTE: deviation has already been checked above

# emission lifetime diff plots

lifetime_diff_emission_labels = ['diff_sumo_CO', 'diff_sumo_CO2', 'diff_sumo_HC', 'diff_sumo_NOx', 'diff_sumo_PMx', 'diff_sumo_fuel']

for label in lifetime_diff_emission_labels:

    print(f"Plotting life time for {label}...")
    lal = re.sub('diff_sumo_', '', label)

    pl.figure()
    pl.title("Average Deviation to Sumo in %s during trip for %d Vehicles in mg/ml" % (lal, args.vehicles))

    seaborn.lineplot(
        data=merged_emission_traces,
        x='lifetime',
        y=label,
        estimator='mean',
        n_boot=10,
    )
    pl.xlabel("trip duration [s]")

    pl.savefig('%s_diff_%s_line.png' % (args.experiment, lal))

    # NOTE: deviation has already been checked above

if error:
    sys.exit("There was at least one deviation error!")
