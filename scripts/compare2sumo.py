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
trip_diff_labels = ['desiredSpeed', 'arrival', 'arrivalLane', 'arrivalSpeed', 'duration', 'timeLoss']
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
    d = data.describe()

    # absolute values might be misleading here, since the absolute values are in mg and rather high
    if args.experiment == "cc":
        if label == 'CO':
            l_mean = 33854.537683
            l_std = 237918.773787
            l_min = -676343.201605
            l_25 = -134638.362404
            l_50 = 65922.721464
            l_75 = 169003.543947
            l_max = 604097.177810
        elif label == 'CO2':
            l_mean = 9.778211e+05
            l_std = 3.310390e+06  # this value is slighly higher than d shows it
            l_min = -8.596299e+06
            l_25 = -1.265574e+06
            l_50 = 1.241510e+06
            l_75 = 2.813392e+06
            l_max = 8.712486e+06
        elif label == 'HC':
            l_mean = 180.352981
            l_std = 1183.431479
            l_min = -3348.217487
            l_25 = -663.204914
            l_50 = 346.842371
            l_75 = 845.805285
            l_max = 3012.187705
        elif label == 'NOx':
            l_mean = 439.127595
            l_std = 1635.263606
            l_min = -4308.844887
            l_25 = -688.651153
            l_50 = 595.699680
            l_75 = 1346.192402
            l_max = 4270.181037
        elif label == 'PMx':
            l_mean = 32.321003
            l_std = 170.677006
            l_min = -478.768746
            l_25 = -90.452106
            l_50 = 58.668938
            l_75 = 131.573556
            l_max = 437.184270
        elif label == 'fuel':
            l_mean = 420.322618
            l_std = 1423.028802
            l_min = -3695.284690
            l_25 = -544.044015
            l_50 = 533.683340
            l_75 = 1209.367469
            l_max = 3745.215032
        else:
            sys.exit(f"Unknown label {label}!")
    elif args.experiment == "acc":
        # we might need to work on ACC, since there is some deviation
        if label == 'CO':
            l_mean = 151706.768323
            l_std = 193003.005601
            l_min = -420492.744921
            l_25 = 47622.559299
            l_50 = 142701.697674
            l_75 = 239915.125962
            l_max = 636711.291780
        elif label == 'CO2':
            l_mean = 4.905430e+06
            l_std = 3.895389e+06  # this value is slightly higher than d shows it
            l_min = -2.862072e+06
            l_25 = 2.371023e+06
            l_50 = 4.405795e+06  # this value is slightly higher than d shows it
            l_75 = 6.738404e+06
            l_max = 1.707963e+07
        elif label == 'HC':
            l_mean = 821.766184
            l_std = 975.968855
            l_min = -2020.076752
            l_25 = 285.810517
            l_50 = 805.506584
            l_75 = 1289.274893
            l_max = 3240.705876
        elif label == 'NOx':
            l_mean = 2077.075411
            l_std = 1836.185345
            l_min = -2003.304649
            l_25 = 889.421482
            l_50 = 1880.705944
            l_75 = 2884.777260
            l_max = 7414.330248
        elif label == 'PMx':
            l_mean = 152.142005
            l_std = 152.859800
            l_min = -264.507987
            l_25 = 64.118213
            l_50 = 145.643393
            l_75 = 223.947810
            l_max = 550.362894
        elif label == 'fuel':
            l_mean = 2108.636373
            l_std = 1674.465220
            l_min = -1230.334672
            l_25 = 1019.205655
            l_50 = 1893.857285
            l_75 = 2896.600530
            l_max = 7341.810317
        else:
            sys.exit(f"Unknown label {label}!")
    elif args.experiment == "cacc":
        # we might need to work on CACC, since there is some deviation
        # the absolute values here might not be useful
        # from the SUMO wiki: When running with sub-second resolution, the emissions written during every simulation step are extrapolated to those that would be generated in 1 second. Thus, care must be taken when computing sums over time.
        # https://sumo.dlr.de/docs/Simulation/Output/EmissionOutput.html
        if label == 'CO':
            # Plexe step size 1s
            # l_mean = 1.804747e+02
            # l_std = 1.830138e+01
            # l_min = 2.107699e-07
            # l_25 = 1.795149e+02
            # l_50 = 1.832378e+02
            # l_75 = 1.832378e+02
            # l_max = 1.832378e+02
            # Plexe step size 0.1s
            l_mean = 1.650070e+04
            l_std = 7.785547e+03
            l_min = 2.107699e-07
            l_25 = 1.027506e+04
            l_50 = 1.729065e+04
            l_75 = 2.331770e+04
            l_max = 2.813597e+04
        elif label == 'CO2':
            # this seems to be broken somehow
            # Plexe step size 1s
            # l_mean = 7.716914e+03
            # l_std = 9.668326e+02
            # l_min = -7.823110e-08
            # l_25 = 6.810784e+03
            # l_50 = 8.127322e+03
            # l_75 = 8.127322e+03
            # l_max = 8.127322e+03
            # Plexe step size 0.1s
            l_mean = 7.296714e+05
            l_std = 3.428502e+05
            l_min = 8.922070e-06
            l_25 = 4.557396e+05
            l_50 = 7.658132e+05
            l_75 = 1.031959e+06
            l_max = 1.234215e+06  # this value is slightly higher than d shows it
        elif label == 'HC':
            # Plexe step size 1s
            # l_mean = 1.029039e+00
            # l_std = 1.058782e-01
            # l_min = -4.443846e-07
            # l_25 = 1.004768e+00
            # l_50 = 1.051144e+00
            # l_75 = 1.051145e+00
            # l_max = 1.051145e+00
            # Plexe step size 0.1s
            l_mean = 9.461114e+01
            l_std = 4.461127e+01
            l_min = -4.443846e-07
            l_25 = 5.894292e+01
            l_50 = 9.916271e+01
            l_75 = 1.337194e+02
            l_max = 1.611159e+02
        elif label == 'NOx':
            # Plexe step size 1s
            # l_mean = 2.741020e+00
            # l_std = 3.994282e-01
            # l_min = -4.440753e-07
            # l_25 = 2.273388e+00
            # l_50 = 2.936044e+00
            # l_75 = 2.936045e+00
            # l_max = 2.936045e+00
            # Plexe step size 0.1s
            l_mean = 2.636213e+02
            l_std = 1.238922e+02
            l_min = -4.440753e-07
            l_25 = 1.646387e+02
            l_50 = 2.766526e+02
            l_75 = 3.727860e+02
            l_max = 4.461804e+02
        elif label == 'PMx':
            # Plexe step size 1s
            # l_mean = 2.100178e-01
            # l_std = 2.339486e-02
            # l_min = -2.221988e-07
            # l_25 = 1.951699e-01
            # l_50 = 2.178721e-01
            # l_75 = 2.178723e-01
            # l_max = 2.178727e-01
            # Plexe step size 0.1s
            l_mean = 1.960612e+01
            l_std = 9.244292e+00
            l_min = -2.221988e-07
            l_25 = 1.221719e+01
            l_50 = 2.054577e+01
            l_75 = 2.770946e+01
            l_max = 3.338916e+01
        elif label == 'fuel':
            # Plexe step size 1s
            # l_mean = 3.317204e+00
            # l_std = 4.155958e-01
            # l_min = 1.182689e-07
            # l_25 = 2.927718e+00
            # l_50 = 3.493615e+00
            # l_75 = 3.493615e+00
            # l_max = 3.493615e+00
            # Plexe step size 0.1s
            l_mean = 3.136569e+02
            l_std = 1.473778e+02
            l_min = 1.182689e-07
            l_25 = 1.959045e+02
            l_50 = 3.291928e+02
            l_75 = 4.435984e+02
            l_max = 5.305399e+02
        else:
            sys.exit(f"Unknown label {label}!")
    elif args.experiment == "cc_single":
        if label == 'CO':
            l_mean = -18.46
            l_std = 0.0
            l_min = -18.46
            l_25 = -18.46
            l_50 = -18.46
            l_75 = -18.46
            l_max = -18.46
        elif label == 'CO2':
            l_mean = -5502.6
            l_std = 0.0
            l_min = -5502.6
            l_25 = -5502.6
            l_50 = -5502.6
            l_75 = -5502.6
            l_max = -5502.6
        elif label == 'HC':
            l_mean = -0.2392
            l_std = 0.0
            l_min = -0.2392
            l_25 = -0.2392
            l_50 = -0.2392
            l_75 = -0.2392
            l_max = -0.2392
        elif label == 'NOx':
            l_mean = -1.7316
            l_std = 0.0
            l_min = -1.7316
            l_25 = -1.7316
            l_50 = -1.7316
            l_75 = -1.7316
            l_max = -1.7316
        elif label == 'PMx':
            l_mean = -0.1519
            l_std = 0.0
            l_min = -0.1519
            l_25 = -0.1519
            l_50 = -0.1519
            l_75 = -0.1519
            l_max = -0.1519
        elif label == 'fuel':
            l_mean = -2.365283
            l_std = 0.0
            l_min = -2.365283
            l_25 = -2.365283
            l_50 = -2.365283
            l_75 = -2.365283
            l_max = -2.365283
        else:
            sys.exit(f"Unknown label {label}!")
    elif args.experiment == "acc_single":
        if label == 'CO':
            l_mean = -18.46
            l_std = 0.0
            l_min = -18.46
            l_25 = -18.46
            l_50 = -18.46
            l_75 = -18.46
            l_max = -18.46
        elif label == 'CO2':
            l_mean = -5502.6
            l_std = 0.0
            l_min = -5502.6
            l_25 = -5502.6
            l_50 = -5502.6
            l_75 = -5502.6
            l_max = -5502.6
        elif label == 'HC':
            l_mean = -0.2392
            l_std = 0.0
            l_min = -0.2392
            l_25 = -0.2392
            l_50 = -0.2392
            l_75 = -0.2392
            l_max = -0.2392
        elif label == 'NOx':
            l_mean = -1.7316
            l_std = 0.0
            l_min = -1.7316
            l_25 = -1.7316
            l_50 = -1.7316
            l_75 = -1.7316
            l_max = -1.7316
        elif label == 'PMx':
            l_mean = -0.1519
            l_std = 0.0
            l_min = -0.1519
            l_25 = -0.1519
            l_50 = -0.1519
            l_75 = -0.1519
            l_max = -0.1519
        elif label == 'fuel':
            l_mean = -2.365283
            l_std = 0.0
            l_min = -2.365283
            l_25 = -2.365283
            l_50 = -2.365283
            l_75 = -2.365283
            l_max = -2.365283
        else:
            sys.exit(f"Unknown label {label}!")
    elif args.experiment == "cacc_single":
        if label == 'CO':
            l_mean = 54.971334
            l_std = 0.0
            l_min = 54.971334
            l_25 = 54.971334
            l_50 = 54.971334
            l_75 = 54.971334
            l_max = 54.971334
        elif label == 'CO2':
            l_mean = 2438.196675
            l_std = 0.0
            l_min = 2438.196675
            l_25 = 2438.196675
            l_50 = 2438.196675
            l_75 = 2438.196675
            l_max = 2438.196675
        elif label == 'HC':
            l_mean = 0.315344
            l_std = 0.0
            l_min = 0.315344
            l_25 = 0.315344
            l_50 = 0.315344
            l_75 = 0.315344
            l_max = 0.315344
        elif label == 'NOx':
            l_mean = 0.880814
            l_std = 0.0
            l_min = 0.880814
            l_25 = 0.880814
            l_50 = 0.880814
            l_75 = 0.880814
            l_max = 0.880814
        elif label == 'PMx':
            l_mean = 0.065361
            l_std = 0.0
            l_min = 0.065361
            l_25 = 0.065361
            l_50 = 0.065361
            l_75 = 0.065361
            l_max = 0.065361
        elif label == 'fuel':
            l_mean = 1.048084
            l_std = 0.0
            l_min = 1.048084
            l_25 = 1.048084
            l_50 = 1.048084
            l_75 = 1.048084
            l_max = 1.048084
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
        print(f"Deviation to Sumo in {label} exceeded limits ({args.experiment}, {args.vehicles}, {args.desired_speed})!")
        print(d)

# lifetime plots

lifetime_labels = ['speed', 'position', 'diff_desired', 'lane']

for label in lifetime_labels:

    data = merged_traces[label]

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
            # we might need to work on ACC, since there is some deviation
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
            print(f"Deviation to Sumo in {label} exceeded limits ({args.experiment}, {args.vehicles}, {args.desired_speed})!")
            print(d)
    elif label == 'lane':
        ax.hlines(0, 0, merged_traces.lifetime.max(), color='black', label='desired')
    else:
        sys.exit(f"Unknown label {label}!")

    fig.savefig('%s_%s.png' % (args.experiment, label))

# emission lifetime

for label in emission_labels:

    data = merged_emission_traces[label]

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

lifetime_diff_labels = ['diff_sumo_speed', 'diff_sumo_position', 'diff_sumo_lane']

# diff lifetime plots

for label in lifetime_diff_labels:

    data = merged_traces[label]

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
        # we might need to work on ACC, since there is some deviation
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
        print(f"Deviation to Sumo in {label} exceeded limits ({args.experiment}, {args.vehicles}, {args.desired_speed})!")
        print(d)

# emission lifetime diff plots

lifetime_diff_emission_labels = ['diff_sumo_CO', 'diff_sumo_CO2', 'diff_sumo_HC', 'diff_sumo_NOx', 'diff_sumo_PMx', 'diff_sumo_fuel']

for label in lifetime_diff_emission_labels:

    data = merged_emission_traces[label]

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

    d = data.describe()

    # absolute values might be misleading here, since the absolute values are in mg and rather high
    if args.experiment == "cc":
        if label == 'diff_sumo_CO':
            l_mean = 14.196172
            l_std = 126.422998
            l_min = -876.603311
            l_25 = -69.078238
            l_50 = 24.272504
            l_75 = 79.784762
            l_max = 910.777204
        elif label == 'diff_sumo_CO2':
            l_mean = 455.115576
            l_std = 3722.970459
            l_min = -34258.020000
            l_25 = -1462.204153
            l_50 = 535.713858
            l_75 = 1615.265480
            l_max = 38814.416256
        elif label == 'diff_sumo_HC':
            l_mean = 0.076900
            l_std = 0.669757
            l_min = -4.849464
            l_25 = -0.355834
            l_50 = 0.123003
            l_75 = 0.415663
            l_max = 5.138938
        elif label == 'diff_sumo_NOx':
            l_mean = 0.195217
            l_std = 1.635172
            l_min = -15.721288
            l_25 = -0.605726
            l_50 = 0.221594
            l_75 = 0.689985
            l_max = 17.291612
        elif label == 'diff_sumo_PMx':
            l_mean = 0.014630
            l_std = 0.118438
            l_min = -0.972935
            l_25 = -0.056999
            l_50 = 0.019423
            l_75 = 0.066654
            l_max = 1.064426
        elif label == 'diff_sumo_fuel':
            l_mean = 0.195474
            l_std = 1.600295
            l_min = -14.730000
            l_25 = -0.624503
            l_50 = 0.229499
            l_75 = 0.689416
            l_max = 16.684454
        else:
            sys.exit(f"Unknown label {label}!")
    elif args.experiment == "acc":
        # we might need to work on ACC, since there is some deviation
        if label == 'diff_sumo_CO':
            l_mean = 61.140744
            l_std = 147.912688
            l_min = -959.750000
            l_25 = -16.083763
            l_50 = 45.820507
            l_75 = 130.795609
            l_max = 986.948724
        elif label == 'diff_sumo_CO2':
            l_mean = 2068.880022
            l_std = 5839.800377
            l_min = -40182.260000
            l_25 = -470.250779
            l_50 = 985.826695
            l_75 = 2988.004917
            l_max = 40932.716004
        elif label == 'diff_sumo_HC':
            l_mean = 0.333900
            l_std = 0.819473
            l_min = -5.400000
            l_25 = -0.084487
            l_50 = 0.237836
            l_75 = 0.680822
            l_max = 5.542283
        elif label == 'diff_sumo_NOx':
            l_mean = 0.857386
            l_std = 2.523093
            l_min = -17.920000
            l_25 = -0.199512
            l_50 = 0.421061
            l_75 = 1.253855
            l_max = 18.258061
        elif label == 'diff_sumo_PMx':
            l_mean = 0.062289
            l_std = 0.163411
            l_min = -1.110000
            l_25 = -0.016572
            l_50 = 0.036651
            l_75 = 0.115985
            l_max = 1.134658
        elif label == 'diff_sumo_fuel':
            l_mean = 0.890441
            l_std = 2.509930
            l_min = -17.270000
            l_25 = -0.197876
            l_50 = 0.419964
            l_75 = 1.283464
            l_max = 17.595019
        else:
            sys.exit(f"Unknown label {label}!")
    elif args.experiment == "cacc":
        # only deviation at the end due to leaving the platoon
        # we might need to work on CACC, since there is some deviation
        # the absolute values here might not be useful
        # from the SUMO wiki: When running with sub-second resolution, the emissions written during every simulation step are extrapolated to those that would be generated in 1 second. Thus, care must be taken when computing sums over time.
        # https://sumo.dlr.de/docs/Simulation/Output/EmissionOutput.html
        if label == 'diff_sumo_CO':
            # Plexe step size 1s
            # Plexe step size 0.1s
            l_mean = 5.891669
            l_std = 32.330520
            l_min = -0.002222
            l_25 = -0.002222
            l_50 = -0.002222
            l_75 = -0.002222
            l_max = 183.237778
        elif label == 'diff_sumo_CO2':
            # Plexe step size 1s
            # Plexe step size 0.1s
            l_mean = 261.416445
            l_std = 1433.968996
            l_min = 0.002222
            l_25 = 0.002222
            l_50 = 0.002222
            l_75 = 0.002222
            l_max = 8127.322222
        elif label == 'diff_sumo_HC':
            # Plexe step size 1s
            # Plexe step size 0.1s
            l_mean = 0.034918
            l_std = 0.185260
            l_min = 0.001144
            l_25 = 0.001144
            l_50 = 0.001144
            l_75 = 0.001144
            l_max = 1.051144
        elif label == 'diff_sumo_NOx':
            # Plexe step size 1s
            # Plexe step size 0.1s
            l_mean = 0.090609
            l_std = 0.518728
            l_min = -0.003956
            l_25 = -0.003956
            l_50 = -0.003956
            l_75 = -0.003956
            l_max = 2.936044
        elif label == 'diff_sumo_PMx':
            # Plexe step size 1s
            # Plexe step size 0.1s
            l_mean = 0.004948
            l_std = 0.038816
            l_min = -0.002128
            l_25 = -0.002128
            l_50 = -0.002128
            l_75 = -0.002128
            l_max = 0.217872
        elif label == 'diff_sumo_fuel':
            # Plexe step size 1s
            # Plexe step size 0.1s
            l_mean = 0.115870
            l_std = 0.615769
            l_min = 0.003615
            l_25 = 0.003615
            l_50 = 0.003615
            l_75 = 0.003615
            l_max = 3.493615
        else:
            sys.exit(f"Unknown label {label}!")
    elif args.experiment == "cc_single":
        if label == 'diff_sumo_CO':
            l_mean = -0.002204
            l_std = 0.000320
            l_min = -0.003333
            l_25 = -0.002222
            l_50 = -0.002222
            l_75 = -0.002222
            l_max = 0.004444
        elif label == 'diff_sumo_CO2':
            l_mean = 0.002205
            l_std = 0.000312
            l_min = -0.004444
            l_25 = 0.002222
            l_50 = 0.002222
            l_75 = 0.002222
            l_max = 0.003056
        elif label == 'diff_sumo_HC':
            l_mean = 0.001142
            l_std = 0.000191
            l_min = -0.004028
            l_25 = 0.001144
            l_50 = 0.001144
            l_75 = 0.001144
            l_max = 0.004144
        elif label == 'diff_sumo_NOx':
            l_mean = -0.003931
            l_std = 0.000394
            l_min = -0.003956
            l_25 = -0.003956
            l_50 = -0.003956
            l_75 = -0.003956
            l_max = 0.004497
        elif label == 'diff_sumo_PMx':
            l_mean = -0.002118
            l_std = 0.000279
            l_min = -0.004910
            l_25 = -0.002128
            l_50 = -0.002128
            l_75 = -0.002128
            l_max = 0.004189
        elif label == 'diff_sumo_fuel':
            l_mean = 0.003597
            l_std = 0.000329
            l_min = -0.004485
            l_25 = 0.003615
            l_50 = 0.003615
            l_75 = 0.003615
            l_max = 0.004885
        else:
            sys.exit(f"Unknown label {label}!")
    elif args.experiment == "acc_single":
        if label == 'diff_sumo_CO':
            l_mean = -0.002204
            l_std = 0.000320
            l_min = -0.003333
            l_25 = -0.002222
            l_50 = -0.002222
            l_75 = -0.002222
            l_max = 0.004444
        elif label == 'diff_sumo_CO2':
            l_mean = 0.002205
            l_std = 0.000312
            l_min = -0.004444
            l_25 = 0.002222
            l_50 = 0.002222
            l_75 = 0.002222
            l_max = 0.003056
        elif label == 'diff_sumo_HC':
            l_mean = 0.001142
            l_std = 0.000191
            l_min = -0.004028
            l_25 = 0.001144
            l_50 = 0.001144
            l_75 = 0.001144
            l_max = 0.004144
        elif label == 'diff_sumo_NOx':
            l_mean = -0.003931
            l_std = 0.000394
            l_min = -0.003956
            l_25 = -0.003956
            l_50 = -0.003956
            l_75 = -0.003956
            l_max = 0.004497
        elif label == 'diff_sumo_PMx':
            l_mean = -0.002118
            l_std = 0.000279
            l_min = -0.004910
            l_25 = -0.002128
            l_50 = -0.002128
            l_75 = -0.002128
            l_max = 0.004189
        elif label == 'diff_sumo_fuel':
            l_mean = 0.003597
            l_std = 0.000329
            l_min = -0.004485
            l_25 = 0.003615
            l_50 = 0.003615
            l_75 = 0.003615
            l_max = 0.004885
        else:
            sys.exit(f"Unknown label {label}!")
    elif args.experiment == "cacc_single":
        if label == 'diff_sumo_CO':
            l_mean = -0.002222
            l_std = 0.000000
            l_min = -0.002222
            l_25 = -0.002222
            l_50 = -0.002222
            l_75 = -0.002222
            l_max = -0.002222
        elif label == 'diff_sumo_CO2':
            l_mean = 0.002222
            l_std = 0.000000
            l_min = 0.002222
            l_25 = 0.002222
            l_50 = 0.002222
            l_75 = 0.002222
            l_max = 0.002222
        elif label == 'diff_sumo_HC':
            l_mean = 0.001144
            l_std = 0.000000
            l_min = 0.001144
            l_25 = 0.001144
            l_50 = 0.001144
            l_75 = 0.001144
            l_max = 0.001144
        elif label == 'diff_sumo_NOx':
            l_mean = -0.003956
            l_std = 0.000000
            l_min = -0.003956
            l_25 = -0.003956
            l_50 = -0.003956
            l_75 = -0.003956
            l_max = -0.003956
        elif label == 'diff_sumo_PMx':
            l_mean = -2.127778e-03
            l_std = 4.337589e-19
            l_min = -2.127778e-03
            l_25 = -2.127778e-03
            l_50 = -2.127778e-03
            l_75 = -2.127778e-03
            l_max = -2.127778e-03
        elif label == 'diff_sumo_fuel':
            l_mean = 3.614855e-03
            l_std = 4.337589e-19
            l_min = 3.614855e-03
            l_25 = 3.614855e-03
            l_50 = 3.614855e-03
            l_75 = 3.614855e-03
            l_max = 3.614855e-03
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
        print(f"Deviation to Sumo in {label} exceeded limits ({args.experiment}, {args.vehicles}, {args.desired_speed})!")
        print(d)

if error:
    sys.exit("There was at least one deviation error!")
