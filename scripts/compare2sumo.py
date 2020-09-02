#!/usr/bin/env python3

import argparse
import matplotlib.pyplot as pl
import pandas
import re
import seaborn


## Read parameters

class CustomFormatter(argparse.ArgumentDefaultsHelpFormatter,
                      argparse.RawDescriptionHelpFormatter,
                      argparse.MetavarTypeHelpFormatter):
    """Metaclass combining multiple formatter classes for argparse"""
    pass


parser = argparse.ArgumentParser(formatter_class=CustomFormatter, description="")
parser.add_argument('--experiment', type=str, default='human', help="The name of the experiment to use for all result files")
parser.add_argument('--desired-speed', type=float, default=36.0, help="The desired speed to use for the comparison")
parser.add_argument('--arrival-position', type=int, default=100000, help="The arrival position to use for the comparison")
args = parser.parse_args()

## Read runtimes

runtimes = pandas.read_csv('runtimes_%s.csv' % args.experiment)
runtimes = runtimes.astype({'simulator': str})
runtimes = runtimes.set_index('simulator')

## Read trips/emissions

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

## Read traces

sumo_traces = pandas.read_csv('%s-traces.csv' % args.experiment, usecols=['timestep_time', 'vehicle_id', 'vehicle_lane', 'vehicle_pos', 'vehicle_speed'])
sumo_traces.columns = ['step', 'id', 'lane', 'position', 'speed']
sumo_traces.dropna(inplace=True)
sumo_traces.replace(r'static\.', '', regex=True, inplace=True)
sumo_traces.replace('edge_0_0_', '', regex=True, inplace=True)
sumo_traces = sumo_traces.astype({'step': int, 'id': int, 'lane': int})
sumo_traces.sort_values(by='step', inplace=True)

plafosim_traces = pandas.read_csv('%s_vehicle_traces.csv' % args.experiment, usecols=['step', 'id', 'position', 'lane', 'speed'])
plafosim_traces.sort_values(by='step', inplace=True)

## Read lane-changes

sumo_changes = pandas.read_csv('%s-changes.csv' % args.experiment, usecols=['change_from', 'change_id', 'change_pos', 'change_reason', 'change_speed', 'change_time', 'change_to'])
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

## Evalute runtime

pl.figure()
pl.title("Runtime for %d Vehicles" % number_of_vehicles)
data = runtimes.reset_index().melt('simulator', var_name='kind').set_index('simulator')
# does not work because of imcompatibility between matplotlib and seaborn
#seaborn.scatterplot(data=data, x='kind', y='value', hue='simulator')
pl.scatter(data=data.loc['sumo'], x='kind', y='value', label='sumo')
pl.scatter(data=data.loc['plafosim'], x='kind', y='value', label='plafosim')
pl.ylabel("time [s]")
pl.legend()
pl.savefig('%s_runtime.png' % args.experiment)

## Evaluate trips

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

## Evaluate emissions

# TODO emissions
#emission_labels =['co', 'co2', 'hc', 'pmx', 'nox', 'fuel']
#print(sumo_trips.head())
#diff_emissions = plafosim_emissions[emission_labels] - sumo_trips[emission_labels]
#print(diff_emissions)

## Evaluate traces

sumo_traces = sumo_traces.set_index(['id', 'step'], drop=False).sort_index()
plafosim_traces = plafosim_traces.set_index(['id', 'step'], drop=False).sort_index()

plafosim_traces = plafosim_traces.assign(lifetime=lambda x: x.step - x.groupby(level='id').step.min(),
                                         diff_desired=lambda x: x.speed - plafosim_trips.desiredSpeed,
                                         diff_sumo_speed=lambda x: x.speed - sumo_traces.speed,
                                         diff_sumo_position=lambda x: x.position - sumo_traces.position,
                                         diff_sumo_lane=lambda x: abs(x.lane - sumo_traces.lane)
                                         ).reset_index(drop=True)

sumo_traces = sumo_traces.assign(lifetime=lambda x: x.step - x.groupby(level='id').step.min(),
                                 diff_desired=lambda x: x.speed - sumo_trips.speedFactor * args.desired_speed).reset_index(drop=True)

sumo_traces = sumo_traces.set_index(['id', 'lifetime']).sort_index()
plafosim_traces = plafosim_traces.set_index(['id', 'lifetime']).sort_index()

merged_traces = pandas.concat([sumo_traces, plafosim_traces], keys=['sumo', 'plafosim'], names=['simulator']).reset_index()

## Plotting

print("Plotting trips/emissions/traces...")

# boxplot with desired driving speed

pl.figure()
pl.title("Desired Driving Speed for %d Vehicles" % number_of_vehicles)
pl.boxplot([sumo_trips.desiredSpeed, plafosim_trips.desiredSpeed], showmeans=True, labels=['sumo', 'plasfosim'])
#seaborn.boxplot(x=['sumo', 'plafosim'], y=[sumo_trips.desiredSpeed, plafosim_trips.desiredSpeed], showmeans=True)
pl.xlabel("simulator")
pl.ylabel("speed [m/s]")
pl.savefig('%s_desired_speed.png' % args.experiment)

# distplot with desired driving speed

fig, ax = pl.subplots()
pl.title("Desired Driving Speed for %d Vehicles" % number_of_vehicles)
seaborn.distplot(
    sumo_trips.desiredSpeed,
    hist=False,
    kde_kws={'cumulative': True},
    label='sumo',
    ax=ax
)
seaborn.distplot(
    plafosim_trips.desiredSpeed,
    hist=False,
    kde_kws={'cumulative': True},
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
    seaborn.distplot(
        data,
        hist=False,
        kde_kws={'cumulative': True},
    )
    pl.savefig('%s_diff_%s_dist.png' % (args.experiment, label))

    # check limits for deviation to sumo
    d = data.describe()

    if label == 'desiredSpeed':
        al_qst = 4.5
        al_mean = 1
        al_std = 6
        al_median = 2
        al_qrd = 4.5
    elif label == 'arrival':
        al_qst = 400
        al_mean = 160
        al_std = 460
        al_median = 160
        al_qrd = 300
    elif label == 'arrivalLane':
        al_qst = 0
        al_mean = 0.25
        al_std = 0.8
        al_median = 0
        al_qrd = 0
    elif label == 'arrivalSpeed':
        al_qst = 4.5
        al_mean = 1.5
        al_std = 6
        al_median = 2
        al_qrd = 5
    elif label == 'duration':
        al_qst = 400
        al_mean = 160
        al_std = 460
        al_median = 160
        al_qrd = 300
    elif label == 'timeLoss':
        al_qst = 150
        al_mean = 90
        al_std = 140
        al_median = 60
        al_qrd = 50

    if abs(d['25%']) > al_qst or abs(d['mean']) > al_mean or abs(d['std']) > al_std or abs(d['50%']) > al_median or abs(d['75%']) > al_qrd:
        error = True
        print("Deviation to Sumo in %s exceeded limits!" % label)
        print(d)

# TODO emissions

#for label in emission_labels:
#    idata = diff_emissions[label]
#
#    pl.figure()
#    pl.title("Deviation to Sumo in %s for %d Vehicles" % (label, number_of_vehicles))
#    pl.boxplot(data, showmeans=True)
#    pl.savefig('%s_diff_%s_box.png' % (args.experiment, label))
#
#    pl.figure()
#    pl.title("Deviation to Sumo in %s for %d Vehicles" % (label, number_of_vehicles))
#    seaborn.distplot(
#        data,
#        hist=False,
#        kde_kws={'cumulative': True},
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
            ax=ax
        )
    elif label == 'diff_desired':
        ax.hlines(0, 0, merged_traces.lifetime.max(), color='black', label='desired')

        # check limits for deviation in desired speed
        d = data.describe()
        if abs(d['25%']) > 2 or abs(d['mean']) > 1 or abs(d['std']) > 3 or abs(d['50%']) > 0 or abs(d['75%']) > 0:
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

    if label == 'diff_sumo_speed':
        al_qst = 4
        al_mean = 2
        al_std = 6
        al_median = 1.5
        al_qrd = 4.5
    elif label == 'diff_sumo_position':
        al_qst = 4000
        al_mean = 2500
        al_std = 7800
        al_median = 1200
        al_qrd = 5100
    elif label == 'diff_sumo_lane':
        al_qst = 0
        al_mean = 0.7
        al_std = 0.8
        al_median = 0
        al_qrd = 1

    if abs(d['25%']) > al_qst or abs(d['mean']) > al_mean or abs(d['std']) > al_std or abs(d['50%']) > al_median or abs(d['75%']) > al_qrd:
        error = True
        print("Deviation to Sumo in %s exceeded limits!" % lal)
        print(d)

if error:
    print("There was at least one deviation error!")
    exit(1)
