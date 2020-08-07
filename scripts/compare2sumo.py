#!/usr/bin/python3

import matplotlib.pyplot as pl
import pandas
import re
from mpl_toolkits.axes_grid1.inset_locator import inset_axes
from statistics import mean


desiredSpeed = 36  # TODO read from output
arrivalPosition = 100000

## Read trips/emissions

sumo_trips = pandas.read_csv('static-trips.csv')

sumo_trips.rename(columns=lambda x: re.sub('tripinfo_', '', x), inplace=True)
sumo_trips.rename(columns=lambda x: re.sub('emissions_', '', x), inplace=True)

sumo_trips.replace('static\.', '', regex=True, inplace=True)
sumo_trips.replace('edge_0_0_', '', regex=True, inplace=True)

sumo_trips = sumo_trips.astype({'arrivalLane': int, 'departLane': int, 'id': int, 'vType': str})

sumo_trips.sort_values(by='id', inplace=True)

plafosim_trips = pandas.read_csv('results_vehicle_trips.csv')
plafosim_trips.sort_values(by='id', inplace=True)

plafosim_emissions = pandas.read_csv('results_vehicle_emissions.csv')
plafosim_emissions.sort_values(by='id', inplace=True)

ids = frozenset(sumo_trips['id'].unique()).intersection(plafosim_trips['id'].unique())

## Evalute runtime

print("Evaluating runtime...")

runtimes = pandas.read_csv('runtimes.csv')
runtimes = runtimes.astype({'tool': str})

print("Plotting runtime...")

pl.figure()
pl.title("Runtime for %d Vehicles" % len(ids))
data = runtimes.loc[runtimes.tool == "sumo", runtimes.columns != "tool"].values.flatten()

pl.scatter(range(0, len(data), 1), data, label="sumo", color="black")
data = runtimes.loc[runtimes.tool == "plafosim", runtimes.columns != "tool"].values.flatten()
pl.scatter(range(0, len(data), 1), data, label="plafosim", color="blue")

pl.ylabel("time [s]")
pl.xticks(range(0, len(data), 1), runtimes.loc[:, runtimes.columns != "tool"])
pl.xlabel("kind")
pl.legend()
pl.savefig('runtime.png')

## Read traces

sumo_traces = pandas.read_csv('static-traces.csv', usecols=['timestep_time', 'vehicle_id', 'vehicle_lane', 'vehicle_pos', 'vehicle_speed'])
sumo_traces.columns = ['step', 'id', 'lane', 'position', 'speed']

sumo_traces.dropna(inplace=True)

sumo_traces.replace('static\.', '', regex=True, inplace=True)
sumo_traces.replace('edge_0_0_', '', regex=True, inplace=True)

sumo_traces = sumo_traces.astype({'step': int, 'id': int, 'lane': int})

sumo_traces.sort_values(by='step', inplace=True)

plafosim_traces = pandas.read_csv('results_vehicle_traces.csv', usecols=['step', 'id', 'position', 'lane', 'speed'])
plafosim_traces.sort_values(by='step', inplace=True)

## Read lane-changes

sumo_changes = pandas.read_csv('static-changes.csv', usecols=['change_from', 'change_id', 'change_pos', 'change_reason', 'change_speed', 'change_time', 'change_to'])
sumo_changes.columns = ['from', 'id', 'position', 'reason', 'speed', 'step', 'to']

sumo_changes.dropna(inplace=True)

sumo_changes.replace('static\.', '', regex=True, inplace=True)
sumo_changes.replace('edge_0_0_', '', regex=True, inplace=True)

sumo_changes = sumo_changes.astype({'step': int, 'id': int, 'from': int, 'to': int})

sumo_changes.sort_values(by='step', inplace=True)

plafosim_changes = pandas.read_csv('results_vehicle_changes.csv')
plafosim_changes.sort_values(by='step', inplace=True)

## Evaluate trips/emissions/traces/lane changes

print("Evaluating trips/emissions/traces/changes...")

### Evaluate for every vehicle

# create empty dicts
diff_trips = {}

speeds_lifetime = {}
speeds_lifetime['sumo'] = {}
speeds_lifetime['plafosim'] = {}
positions_lifetime = {}
positions_lifetime['sumo'] = {}
positions_lifetime['plafosim'] = {}
diff_desired_lifetime = {}
diff_desired_lifetime['sumo'] = {}
diff_desired_lifetime['plafosim'] = {}

diff_speeds_lifetime = {}
diff_positions_lifetime = {}
diff_lanes_lifetime = {}

# TODO use multi-threading to parallelize execution

for vid in list(ids):

    print("Current vehicle %d" % vid)

    # create empty sub-dicts for this vehicle
    diff_trips[vid] = {}

    speeds_lifetime['sumo'][vid] = {}
    speeds_lifetime['plafosim'][vid] = {}
    positions_lifetime['sumo'][vid] = {}
    positions_lifetime['plafosim'][vid] = {}
    diff_desired_lifetime['sumo'][vid] = {}
    diff_desired_lifetime['plafosim'][vid] = {}

    diff_speeds_lifetime[vid] = {}
    diff_positions_lifetime[vid] = {}
    diff_lanes_lifetime[vid] = {}

    trips_sumo = sumo_trips.loc[sumo_trips.id == vid].reset_index(drop=True)
    trips_plafosim = plafosim_trips[plafosim_trips.id == vid].reset_index(drop=True)
    ep = plafosim_emissions[plafosim_emissions.id == vid].reset_index(drop=True)

    traces_sumo = sumo_traces.loc[sumo_traces.id == vid].reset_index(drop=True)
    traces_plafosim = plafosim_traces[plafosim_traces.id == vid].reset_index(drop=True)

    if len(trips_sumo) != 1 or len(trips_plafosim) != 1 or len(ep) != 1:
        print("Vehicle does not exist in all data sets")
        exit(1)

    # depart time
    # should not be different
    if int(trips_sumo.depart) != int(trips_plafosim.depart):
        print("depart time is not the same %d vs. %d" % (trips_sumo.depart, trips_plafosim.depart))
        exit(1)

    # depart lane
    # should not be different
    if round(float(trips_sumo.departLane)) != int(trips_plafosim.departLane):
        print("depart lane is not the same %d vs. %d" % (trips_sumo.departLane, trips_plafosim.departLane))
        exit(1)

    # depart position
    # should not be different
    # currently, there is a diff of 10 cm which we ignore
    if float(trips_sumo.departPos) != float(trips_plafosim.departPos):
        print("depart pos is not the same %f vs. %f" % (trips_sumo.departPos, trips_plafosim.departPos))

    # depart speed
    # should not be different
    if float(trips_sumo.departSpeed) != float(trips_plafosim.departSpeed):
        print("depart speed is not the same %f vs. %f" % (trips_sumo.departSpeed, trips_plafosim.departSpeed))
        exit(1)

    # arrival position
    # should not be different
    if float(trips_sumo.arrivalPos) != float(trips_plafosim.arrivalPos):
        print("arrival pos it not the same %f vs. %f" % (trips_sumo.arrivalPos, trips_plafosim.arrivalPos))
        exit(1)

    # route length
    # should not be different
    # currently, there is a diff of 10 cm which we ignore
    if round(float(trips_sumo.routeLength)) != float(trips_plafosim.routeLength):
        print("route length is not the same %f vs. %f" % (trips_sumo.routeLength, trips_plafosim.routeLength))
        exit(1)

    # speed factor aka desired drving speed
    desired_speed_sumo = float(trips_sumo.speedFactor) * desiredSpeed
    desired_speed_plafosim = float(trips_plafosim.desiredSpeed)
    diff_trips[vid]['desiredSpeed'] = desired_speed_plafosim - desired_speed_sumo

    # arrivel time
    # can be different
    diff_trips[vid]['arrival'] = float(trips_plafosim.arrival) - float(trips_sumo.arrival)

    # arrival lane
    # can be different
    diff_trips[vid]['arrivalLane'] = int(trips_plafosim.arrivalLane) - int(trips_sumo.arrivalLane)

    # arrival speed
    # can be different
    diff_trips[vid]['arrivalSpeed'] = float(trips_plafosim.arrivalSpeed) - float(trips_sumo.arrivalSpeed)

    # trip duration
    # can be different
    diff_trips[vid]['duration'] = float(trips_plafosim.duration) - float(trips_sumo.duration)

    # time loss
    # can be different
    diff_trips[vid]['timeLoss'] = float(trips_plafosim.timeLoss) - float(trips_sumo.timeLoss)

    # co
    # can be different
    diff_trips[vid]['co'] = float(ep.CO) - float(trips_sumo.CO_abs)

    # co2
    # can be different
    diff_trips[vid]['co2'] = float(ep.CO2) - float(trips_sumo.CO2_abs)

    # hc
    # can be different
    diff_trips[vid]['hc'] = float(ep.HC) - float(trips_sumo.HC_abs)

    # pmx
    # can be different
    diff_trips[vid]['pmx'] = float(ep.PMx) - float(trips_sumo.PMx_abs)

    # nox
    # can be different
    diff_trips[vid]['nox'] = float(ep.NOx) - float(trips_sumo.NOx_abs)

    # fuel
    # can be different
    diff_trips[vid]['fuel'] = float(ep.fuel) - float(trips_sumo.fuel_abs)

    # determine data interval for this vehicle
    min_step = int(min(traces_sumo.step.min(), traces_plafosim.step.min()))
    assert(min_step == int(trips_sumo.depart))
    max_step = int(max(traces_sumo.step.max(), traces_plafosim.step.max()))

    # go trough lifetime of vehicle
    for step in range(min_step, max_step, 1):

        # get data for this vehicle in this step
        step_sumo = traces_sumo.loc[traces_sumo.step == step].reset_index(drop=True)
        step_plafosim = traces_plafosim.loc[traces_plafosim.step == step].reset_index(drop=True)

        # caluclate lifetime
        life_time = step - min_step

        if len(step_sumo) is 1:
            speeds_lifetime['sumo'][vid][life_time] = float(step_sumo.speed)
            positions_lifetime['sumo'][vid][life_time] = float(step_sumo.position)
            diff_desired_lifetime['sumo'][vid][life_time] = float(step_sumo.speed) - desired_speed_sumo

        if len(step_plafosim) is 1:
            speeds_lifetime['plafosim'][vid][life_time] = float(step_plafosim.speed)
            positions_lifetime['plafosim'][vid][life_time] = float(step_plafosim.position)
            diff_desired_lifetime['plafosim'][vid][life_time] = float(step_plafosim.speed) - desired_speed_plafosim

        if len(step_sumo) is len(step_plafosim) is 1:
            diff_speeds_lifetime[vid][life_time] = float(step_plafosim.speed - step_sumo.speed)
            diff_positions_lifetime[vid][life_time] = float(step_plafosim.position - step_sumo.position)
            diff_lanes_lifetime[vid][life_time] = int(step_plafosim.lane - step_sumo.lane)

### Plotting

print("Plotting trips/emissions/traces...")

# boxplot with sumo and plafosim showing the picked desired driving speed
data_sumo = [float(speedFactor) * desiredSpeed for speedFactor in sumo_trips.speedFactor.values]
data_plafosim = plafosim_trips.desiredSpeed.values

pl.figure()
pl.title("Desired Driving Speed for %d Vehicles" % len(ids))
pl.boxplot([data_sumo, data_plafosim], showmeans=True, labels=['sumo', 'plasfosim'])
pl.ylabel("speed [m/s]")
pl.savefig('desired_speed.png')

# devation to sumo in desired speed
data = [diff_trips[x]['desiredSpeed'] for x in diff_trips.keys()]
pl.figure()
pl.title("Deviation to Sumo in Desired Speed for %d Vehicles" % len(ids))
pl.boxplot(data, showmeans=True)
pl.ylabel("speed [m/s]")
pl.savefig('diff_desired_box.png')

# devation to sumo in arrival time
data = [diff_trips[x]['arrival'] for x in diff_trips.keys()]
pl.figure()
pl.title("Deviation to Sumo in Arrival Time for %d Vehicles" % len(ids))
pl.boxplot(data, showmeans=True)
pl.ylabel("time [s]")
pl.savefig('diff_arrival_box.png')

# devation to sumo in arrival lane
data = [diff_trips[x]['arrivalLane'] for x in diff_trips.keys()]
pl.figure()
pl.title("Deviation to Sumo in Arrival Lane for %d Vehicles" % len(ids))
pl.boxplot(data, showmeans=True)
pl.xlabel("Deviation to SUMO in arrival lane")
pl.ylabel("lane")
pl.savefig('diff_arrivalLane_box.png')

# deviation to sumo in arrival speed
data = [diff_trips[x]['arrivalSpeed'] for x in diff_trips.keys()]
pl.figure()
pl.title("Deviation to Sumo in Arrival Speed for %d Vehicles" % len(ids))
pl.boxplot(data, showmeans=True)
pl.ylabel("speed [m/s]")
pl.savefig('diff_arrivalSpeed_box.png')

# deviation to sumo in trip duration
data = [diff_trips[x]['duration'] for x in diff_trips.keys()]
pl.figure()
pl.title("Deviation to Sumo in Trip Duration for %d Vehicles" % len(ids))
pl.boxplot(data, showmeans=True)
pl.ylabel("time [s]")
pl.savefig('diff_duration_box.png')

# devation to sumo in time loss
## TODO no time departure delay in plafosim (yet)
#data = [diff_trips[x]['timeLoss'] for x in diff_trips.keys()]
#pl.figure()
#pl.boxplot(data, showmeans=True)
#pl.xlabel("Deviation to SUMO in time loss")
#pl.ylabel("time [s]")
#pl.savefig('diff_timeLoss_box.png')
#
# devation to sumo in co
## TODO no fuel model yet
#data = [diff_trips[x]['co'] for x in diff_trips.keys()]
#pl.figure()
#pl.boxplot(data, showmeans=True)
#pl.xlabel("Deviation to SUMO in co")
#pl.ylabel("mg")
#pl.savefig('diff_co_box.png')
#
# devation to sumo in co2
## TODO no fuel model yet
#data = [diff_trips[x]['co2'] for x in diff_trips.keys()]
#pl.figure()
#pl.boxplot(data, showmeans=True)
#pl.xlabel("Deviation to SUMO in co2")
#pl.ylabel("mg")
#pl.savefig('diff_co2_box.png')
#
# devation to sumo in hc
## TODO no fuel model yet
#data = [diff_trips[x]['hc'] for x in diff_trips.keys()]
#pl.figure()
#pl.boxplot(data, showmeans=True)
#pl.xlabel("Deviation to SUMO in hc")
#pl.ylabel("mg")
#pl.savefig('diff_hc_box.png')
#
# devation to sumo in pmx
## TODO no fuel model yet
#data = [diff_trips[x]['pmx'] for x in diff_trips.keys()]
#pl.figure()
#pl.boxplot(data, showmeans=True)
#pl.xlabel("Deviation to SUMO in pmx")
#pl.ylabel("mg")
#pl.savefig('diff_pmx_box.png')
#
# devation to sumo in nox
## TODO no fuel model yet
#data = [diff_trips[x]['nox'] for x in diff_trips.keys()]
#pl.figure()
#pl.boxplot(data, showmeans=True)
#pl.xlabel("Deviation to SUMO in nox")
#pl.ylabel("mg")
#pl.savefig('diff_nox_box.png')
#
# devation to sumo in fuel
## TODO no fuel model yet
#data = [diff_trips[x]['fuel'] for x in diff_trips.keys()]
#pl.figure()
#pl.boxplot(data, showmeans=True)
#pl.xlabel("Deviation to SUMO in fuel")
#pl.ylabel("ml")
#pl.savefig('diff_fuel_box.png')

### Speed in lifetime

# plot about time step and speed, containing 3 lines: desired, sumo, plafosim
fig = pl.figure()
pl.title("Average Driving Speed for %d Vehicles" % len(ids))
xlim = max(int(sumo_traces.step.max()), int(plafosim_traces.step.max()))
pl.hlines(desiredSpeed, 0, xlim, color="red", label="desired")
pl.ylim(0, desiredSpeed + 5)
pl.ylabel("speed [m/s]")
pl.xlim(0, xlim)
pl.xlabel("trip duration [s]")

x_sumo = sorted(set([step for vehicle in speeds_lifetime['sumo'].values() for step in vehicle.keys()]))
y_sumo = [mean([vehicle[step] for vehicle in speeds_lifetime['sumo'].values() if step in vehicle]) for step in x_sumo]
pl.plot(x_sumo, y_sumo, color="black", label="sumo")

x_plafosim = sorted(set([step for vehicle in speeds_lifetime['plafosim'].values() for step in vehicle.keys()]))
y_plafosim = [mean([vehicle[step] for vehicle in speeds_lifetime['plafosim'].values() if step in vehicle]) for step in x_plafosim]
pl.plot(x_plafosim, y_plafosim, color="blue", label="plafosim")

pl.legend(loc='lower right')

ax = fig.add_subplot(111)
ia = inset_axes(ax, width="50%", height=1., loc=5)
xlim = 60
pl.hlines(desiredSpeed, 0, xlim, color="red", label="desired")
pl.ylim(desiredSpeed - 5, desiredSpeed + 2)
pl.ylabel("speed [m/s]")
pl.xlim(0, xlim)
#pl.xlabel("trip duration [s]")

pl.plot(x_sumo, y_sumo, color="black", label="sumo")
pl.plot(x_plafosim, y_plafosim, color="blue", label="plafosim")

pl.savefig('speed_line.png')

### Position in lifetime

# plot about time step and pos, containing 3 lines: desired, sumo, plafosim
fig = pl.figure()
pl.title("Average Position for %d Vehicles" % len(ids))
xlim = max(int(sumo_traces.step.max()), int(plafosim_traces.step.max()))
pl.hlines(arrivalPosition, 0, xlim, color="red", label="destination")
pl.ylim(0, arrivalPosition + 5000)
pl.ylabel("position [m]")
pl.xlim(0, xlim)
pl.xlabel("trip duration [s]")

x_sumo = sorted(set([step for vehicle in positions_lifetime['sumo'].values() for step in vehicle.keys()]))
y_sumo = [mean([vehicle[step] for vehicle in positions_lifetime['sumo'].values() if step in vehicle]) for step in x_sumo]
pl.plot(x_sumo, y_sumo, color="black", label="sumo")

x_plafosim = sorted(set([step for vehicle in positions_lifetime['plafosim'].values() for step in vehicle.keys()]))
y_plafosim = [mean([vehicle[step] for vehicle in positions_lifetime['plafosim'].values() if step in vehicle]) for step in x_plafosim]
pl.plot(x_plafosim, y_plafosim, color="blue", label="plafosim")

pl.legend(loc='upper left')

ax = fig.add_subplot(111)
ia = inset_axes(ax, width="30%", height=1., loc=4)
pl.hlines(arrivalPosition, 0, xlim + 5, color="red", label="destination")
pl.ylim(arrivalPosition - 2500, arrivalPosition + 2500)
pl.ylabel("position [m/s]")
pl.xlim(xlim - 100, xlim + 5)
#pl.xlabel("trip duration [s]")

pl.plot(x_sumo, y_sumo, color="black", label="sumo")
pl.plot(x_plafosim, y_plafosim, color="blue", label="plafosim")

pl.savefig('position_line.png')

### Diff desired Speed in lifetime

# plot about time step and speed, containing 3 lines: desired, sumo, plafosim
fig = pl.figure()
pl.title("Average Deviation to Desired Driving Speed for %d Vehicles" % len(ids))
xlim = max(int(sumo_traces.step.max()), int(plafosim_traces.step.max()))
pl.hlines(0, 0, xlim, color="red", label="desired")
pl.ylim(-desiredSpeed-5, 5)
pl.ylabel("speed [m/s]")
pl.xlim(0, xlim)
pl.xlabel("trip duration [s]")

x_sumo = sorted(set([step for vehicle in diff_desired_lifetime['sumo'].values() for step in vehicle.keys()]))
y_sumo = [mean([vehicle[step] for vehicle in diff_desired_lifetime['sumo'].values() if step in vehicle]) for step in x_sumo]
pl.plot(x_sumo, y_sumo, color="black", label="sumo")

x_plafosim = sorted(set([step for vehicle in diff_desired_lifetime['plafosim'].values() for step in vehicle.keys()]))
y_plafosim = [mean([vehicle[step] for vehicle in diff_desired_lifetime['plafosim'].values() if step in vehicle]) for step in x_plafosim]
pl.plot(x_plafosim, y_plafosim, color="blue", label="plafosim")

pl.legend(loc='lower right')

ax = fig.add_subplot(111)
ia = inset_axes(ax, width="50%", height=1., loc=5)
xlim = 60
pl.hlines(0, 0, xlim, color="red", label="desired")
pl.ylim(-5, 2)
pl.ylabel("speed [m/s]")
pl.xlim(0, xlim)
#pl.xlabel("trip duration [s]")

pl.plot(x_sumo, y_sumo, color="black", label="sumo")
pl.plot(x_plafosim, y_plafosim, color="blue", label="plafosim")

pl.savefig('diff_desired_speed_line.png')

### deviation to sumo in speed (box)

pl.figure()
pl.title("Average Deviation to Sumo in Speed in Lifetime for %d Vehicles" % len(ids))

x = sorted(set([step for vehicle in diff_speeds_lifetime.values() for step in vehicle.keys()]))
y = [mean([vehicle[step] for vehicle in diff_speeds_lifetime.values() if step in vehicle]) for step in x]

pl.boxplot(y, showmeans=True)

pl.ylabel("diff in speed [m/s]")
pl.savefig('diff_speed_box.png')

### devation to sumo in speed (line)

pl.figure()
pl.title("Average Deviation to Sumo in Speed for %d Vehicles" % len(ids))

pl.plot(x, y)

pl.xlabel("trip duration [s]")
pl.ylabel("diff in speed [m/s]")
pl.savefig('diff_speed_line.png')

### deviation to sumo in position (box)

pl.figure()
pl.title("Average Deviation to Sumo in Position in Lifetime for %d Vehicles" % len(ids))

x = sorted(set([step for vehicle in diff_positions_lifetime.values() for step in vehicle.keys()]))
y = [mean([vehicle[step] for vehicle in diff_positions_lifetime.values() if step in vehicle]) for step in x]

pl.boxplot(y, showmeans=True)

pl.ylabel("diff in position [m]")
pl.savefig('diff_position_box.png')

### devation to sumo in position (line)

pl.figure()
pl.title("Average Deviation to Sumo in Position for %d Vehicles" % len(ids))

pl.plot(x, y)

pl.xlabel("trip duration [s]")
pl.ylabel("diff in position [m]")
pl.savefig('diff_position_line.png')

### deviation to sumo in lane (box)

pl.figure()
pl.title("Average Deviation to Sumo in Lane in Lifetime for %d Vehicles" % len(ids))

x = sorted(set([step for vehicle in diff_lanes_lifetime.values() for step in vehicle.keys()]))
y = [mean([vehicle[step] for vehicle in diff_lanes_lifetime.values() if step in vehicle]) for step in x]

pl.boxplot(y, showmeans=True)

pl.ylabel("diff in lane")
pl.savefig('diff_lane_box.png')

### devation to sumo in lane (line)

pl.figure()
pl.title("Average Deviation to Sumo in lane for %d Vehicles" % len(ids))

pl.plot(x, y)

pl.xlabel("trip duration [s]")
pl.ylabel("diff in lane")
pl.savefig('diff_lane_line.png')
